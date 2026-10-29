// SPDX-License-Identifier: AGPL-3.0-only
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use cerulion_bagd::{run_bagd, BagdConfig, BagdSummary, RecordedTopic, RECORDED_TOPIC_FORMAT};
use cerulion_cli_engine::graph_cmd::{cdylib_name, resolve_recorded_topics};
use cerulion_cli_engine::schema_cmd::parse_builtin_schemas;
use cerulion_core::clock::{Clock, RealClock};
use cerulion_core::dynamic::{SchemaSet, WireLayout};
use cerulion_core::graph::{
    parse_graph, resolve_output_topic, DylibNodeEntry, GraphConfig, NodeEntry, NodeTickPanicRecords,
};
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, SchemaServing, TopicSchema,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{
    recording_tap_buffer_depth, TopicServiceConfig, TransportConfig, TransportKind,
    TransportManager, RECORDING_SUBSCRIBER_MAX_BORROWED, SUBSCRIBER_MAX_BORROWED_HELD,
    TRANSPORT_KIND,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{CerulionPublisher, GraphRuntime, LanAccess};
use cerulion_mdns::MdnsAdvertiseGuard;
use iceoryx2::config::Config as IoxConfig;
use iceoryx2::prelude::{FilePath, SemanticString};
use serde::Serialize;

use crate::config::{validate_robot_name, EmbeddedConfig};
use crate::encode::encode_json_into;
use crate::error::{codes, EmbedError};

type ProcessTransportIdentity = (Vec<String>, String);

static PROCESS_TRANSPORT_IDENTITY: OnceLock<Mutex<Option<ProcessTransportIdentity>>> =
    OnceLock::new();

/// Set while one [`EmbeddedRuntime`] in this process owns the network
/// gateway. See [`GatewayLease`].
static GATEWAY_OWNED: AtomicBool = AtomicBool::new(false);

/// Exclusive ownership of the gateway on the process-wide transport manager.
///
/// Every runtime with the same identity shares one manager, and its
/// announcements and bridge flags carry no owner. A gateway start resets
/// all of them to the quiescent baseline (see [`quiesce`]), which is correct
/// only when no other runtime has a live gateway on that manager, so a
/// second runtime with listen endpoints is refused while one holds this
/// lease.
struct GatewayLease(());

impl GatewayLease {
    fn acquire() -> Result<Self, EmbedError> {
        GATEWAY_OWNED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| Self(()))
            .map_err(|_| {
                EmbedError::BadState(
                    "another embedded runtime in this process owns the network gateway; stop it before starting a runtime with listen endpoints".to_string(),
                )
            })
    }
}

impl Drop for GatewayLease {
    fn drop(&mut self) {
        GATEWAY_OWNED.store(false, Ordering::Release);
    }
}

/// TEST SEAM: whether a runtime in this process holds the gateway lease.
#[cfg(any(test, feature = "test-helpers"))]
#[doc(hidden)]
pub fn gateway_owned_for_test() -> bool {
    GATEWAY_OWNED.load(Ordering::Acquire)
}

/// A gateway runtime's hold on the shared manager. Taking it claims the
/// [`GatewayLease`] and then resets the manager to the quiescent baseline;
/// dropping it resets the manager again and only then frees the lease, so
/// no other runtime can start a gateway before the reset is done.
struct GatewayOwnership {
    manager: Arc<TransportManager>,
    // Declared last: fields drop in declaration order, so the lease is
    // released after everything else this hold owns.
    _lease: GatewayLease,
}

impl GatewayOwnership {
    fn acquire(manager: &Arc<TransportManager>) -> Result<Self, EmbedError> {
        let lease = GatewayLease::acquire()?;
        quiesce(manager);
        Ok(Self {
            manager: Arc::clone(manager),
            _lease: lease,
        })
    }
}

impl Drop for GatewayOwnership {
    fn drop(&mut self) {
        quiesce(&self.manager);
    }
}

/// Reset the manager-wide gateway state to the quiescent baseline: LAN
/// access paired, no egress topic announced (the bare robot-identity token
/// stays), no bridge flag registered and the latched schema serving empty.
/// Idempotent; it logs a failure and never returns one, so it can run on
/// every teardown path. Call it only while no gateway drives the manager.
fn quiesce(manager: &TransportManager) {
    if let Some(network) = manager.network() {
        network.set_lan_access(LanAccess::Paired);
        // Paired already withdraws every announce; this repairs a teardown
        // that stopped part-way.
        network.withdraw_announcements_except(&[]);
    }
    if let Err(error) = manager.bridge_manager().unregister_topics_except(&[]) {
        tracing::warn!(error = %error, "could not drop the gateway's bridge flags");
    }
    if let Some(live) = manager
        .network()
        .and_then(|network| network.query_surface_serving_handles())
    {
        live.serve_nothing();
    }
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
/// Runtime lifecycle state.
pub enum RuntimeState {
    /// No runtime has been started.
    Idle,
    /// Runtime startup is in progress.
    Starting,
    /// Runtime is accepting publishes.
    Running,
    /// Runtime shutdown is in progress.
    Stopping,
    /// Runtime has stopped.
    Stopped,
    /// A graph finished on its own after a node requested shutdown. The
    /// gateway, recorder, and app publishers keep their resources until
    /// `stop`, which finalizes the recording; `publish` is refused.
    Finished,
    /// Startup or a runtime component failed.
    Failed,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
/// The last error exposed in runtime status.
pub struct LastError {
    /// Stable error code.
    pub code: i32,
    /// Human-readable error message.
    pub message: String,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
/// A graph node whose tick panicked. The graph runtime contains the panic:
/// the node does not run again, and the rest of the graph keeps running.
pub struct NodeFailure {
    /// The node's graph id.
    pub node_id: String,
    /// The node's first panic message.
    pub message: String,
}

/// The nodes of the current graph whose tick panicked, in graph order.
///
/// Read while the graph runs: [`Self::list`] and the status JSON read the
/// graph runtime's records at the time of the read. A clone keeps the list
/// as it was when cloned, so a cloned [`Status`] is a snapshot. The records
/// stay readable after `stop`; the next start whose config parses and
/// passes its path checks clears them, also when that start then fails.
#[derive(Default)]
pub struct NodeFailures {
    source: FailureSource,
}

enum FailureSource {
    /// The records the graph's tick callbacks write.
    Live(NodeTickPanicRecords),
    /// A fixed list: empty before a graph is built, or the list a clone read.
    Snapshot(Vec<NodeFailure>),
}

impl Default for FailureSource {
    fn default() -> Self {
        Self::Snapshot(Vec::new())
    }
}

impl NodeFailures {
    fn live(records: NodeTickPanicRecords) -> Self {
        Self {
            source: FailureSource::Live(records),
        }
    }

    /// The failed nodes, in graph order.
    pub fn list(&self) -> Vec<NodeFailure> {
        match &self.source {
            FailureSource::Live(records) => records
                .snapshot()
                .into_iter()
                .map(|(node_id, message)| NodeFailure { node_id, message })
                .collect(),
            FailureSource::Snapshot(list) => list.clone(),
        }
    }
}

impl Clone for NodeFailures {
    fn clone(&self) -> Self {
        Self {
            source: FailureSource::Snapshot(self.list()),
        }
    }
}

impl PartialEq for NodeFailures {
    fn eq(&self, other: &Self) -> bool {
        // Two reads of one live view could straddle a new panic.
        std::ptr::eq(self, other) || self.list() == other.list()
    }
}

impl Eq for NodeFailures {}

impl std::fmt::Debug for NodeFailures {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.debug_list().entries(self.list()).finish()
    }
}

impl Serialize for NodeFailures {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.list().serialize(serializer)
    }
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
/// The recorder totals written by a completed runtime.
pub struct RecorderSummary {
    /// Total user messages written.
    pub messages: u64,
    /// Number of topics with recorded messages.
    pub channels: usize,
    /// Total wire-frame bytes written.
    pub bytes: u64,
    /// Recorded message count by topic.
    pub per_topic: BTreeMap<String, u64>,
    /// Bag files produced by the recorder, in order.
    pub bag_paths: Vec<String>,
    /// First output path, when a bag was written.
    pub out_path: Option<String>,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
/// Deterministic runtime status snapshot.
pub struct Status {
    /// Lifecycle state.
    pub state: RuntimeState,
    /// Robot identity.
    pub robot: String,
    /// Parsed graph name, if configured.
    pub graph: Option<String>,
    /// Number of graph nodes.
    pub nodes: usize,
    /// Node types linked into the app library, sorted.
    pub linked_nodes: Vec<String>,
    /// Network listen endpoints.
    pub listen: Vec<String>,
    /// LAN access posture of the running gateway (`paired` or `open`).
    pub lan_access: Option<String>,
    /// Whether the `_cerulion._tcp` mDNS beacon is registered.
    pub mdns_advertised: bool,
    /// Configured recording path.
    pub record_path: Option<String>,
    /// Successful publish count by topic.
    pub published: BTreeMap<String, u64>,
    /// Recorder totals, available after stop.
    pub recorder: Option<RecorderSummary>,
    /// Most recent runtime error.
    pub last_error: Option<LastError>,
    /// Graph nodes whose tick panicked, in graph order. The state stays
    /// `running`: the graph keeps running without them.
    pub node_failures: NodeFailures,
}

impl Status {
    fn new(robot: String) -> Self {
        Self {
            state: RuntimeState::Idle,
            robot,
            graph: None,
            nodes: 0,
            linked_nodes: crate::nodes::linked_node_types(),
            listen: Vec::new(),
            lan_access: None,
            mdns_advertised: false,
            record_path: None,
            published: BTreeMap::new(),
            recorder: None,
            last_error: None,
            node_failures: NodeFailures::default(),
        }
    }
}

/// A hook a test runs between a start's build and its commit.
#[cfg(any(test, feature = "test-helpers"))]
type BeforeCommitHook = Box<dyn FnOnce(&TransportManager) + Send>;

/// Synchronous owner of embedded graph, gateway, recorder, and publisher resources.
/// Dropping it stops and joins whatever is still running.
pub struct EmbeddedRuntime {
    files_dir: PathBuf,
    robot_name: String,
    status: Arc<Mutex<Status>>,
    /// The committed run. A start builds its run off to the side and stores
    /// it here only once the commit succeeds.
    run: Option<Run>,
    #[cfg(any(test, feature = "test-helpers"))]
    before_commit: Option<BeforeCommitHook>,
}

/// Every component one start owns. A start fills it step by step; the
/// teardown in [`Run::shutdown`] (also run by `Drop`, so a panic mid-build
/// tears down too) stops whatever was filled in a safe order.
struct Run {
    graph: Option<GraphWorker>,
    gateway: Option<GatewayWorker>,
    recorder: Option<RecorderWorker>,
    publishers: BTreeMap<String, PublisherState>,
    /// Only `ownership` must drop after the workers: `shutdown` takes it
    /// last, and its own fields release the lease after the reset.
    ownership: Option<GatewayOwnership>,
    manager: Arc<TransportManager>,
    /// Orders a worker's failure against the commit. A worker records
    /// Failed while holding it, and the commit holds it from its failure
    /// check until the run is Running, so a failure lands wholly before
    /// the commit opens LAN access or wholly after the run started. It is
    /// separate from the status lock, so status reads never wait out the
    /// commit's network steps.
    commit_gate: Arc<Mutex<()>>,
}

struct GraphWorker {
    running: Arc<AtomicBool>,
    ready: Arc<AtomicBool>,
    thread: JoinHandle<Result<(), String>>,
}

struct GatewayWorker {
    exit: Arc<AtomicBool>,
    ready: Arc<AtomicBool>,
    thread: JoinHandle<Result<(), String>>,
    beacon: Arc<Mutex<BeaconSlot<MdnsAdvertiseGuard>>>,
}

struct RecorderWorker {
    shutdown: Arc<AtomicBool>,
    thread: JoinHandle<Result<BagdSummary, String>>,
}

/// The mDNS beacon of a gateway thread. The commit raises the beacon and
/// stores it here unless the thread already exited (`closed`); the thread's
/// exit closes the slot and withdraws whatever beacon it holds.
struct BeaconSlot<G> {
    closed: bool,
    guard: Option<G>,
}

/// Held by the gateway thread: on a normal return or an unwind it closes the
/// beacon slot, withdraws the beacon and clears `mdns_advertised`.
struct BeaconRelease<G> {
    slot: Arc<Mutex<BeaconSlot<G>>>,
    status: Arc<Mutex<Status>>,
}

impl<G> Drop for BeaconRelease<G> {
    fn drop(&mut self) {
        // Lock order: slot, then status (the commit's beacon step uses the
        // same order).
        let mut slot = lock_slot(&self.slot);
        slot.closed = true;
        drop(slot.guard.take());
        lock_status(&self.status).mdns_advertised = false;
    }
}

/// Store a beacon the commit raised, unless the gateway thread already
/// closed the slot. Returns the beacon when it was not stored, for the
/// caller to drop outside the lock.
fn install_beacon<G>(
    slot: &Mutex<BeaconSlot<G>>,
    status: &Arc<Mutex<Status>>,
    beacon: Option<G>,
) -> Option<G> {
    let mut slot = lock_slot(slot);
    if slot.closed {
        return beacon;
    }
    let advertised = beacon.is_some();
    slot.guard = beacon;
    lock_status(status).mdns_advertised = advertised;
    None
}

fn lock_slot<G>(slot: &Mutex<BeaconSlot<G>>) -> MutexGuard<'_, BeaconSlot<G>> {
    slot.lock().unwrap_or_else(|error| error.into_inner())
}

/// The recording file a start created. Dropping the reservation removes the
/// file while it is still armed, so a start that fails before bagd owns the
/// file leaves the name free; `disarm` hands the file to bagd.
struct RecordingReservation {
    path: PathBuf,
    armed: bool,
}

impl RecordingReservation {
    fn reserve(path: &Path, file: &str) -> Result<Self, EmbedError> {
        reserve_recording_path(path, file)?;
        Ok(Self {
            path: path.to_path_buf(),
            armed: true,
        })
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for RecordingReservation {
    fn drop(&mut self) {
        if self.armed {
            // Best-effort cleanup of an already-failing start.
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

struct PublisherState {
    publisher: CerulionPublisher,
    layout: WireLayout,
    max_payload_bytes: usize,
    sequence: u32,
    /// Reused encode buffer: cleared per publish, its capacity kept (the JSON
    /// facade is an encoding boundary, not the zero-copy typed path).
    frame: Vec<u8>,
    /// Reused length scratch for the same reason.
    lengths: Vec<usize>,
}

struct ResolvedPublish {
    topic: String,
    schema: String,
    layout: WireLayout,
    schema_hash: u64,
    wire_fixed_size: u32,
    max_payload_bytes: usize,
}

struct GraphInputs {
    config: GraphConfig,
    factories: indexmap::IndexMap<String, Box<dyn NodeEntry>>,
    infos: indexmap::IndexMap<String, cerulion_core::graph::NodeInfo>,
}

/// Everything a start needs, resolved before it touches any shared state.
struct StartPlan {
    graph: Option<GraphInputs>,
    graph_name: Option<String>,
    nodes: usize,
    schemas: SchemaSet,
    publishes: Vec<ResolvedPublish>,
    recorded_topics: Option<HashSet<String>>,
    recorded_graph_topics: Option<HashSet<String>>,
    listen: Vec<String>,
    gateway: Option<GatewayStartPlan>,
    recorder: Option<RecorderPlan>,
}

struct GatewayStartPlan {
    plan: GatewayPlan,
    /// The schema serving, with `served_topics` naming every served topic
    /// (graph outputs and publish topics, schema-less ones included).
    serving: SchemaServing,
    lan_access: LanAccess,
    mdns: bool,
}

struct RecorderPlan {
    config: BagdConfig,
    file: String,
    record_path: PathBuf,
    ready_file: PathBuf,
}

/// A failed build: the error, and the recorder totals its teardown wrote.
struct StartFailure {
    error: EmbedError,
    summary: Option<RecorderSummary>,
}

impl EmbeddedRuntime {
    /// Create an idle runtime for an existing absolute app files directory.
    pub fn new(files_dir: PathBuf, robot_name: String) -> Result<Self, EmbedError> {
        validate_robot_name(&robot_name)?;
        if !files_dir.is_absolute() || !files_dir.is_dir() {
            return Err(EmbedError::InvalidArgument(
                "files_dir must be an existing absolute directory".to_string(),
            ));
        }
        Ok(Self {
            files_dir,
            status: Arc::new(Mutex::new(Status::new(robot_name.clone()))),
            robot_name,
            run: None,
            #[cfg(any(test, feature = "test-helpers"))]
            before_commit: None,
        })
    }

    /// Start configured components and return after they are ready.
    pub fn start(&mut self, config_json: &str) -> Result<(), EmbedError> {
        if matches!(
            lock_status(&self.status).state,
            RuntimeState::Starting | RuntimeState::Running | RuntimeState::Stopping
        ) {
            return Err(EmbedError::BadState(
                "runtime is already starting or running".to_string(),
            ));
        }
        self.start_accepted(config_json)
    }

    /// TEST SEAM: run `hook` once, on the next start, after every component
    /// is built and ready and before the start commits (opens LAN access,
    /// replays announces, raises the beacon and reports Running).
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn set_before_commit_hook_for_test(
        &mut self,
        hook: impl FnOnce(&TransportManager) + Send + 'static,
    ) {
        self.before_commit = Some(Box::new(hook));
    }

    pub(crate) fn start_accepted(&mut self, config_json: &str) -> Result<(), EmbedError> {
        // A component of a failed run or a finished graph may have left
        // siblings running; tear them down BEFORE the new config is
        // validated, so a rejected retry cannot leave the old gateway or
        // recorder active behind the refusal.
        if let Some(mut run) = self.run.take() {
            let (_summary, teardown_error) = run.shutdown();
            if let Some(error) = teardown_error {
                tracing::warn!(error = %error, "stopping components left by a previous run before restart");
            }
        }
        let config = match EmbeddedConfig::parse(config_json) {
            Ok(config) => config,
            Err(error) => {
                set_error(&self.status, &error);
                return Err(error);
            }
        };
        if let Err(error) = config.validate_paths(&self.files_dir) {
            set_error(&self.status, &error);
            return Err(error);
        }
        // Every start whose config parses and passes its path checks sets
        // the level, also when a later step fails, so a restart without the
        // key returns to `info`.
        crate::log::set_level(config.log_level);
        {
            let mut status = lock_status(&self.status);
            status.state = RuntimeState::Starting;
            status.graph = None;
            status.nodes = 0;
            // `nodes::register` may have run after this runtime was made.
            status.linked_nodes = crate::nodes::linked_node_types();
            status.listen = config.listen.clone();
            status.lan_access = None;
            status.mdns_advertised = false;
            status.record_path = None;
            status.published.clear();
            status.recorder = None;
            status.last_error = None;
            status.node_failures = NodeFailures::default();
        }
        let planned = install_iceoryx_config(&self.files_dir)
            .and_then(|()| self.plan_start(config))
            .and_then(|plan| Ok((self.transport_manager(&plan.listen)?, plan)));
        let (manager, mut plan) = match planned {
            Ok(planned) => planned,
            Err(error) => {
                set_error(&self.status, &error);
                return Err(error);
            }
        };
        let mut run = match build(&mut plan, manager, &self.status) {
            Ok(run) => run,
            Err(failure) => {
                let StartFailure { error, summary } = *failure;
                // The recorder thread is joined, so a later `stop()` cannot
                // recover its totals: keep them in status.
                lock_status(&self.status).recorder = summary;
                set_error(&self.status, &error);
                return Err(error);
            }
        };
        #[cfg(any(test, feature = "test-helpers"))]
        if let Some(hook) = self.before_commit.take() {
            hook(&run.manager);
        }
        if let Err(error) = commit(&mut run, &plan, &self.status, &self.robot_name) {
            let (summary, teardown_error) = run.shutdown();
            if let Some(teardown_error) = teardown_error {
                tracing::warn!(error = %teardown_error, "tearing down a start whose commit failed");
            }
            lock_status(&self.status).recorder = summary;
            set_error(&self.status, &error);
            return Err(error);
        }
        self.run = Some(run);
        Ok(())
    }

    /// The publish-JSON length limit for `topic`, the bound `encode_json`
    /// enforces after decoding. The JNI layer checks a Java string's UTF-16
    /// length against this before converting, since every UTF-16 unit costs
    /// at least one UTF-8 byte.
    pub fn publish_json_limit(&self, topic: &str) -> Result<usize, EmbedError> {
        if !matches!(lock_status(&self.status).state, RuntimeState::Running) {
            return Err(EmbedError::BadState(
                "publish requires a running runtime".to_string(),
            ));
        }
        let state = self
            .run
            .as_ref()
            .and_then(|run| run.publishers.get(topic))
            .ok_or_else(|| EmbedError::UnknownTopic(topic.to_string()))?;
        Ok(crate::encode::max_publish_json_len(state.max_payload_bytes))
    }

    /// Test seam: the publish-decision test needs a running status without a
    /// live transport.
    #[cfg(test)]
    pub(crate) fn mark_running_for_test(&mut self) {
        set_state(&self.status, RuntimeState::Running);
    }

    /// Publish a JSON object and return its wire sequence number.
    pub fn publish(&mut self, topic: &str, json: &str) -> Result<u32, EmbedError> {
        if !matches!(lock_status(&self.status).state, RuntimeState::Running) {
            return Err(EmbedError::BadState(
                "publish requires a running runtime".to_string(),
            ));
        }
        let state = self
            .run
            .as_mut()
            .and_then(|run| run.publishers.get_mut(topic))
            .ok_or_else(|| EmbedError::UnknownTopic(topic.to_string()))?;
        let timestamp = cerulion_core::clock::real_ns();
        let sequence = state.sequence;
        encode_json_into(
            &state.layout,
            json,
            state.max_payload_bytes,
            timestamp,
            sequence,
            &mut state.frame,
            &mut state.lengths,
        )?;
        let frame = &state.frame;
        let mut loan = state
            .publisher
            .loan_raw_uninit(frame.len())
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
        for (destination, source) in loan.bytes_uninit_mut().iter_mut().zip(frame.iter()) {
            destination.write(*source);
        }
        // SAFETY: the loop copies every byte of the complete frame into the loan.
        let loan = unsafe { loan.assume_init() };
        state
            .publisher
            .send_raw_loan(loan)
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
        state.publisher.check_subscriber_events();
        state.sequence = state.sequence.wrapping_add(1);
        if let Err(error) = state.publisher.notify_sent_sample() {
            tracing::debug!(topic, error = %error, "notify failed after embedded publish");
        }
        let mut status = lock_status(&self.status);
        *status.published.entry(topic.to_string()).or_default() += 1;
        Ok(sequence)
    }

    /// Return the shared runtime status handle.
    ///
    /// Do not hold a guard from this handle across `stop()` or while dropping this runtime.
    /// Both operations lock the same status mutex and join threads that update it.
    pub fn status_handle(&self) -> Arc<Mutex<Status>> {
        Arc::clone(&self.status)
    }

    /// Stop all started components and join their threads.
    pub fn stop(&mut self) -> Result<Option<RecorderSummary>, EmbedError> {
        if !matches!(
            lock_status(&self.status).state,
            RuntimeState::Running | RuntimeState::Failed | RuntimeState::Finished
        ) {
            return Err(EmbedError::BadState(
                "stop requires a started runtime".to_string(),
            ));
        }
        set_state(&self.status, RuntimeState::Stopping);
        let Some(mut run) = self.run.take() else {
            // A failed start already tore its run down and kept its recorder
            // totals in status; leave them visible.
            set_state(&self.status, RuntimeState::Stopped);
            return Ok(None);
        };
        let (summary, error) = run.shutdown();
        // A teardown error keeps Failed: the stopped state must mean every
        // resource actually released, not that the teardown was attempted.
        match &error {
            Some(error) => set_error(&self.status, error),
            None => set_state(&self.status, RuntimeState::Stopped),
        }
        // The summary is observable state (Principle #3): it is recorded even
        // when teardown itself reports an error.
        lock_status(&self.status).recorder = summary.clone();
        if let Some(error) = error {
            return Err(error);
        }
        Ok(summary)
    }

    /// Resolve the config into a [`StartPlan`]. Touches no shared state:
    /// the only thing it creates is the app-local `bags/` directory.
    fn plan_start(&self, config: EmbeddedConfig) -> Result<StartPlan, EmbedError> {
        let graph = config
            .graph_yaml
            .as_deref()
            .map(parse_graph)
            .transpose()
            .map_err(|error| EmbedError::Graph(error.to_string()))?;
        let schema_dir = self.files_dir.join("schemas");
        let builtins = parse_builtin_schemas();
        // `from_workspace_with_builtins` looks for `schemas/` under the given
        // workspace root itself, so it takes `files_dir`, not `schema_dir`.
        let (schemas, _) = if schema_dir.is_dir() {
            SchemaSet::from_workspace_with_builtins(&self.files_dir, builtins)
        } else {
            SchemaSet::from_schemas(builtins)
        }
        .map_err(|error| EmbedError::InvalidArgument(error.to_string()))?;

        let graph_inputs = graph
            .map(|graph| self.load_graph(graph, &config))
            .transpose()?;
        let publishes = config
            .publish
            .iter()
            .map(|publish| {
                let layout = schemas.layout(&publish.schema).ok_or_else(|| {
                    EmbedError::InvalidArgument(format!(
                        "unknown publish schema '{}'",
                        publish.schema
                    ))
                })?;
                let wire_fixed_size = u32::try_from(layout.fixed_size).map_err(|_| {
                    EmbedError::InvalidArgument(format!(
                        "schema '{}' fixed wire size exceeds u32",
                        publish.schema
                    ))
                })?;
                Ok(ResolvedPublish {
                    topic: publish.topic.clone(),
                    schema: publish.schema.clone(),
                    layout: layout.clone(),
                    schema_hash: layout.schema_hash,
                    wire_fixed_size,
                    max_payload_bytes: publish.max_payload_bytes,
                })
            })
            .collect::<Result<Vec<_>, EmbedError>>()?;

        let graph_topics = graph_inputs
            .as_ref()
            .map(|inputs| output_topics(&inputs.config))
            .unwrap_or_default();
        let graph_topic_set: HashSet<&str> = graph_topics.iter().map(String::as_str).collect();
        if let Some(topic) = publishes
            .iter()
            .map(|publish| publish.topic.as_str())
            .find(|topic| graph_topic_set.contains(topic))
        {
            return Err(EmbedError::InvalidArgument(format!(
                "publish topic '{topic}' is a graph output; graph topics are single-writer"
            )));
        }
        let publish_topics: HashSet<&str> =
            publishes.iter().map(|item| item.topic.as_str()).collect();
        if let Some(record) = &config.record {
            let known: HashSet<&str> = graph_topics
                .iter()
                .map(String::as_str)
                .chain(publish_topics.iter().copied())
                .collect();
            if let Some(topics) = &record.topics {
                if let Some(topic) = topics.iter().find(|topic| !known.contains(topic.as_str())) {
                    return Err(EmbedError::InvalidArgument(format!(
                        "record topic '{topic}' is neither a graph output nor a publish topic"
                    )));
                }
            }
        }

        // The recorded-topic set is resolved BEFORE the graph
        // build so the runtime can provision the tap raise at service
        // creation (graph outputs via the build ctor, app publishers via
        // `recorded_publisher_config`). `record.topics` narrows the union;
        // no `record` means no raise.
        let recorded_topics: Option<HashSet<String>> = config.record.as_ref().map(|record| {
            let selected: Option<HashSet<&str>> = record
                .topics
                .as_ref()
                .map(|topics| topics.iter().map(String::as_str).collect());
            graph_topics
                .iter()
                .cloned()
                .chain(publishes.iter().map(|item| item.topic.clone()))
                .filter(|topic| {
                    selected
                        .as_ref()
                        .is_none_or(|set| set.contains(topic.as_str()))
                })
                .collect()
        });
        let recorded_graph_topics: Option<HashSet<String>> = recorded_topics.as_ref().map(|set| {
            set.iter()
                .filter(|topic| graph_topic_set.contains(topic.as_str()))
                .cloned()
                .collect()
        });

        // Only a start with listen endpoints builds the gateway and changes
        // manager-wide LAN state; a listen-less start leaves both alone.
        let gateway = (!config.listen.is_empty()).then(|| {
            let extra = publishes
                .iter()
                .map(|publish| TopicSchema {
                    topic: publish.topic.clone(),
                    schema_name: publish.schema.clone(),
                })
                .collect::<Vec<_>>();
            let mut serving = cerulion_cli_engine::schema_serve::build_app_schema_serving(
                graph_inputs.as_ref().map(|inputs| &inputs.config),
                &schema_dir,
                &extra,
            );
            // The served set names EVERY served topic, including schema-less
            // outputs, which carry no topic_schemas binding.
            let mut announce = graph_topics.clone();
            for publish in &publishes {
                if !announce.contains(&publish.topic) {
                    announce.push(publish.topic.clone());
                }
            }
            serving.served_topics = announce.clone();
            GatewayStartPlan {
                plan: GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce,
                    ingress: Vec::new(),
                },
                serving,
                lan_access: LanAccess::from(config.lan_access),
                mdns: config.mdns,
            }
        });
        let recorder =
            Self::plan_recorder(&self.files_dir, &config, graph_inputs.as_ref(), &publishes)?;
        Ok(StartPlan {
            graph_name: graph_inputs
                .as_ref()
                .map(|inputs| inputs.config.identity().to_string()),
            nodes: graph_inputs
                .as_ref()
                .map_or(0, |inputs| inputs.config.nodes.len()),
            graph: graph_inputs,
            schemas,
            publishes,
            recorded_topics,
            recorded_graph_topics,
            listen: config.listen,
            gateway,
            recorder,
        })
    }

    fn load_graph(
        &self,
        config: GraphConfig,
        embedded: &EmbeddedConfig,
    ) -> Result<GraphInputs, EmbedError> {
        let native_nodes = config.native_nodes().count();
        if native_nodes != config.nodes.len() {
            return Err(EmbedError::Graph(
                "embedded runtime supports only native graph nodes".to_string(),
            ));
        }
        // `validate_paths` already checked that a set directory exists.
        let node_lib_dir = embedded.node_lib_dir.as_deref();
        refuse_node_lib_dir_in_process(TRANSPORT_KIND, native_nodes, node_lib_dir.is_some())?;
        if native_nodes > 0 {
            crate::nodes::check()?;
        }
        let mut factories = indexmap::IndexMap::new();
        let mut infos = indexmap::IndexMap::new();
        for node in &config.nodes {
            // A linked node type wins; a separate library is the host-side
            // fallback for the cross-process transport only.
            let entry = match (crate::nodes::make(&node.node_type), node_lib_dir) {
                (Some(entry), _) => entry,
                (None, Some(dir)) => {
                    let path = dir.join(cdylib_name(&node.node_type));
                    Box::new(
                        DylibNodeEntry::load(&path)
                            .map_err(|error| EmbedError::Graph(error.to_string()))?,
                    ) as Box<dyn NodeEntry>
                }
                (None, None) => return Err(crate::nodes::unlinked(&node.node_type)),
            };
            let info = entry
                .info()
                .map_err(|error| EmbedError::Graph(error.to_string()))?;
            infos.insert(node.id.clone(), info);
            factories.insert(node.id.clone(), entry);
        }
        Ok(GraphInputs {
            config,
            factories,
            infos,
        })
    }

    fn transport_manager(&self, listen: &[String]) -> Result<Arc<TransportManager>, EmbedError> {
        let identity = PROCESS_TRANSPORT_IDENTITY.get_or_init(|| Mutex::new(None));
        let mut identity = identity.lock().unwrap_or_else(|error| error.into_inner());
        if let Some((existing_listen, existing_robot)) = identity.as_ref() {
            if existing_listen != listen {
                return Err(EmbedError::BadState(
                    "network listen endpoints are fixed for the process lifetime by the first start; restart the app process to change them".to_string(),
                ));
            }
            if existing_robot != &self.robot_name {
                return Err(EmbedError::BadState(
                    "the transport identity (robot name) is fixed for the process lifetime by the first start; restart the app process to change it".to_string(),
                ));
            }
            return TransportManager::get().map_err(|error| EmbedError::Network(error.to_string()));
        }
        // Another party may have initialized the process manager first
        // (before any runtime start); it carries no runtime identity yet.
        // Reusing it silently would drop this start's network config, so
        // only a listen-less start may share it.
        if let Ok(existing) = TransportManager::get() {
            if !listen.is_empty() {
                return Err(EmbedError::Network(
                    "the process transport manager is already initialized without this start's network listen endpoints; initialize it through the embedded runtime or restart the app process".to_string(),
                ));
            }
            *identity = Some((listen.to_vec(), self.robot_name.clone()));
            return Ok(existing);
        }
        let network = (!listen.is_empty()).then(|| NetworkConfig {
            listen_endpoints: listen.to_vec(),
            robot_identity: Some(self.robot_name.clone()),
            ..NetworkConfig::default()
        });
        let manager = TransportManager::init(TransportConfig {
            node_name: format!("{}_app", self.robot_name),
            clock: Arc::new(RealClock),
            network,
            ..TransportConfig::default()
        })
        .map_err(|error| EmbedError::Network(error.to_string()))?;
        // The manager takes its first posture from the environment (open
        // on a desktop host, or when `CERULION_LAN_ACCESS=open` is set).
        // Start it paired: LAN access opens only when a start commits.
        if let Some(network) = manager.network() {
            network.set_lan_access(LanAccess::Paired);
        }
        *identity = Some((listen.to_vec(), self.robot_name.clone()));
        Ok(manager)
    }

    fn plan_recorder(
        files_dir: &Path,
        embedded: &EmbeddedConfig,
        graph: Option<&GraphInputs>,
        publishes: &[ResolvedPublish],
    ) -> Result<Option<RecorderPlan>, EmbedError> {
        let Some(record) = &embedded.record else {
            return Ok(None);
        };
        let bags_dir = files_dir.join("bags");
        std::fs::create_dir_all(&bags_dir)
            .map_err(|error| EmbedError::Recorder(error.to_string()))?;
        let record_path = bags_dir.join(&record.file);
        let ready_file = bags_dir.join(format!("{}.ready", record.file));
        let mut recorded = Vec::new();
        if let Some(graph) = graph {
            #[cfg(unix)]
            {
                // Apply the record selection inside the resolution so a
                // names-only output that is NOT selected cannot fail startup.
                let selected: Option<std::collections::HashSet<String>> = record
                    .topics
                    .as_ref()
                    .map(|topics| topics.iter().cloned().collect());
                recorded = resolve_recorded_topics(
                    files_dir,
                    &graph.config,
                    &graph.infos,
                    selected.as_ref(),
                )
                .map_err(|error| EmbedError::Recorder(error.to_string()))?;
            }
            #[cfg(not(unix))]
            {
                let _ = graph;
                return Err(EmbedError::Recorder(
                    "graph recording is only available on Unix".to_string(),
                ));
            }
        }
        for publish in publishes {
            recorded.push(RecordedTopic {
                format: RECORDED_TOPIC_FORMAT,
                topic: publish.topic.clone(),
                schema_name: publish.schema.clone(),
                schema_hash: publish.schema_hash,
                wire_fixed_size: publish.wire_fixed_size,
                multi_publisher: false,
            });
        }
        if let Some(selected) = &record.topics {
            let selected: HashSet<&str> = selected.iter().map(String::as_str).collect();
            recorded.retain(|topic| selected.contains(topic.topic.as_str()));
        }
        if recorded.is_empty() {
            return Err(EmbedError::InvalidArgument(
                "recording requires at least one graph output or publish topic".to_string(),
            ));
        }
        let taps = recorded
            .into_iter()
            .map(RecordedTopic::into_tap_spec)
            .collect();
        let mut config = BagdConfig::new(&record_path, taps);
        // Startup waits for the recorder before the graph and app publishers
        // produce, so the same first-frame accounting `graph run --record`
        // gets applies here.
        config.armed_before_producers = true;
        config.ready_file = Some(ready_file.clone());
        // The same schema catalog the CLI record verbs
        // build: built-in hash bindings always, custom type text when the app
        // ships a `files/schemas/` store, so the bag names and decodes its
        // own channels.
        let schema_dir = files_dir.join("schemas");
        config.schema_catalog = cerulion_cli_engine::bag_cmd::build_record_schema_catalog(
            schema_dir.is_dir().then_some(schema_dir.as_path()),
            None,
        );
        Ok(Some(RecorderPlan {
            config,
            file: record.file.clone(),
            record_path,
            ready_file,
        }))
    }
}

impl Drop for EmbeddedRuntime {
    fn drop(&mut self) {
        let Some(mut run) = self.run.take() else {
            return;
        };
        set_state(&self.status, RuntimeState::Stopping);
        let (summary, error) = run.shutdown();
        match error {
            None => {
                set_state(&self.status, RuntimeState::Stopped);
                lock_status(&self.status).recorder = summary;
            }
            Some(error) => {
                set_error(&self.status, &error);
                tracing::warn!(error = %error, "embedded runtime: stop on drop failed");
            }
        }
    }
}

impl Run {
    fn empty(manager: Arc<TransportManager>) -> Self {
        Self {
            graph: None,
            gateway: None,
            recorder: None,
            publishers: BTreeMap::new(),
            ownership: None,
            manager,
            commit_gate: Arc::new(Mutex::new(())),
        }
    }

    /// Stop every component and join its thread. Returns the recorder
    /// summary (if any) and the first teardown error separately, so the
    /// caller can store the summary in status BEFORE propagating the error.
    /// Idempotent: each step empties what it stopped.
    fn shutdown(&mut self) -> (Option<RecorderSummary>, Option<EmbedError>) {
        // Close LAN access before any join: a join that blocks must not
        // leave an open gateway serving LAN peers. Only the gateway owner
        // may: a listen-less runtime sharing the manager leaves the owner's
        // posture alone.
        if self.ownership.is_some() {
            if let Some(network) = self.manager.network() {
                network.set_lan_access(LanAccess::Paired);
            }
        }
        let mut first_error = None;
        if let Some(graph) = self.graph.take() {
            graph.running.store(false, Ordering::Relaxed);
            match graph.thread.join() {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    first_error.get_or_insert(EmbedError::Graph(error));
                }
                Err(error) => {
                    first_error.get_or_insert_with(|| {
                        EmbedError::Graph(format!("graph thread panicked: {error:?}"))
                    });
                }
            }
        }
        if let Some(gateway) = self.gateway.take() {
            gateway.exit.store(true, Ordering::Relaxed);
            self.manager.bridge_manager().demand_signal().signal();
            // The beacon is withdrawn on the gateway thread as it exits.
            match gateway.thread.join() {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    first_error.get_or_insert(EmbedError::Network(error));
                }
                Err(error) => {
                    first_error.get_or_insert_with(|| {
                        EmbedError::Network(format!("gateway thread panicked: {error:?}"))
                    });
                }
            }
        }
        // The recorder joins BEFORE the ownership drops: once the lease is
        // free another gateway runtime may start publishing, and a live tap
        // of this run must not record its frames.
        let summary = self
            .recorder
            .take()
            .and_then(|recorder| recorder.stop(&mut first_error));
        self.publishers.clear();
        // Resets the manager to the quiescent baseline, then frees the lease.
        drop(self.ownership.take());
        (summary, first_error)
    }
}

impl Drop for Run {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

impl RecorderWorker {
    fn stop(self, first_error: &mut Option<EmbedError>) -> Option<RecorderSummary> {
        self.shutdown.store(true, Ordering::Relaxed);
        match self.thread.join() {
            Ok(Ok(summary)) => {
                let bag_paths = summary
                    .bag_paths
                    .into_iter()
                    .map(|path| path.to_string_lossy().into_owned())
                    .collect::<Vec<_>>();
                Some(RecorderSummary {
                    messages: summary.messages,
                    channels: summary.per_topic.len(),
                    bytes: summary.bytes,
                    per_topic: summary.per_topic,
                    out_path: bag_paths.first().cloned(),
                    bag_paths,
                })
            }
            Ok(Err(error)) => {
                first_error.get_or_insert_with(|| EmbedError::Recorder(error));
                None
            }
            Err(error) => {
                first_error.get_or_insert_with(|| {
                    EmbedError::Recorder(format!("bagd thread panicked: {error:?}"))
                });
                None
            }
        }
    }
}

/// Build every component of `plan` into one owned [`Run`] while LAN access
/// stays paired. On an error the partial run is shut down, which returns
/// the manager to the quiescent baseline; a panic does the same through
/// `Drop for Run`.
fn build(
    plan: &mut StartPlan,
    manager: Arc<TransportManager>,
    status: &Arc<Mutex<Status>>,
) -> Result<Run, Box<StartFailure>> {
    let mut run = Run::empty(manager);
    match build_into(&mut run, plan, status) {
        Ok(()) => Ok(run),
        Err(error) => {
            let (summary, teardown_error) = run.shutdown();
            if let Some(teardown_error) = teardown_error {
                tracing::warn!(error = %teardown_error, "tearing down a start whose build failed");
            }
            Err(Box::new(StartFailure { error, summary }))
        }
    }
}

fn build_into(
    run: &mut Run,
    plan: &mut StartPlan,
    status: &Arc<Mutex<Status>>,
) -> Result<(), EmbedError> {
    let manager = Arc::clone(&run.manager);
    // 1. Own the gateway: the manager is now at the baseline and paired.
    //    A latched query surface (an earlier start created it) gets this
    //    run's serving before the constructor records any runtime hash;
    //    a first start seeds the surface through the constructor.
    if let Some(gateway) = &plan.gateway {
        run.ownership = Some(GatewayOwnership::acquire(&manager)?);
        if let Some(live) = manager
            .network()
            .and_then(|network| network.query_surface_serving_handles())
        {
            live.replace(&gateway.serving);
        }
    }

    // 2. App publishers.
    create_publishers(
        &plan.publishes,
        &manager,
        plan.recorded_topics.as_ref(),
        &mut run.publishers,
    )?;

    // 3. The graph, built but not yet running.
    let graph_runtime = if let Some(inputs) = plan.graph.as_mut() {
        let clock: Arc<dyn Clock> = Arc::new(RealClock);
        let schema_hashes = schema_hash_map(&plan.schemas);
        let mut graph_runtime = GraphRuntime::build_live_with_schema_hashes_policy_and_recorded(
            inputs.config.clone(),
            std::mem::take(&mut inputs.factories),
            &manager,
            Arc::clone(&clock),
            Some(&schema_hashes),
            cerulion_cli_engine::graph_cmd::embedded_live_monitor_wait_policy(),
            plan.recorded_graph_topics.as_ref(),
        )
        .map_err(|error| EmbedError::Graph(error.to_string()))?;
        graph_runtime.set_live_transport(Arc::clone(&manager));
        // The graph thread takes the runtime: status keeps this reader.
        lock_status(status).node_failures =
            NodeFailures::live(graph_runtime.node_tick_panic_records());
        Some(graph_runtime)
    } else {
        None
    };

    // 4. The recorder runs before any producer, as `armed_before_producers`
    //    requires.
    if let Some(recorder) = &plan.recorder {
        let mut reservation = RecordingReservation::reserve(&recorder.record_path, &recorder.file)?;
        clear_ready_marker(&recorder.ready_file)?;
        let shutdown = Arc::new(AtomicBool::new(false));
        let thread_shutdown = Arc::clone(&shutdown);
        let recorder_manager = Arc::clone(&manager);
        let bagd_config = recorder.config.clone();
        let thread_status = Arc::clone(status);
        let thread_gate = Arc::clone(&run.commit_gate);
        let thread = thread::Builder::new()
            .name("cerulion-bagd".to_string())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_bagd(recorder_manager, bagd_config, thread_shutdown)
                        .map_err(|error| error.to_string())
                }))
                .unwrap_or_else(|_| Err("recorder worker panicked".to_string()));
                if let Err(error) = &result {
                    record_worker_failure(&thread_gate, &thread_status, codes::RECORDER, error);
                }
                result
            })
            .map_err(|error| EmbedError::Recorder(error.to_string()))?;
        run.recorder = Some(RecorderWorker { shutdown, thread });
        // bagd owns the file now: a failure from here on keeps the bag.
        reservation.disarm();
        wait_recorder_ready(&recorder.ready_file, &mut run.recorder)?;
    }

    // 5. The gateway. Under the paired posture its constructor registers
    //    the bridge flags, installs egress, starts the watch, latches the
    //    query surface and declares the identity token, and withholds every
    //    topic announce.
    if let Some(gateway) = &plan.gateway {
        let runtime = GatewayRuntime::new_with_schema_serving(
            Arc::clone(&manager),
            gateway.plan.clone(),
            gateway.serving.clone(),
        )
        .map_err(|error| EmbedError::Network(error.to_string()))?;
        let exit = Arc::new(AtomicBool::new(false));
        let thread_exit = Arc::clone(&exit);
        let ready = Arc::new(AtomicBool::new(false));
        let thread_ready = Arc::clone(&ready);
        let beacon = Arc::new(Mutex::new(BeaconSlot {
            closed: false,
            guard: None,
        }));
        let release = BeaconRelease {
            slot: Arc::clone(&beacon),
            status: Arc::clone(status),
        };
        let thread_status = Arc::clone(status);
        let thread_gate = Arc::clone(&run.commit_gate);
        let thread = thread::Builder::new()
            .name("cerulion-gateway".to_string())
            .spawn(move || {
                // The gateway's fallible setup ran in its synchronous
                // constructor above; `run` is the drive loop, so entering it
                // is ready.
                thread_ready.store(true, Ordering::Release);
                let mut runtime = runtime;
                // A panic must not leave the status reporting Running:
                // record it as a worker failure the same way a returned
                // error is recorded.
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    runtime.run(&thread_exit).map_err(|error| error.to_string())
                }))
                .unwrap_or_else(|_| Err("gateway worker panicked".to_string()));
                drop(release);
                if let Err(error) = &result {
                    record_worker_failure(&thread_gate, &thread_status, codes::NETWORK, error);
                }
                result
            })
            .map_err(|error| EmbedError::Network(error.to_string()))?;
        run.gateway = Some(GatewayWorker {
            exit,
            ready,
            thread,
            beacon,
        });
    }

    // 6. The graph thread.
    if let Some(mut graph_runtime) = graph_runtime {
        let running = Arc::new(AtomicBool::new(true));
        let thread_running = Arc::clone(&running);
        let ready = Arc::new(AtomicBool::new(false));
        let thread_ready = Arc::clone(&ready);
        let thread_status = Arc::clone(status);
        let thread_gate = Arc::clone(&run.commit_gate);
        let thread = thread::Builder::new()
            .name("cerulion-graph".to_string())
            .spawn(move || {
                // A panic unwinds past `run_live`'s own exit bookkeeping, so
                // surface it as a worker error: `record_graph_exit` then
                // marks the runtime Failed instead of leaving it Running.
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    // The launch checks (external sources, the inert-node
                    // refusal) run on this thread BEFORE it reports ready,
                    // so a refusal fails the start instead of landing after
                    // start returned Ok.
                    graph_runtime.prepare_live()?;
                    thread_ready.store(true, Ordering::Release);
                    graph_runtime.run_live(&thread_running)
                }))
                .map(|result| result.map_err(|error| error.to_string()))
                .unwrap_or_else(|_| Err("graph worker panicked".to_string()));
                {
                    let _commit = lock_gate(&thread_gate);
                    record_graph_exit(&thread_status, &thread_running, &result);
                }
                result
            })
            .map_err(|error| EmbedError::Graph(error.to_string()))?;
        run.graph = Some(GraphWorker {
            running,
            ready,
            thread,
        });
    }

    // 7. The startup barrier runs WITHOUT the status lock held: a worker's
    //    error path takes that same lock, and waiting here while holding it
    //    would deadlock the failure we are waiting to see. Every spawned
    //    worker must be ready (past its startup checks) or have exited. The
    //    recorder needs no flag: `wait_recorder_ready` already waited for
    //    its ready marker.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let gateway_settled = run
            .gateway
            .as_ref()
            .is_none_or(|worker| worker_settled(&worker.ready, &worker.thread));
        let graph_settled = run
            .graph
            .as_ref()
            .is_none_or(|worker| worker_settled(&worker.ready, &worker.thread));
        if gateway_settled && graph_settled {
            break;
        }
        if Instant::now() >= deadline {
            set_thread_error(
                status,
                codes::SPAWN,
                "a worker did not finish its startup checks within the startup barrier",
            );
            return Err(recorded_failure(&lock_status(status)));
        }
        thread::sleep(Duration::from_millis(2));
    }
    // A worker that failed during startup already recorded Failed.
    let status = lock_status(status);
    if matches!(status.state, RuntimeState::Failed) {
        return Err(recorded_failure(&status));
    }
    Ok(())
}

/// Make a built run visible: open LAN access to the configured posture,
/// replay the announces the paired build withheld, report the run and raise
/// the beacon. The only step of a start that does so. On an error the
/// caller shuts the run down, which closes LAN access first.
///
/// The run's commit gate is held from the last worker-failure check through
/// the posture write, the announce replay and the switch to Running. A
/// worker records its failure under that same gate, so a failure either
/// lands first (the commit refuses and LAN access never opens) or lands
/// after the run is Running (a failure of a started run). A failed start
/// never authorizes a LAN demand, even briefly. The status lock is taken
/// only briefly, never across the network steps, so a status read during
/// the commit returns at once.
fn commit(
    run: &mut Run,
    plan: &StartPlan,
    status: &Arc<Mutex<Status>>,
    robot: &str,
) -> Result<(), EmbedError> {
    let gate = Arc::clone(&run.commit_gate);
    {
        let _commit = lock_gate(&gate);
        {
            let status = lock_status(status);
            if matches!(status.state, RuntimeState::Failed) {
                return Err(recorded_failure(&status));
            }
        }
        if let Some(gateway) = &plan.gateway {
            let network = run.manager.network().ok_or_else(|| {
                EmbedError::Network("the gateway's transport manager has no network".to_string())
            })?;
            network.set_lan_access(gateway.lan_access);
            if gateway.lan_access == LanAccess::Open {
                // Read AFTER the posture opened: a topic registered before
                // this read is replayed here, and one registered after it
                // announces itself. A second announce of a topic is a no-op.
                let registered = run
                    .manager
                    .bridge_manager()
                    .registered_topics()
                    .map_err(|error| EmbedError::Network(error.to_string()))?;
                for (topic, _flag) in registered {
                    run.manager
                        .announce_egress_topic(&topic)
                        .map_err(|error| EmbedError::Network(error.to_string()))?;
                }
            }
        }
        let mut status = lock_status(status);
        // A graph that already finished on its own keeps Finished.
        if !matches!(status.state, RuntimeState::Finished) {
            status.state = RuntimeState::Running;
            status.last_error = None;
        }
        status.graph = plan.graph_name.clone();
        status.nodes = plan.nodes;
        status.listen = plan.listen.clone();
        status.lan_access = plan
            .gateway
            .as_ref()
            .map(|gateway| gateway.lan_access.as_str().to_string());
        status.record_path = plan
            .recorder
            .as_ref()
            .map(|recorder| recorder.record_path.to_string_lossy().into_owned());
    }
    // mDNS registration does I/O: raise it with no lock held. The beacon
    // names only the device, never a topic, and a gateway that stops
    // closes its slot, so a beacon raised after it stopped is dropped.
    if let Some(gateway) = &plan.gateway {
        if gateway.mdns {
            let beacon = advertise_beacon(robot, &plan.listen);
            if let Some(worker) = &run.gateway {
                drop(install_beacon(&worker.beacon, status, beacon));
            }
        }
    }
    Ok(())
}

fn create_publishers(
    config: &[ResolvedPublish],
    manager: &TransportManager,
    recorded_topics: Option<&HashSet<String>>,
    publishers: &mut BTreeMap<String, PublisherState>,
) -> Result<(), EmbedError> {
    for item in config {
        let slot_len = WireHeader::SIZE
            .checked_add(item.max_payload_bytes)
            .and_then(|length| u32::try_from(length).ok())
            .and_then(MaxSliceLen::try_new)
            .ok_or_else(|| {
                EmbedError::InvalidArgument(
                    "publisher slot size exceeds the wire transport limit".to_string(),
                )
            })?;
        // The app creates its topics before the graph builds, so each one is
        // created at the borrow floor a graph output gets: a graph node that
        // holds the topic as a latest-value input (an absolute `source:`)
        // needs it to open the topic. A RECORDED app topic also gets the tap
        // provisioning the graph build applies to recorded graph outputs.
        let mut topic_config = manager.default_topic_config();
        topic_config.create_borrow_floor = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        if recorded_topics.is_some_and(|set| set.contains(&item.topic)) {
            topic_config = recorded_publisher_config(topic_config, slot_len);
        }
        let publisher = manager
            .create_publisher_with_topic_config(&item.topic, slot_len, 0, topic_config)
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
        publishers.insert(
            item.topic.clone(),
            PublisherState {
                publisher,
                layout: item.layout.clone(),
                max_payload_bytes: item.max_payload_bytes,
                sequence: 0,
                frame: Vec::new(),
                lengths: Vec::new(),
            },
        );
    }
    Ok(())
}

/// Wait for bagd's ready marker. A recorder that exits first is joined here
/// and its error returned.
fn wait_recorder_ready(
    ready_file: &Path,
    recorder: &mut Option<RecorderWorker>,
) -> Result<(), EmbedError> {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if ready_file.exists() {
            return Ok(());
        }
        if recorder
            .as_ref()
            .is_some_and(|worker| worker.thread.is_finished())
        {
            let Some(worker) = recorder.take() else {
                return Err(EmbedError::Recorder(
                    "bagd exited before signaling readiness".to_string(),
                ));
            };
            return match worker.thread.join() {
                Ok(Err(error)) => Err(EmbedError::Recorder(error)),
                Ok(Ok(_)) => Err(EmbedError::Recorder(
                    "bagd exited before signaling readiness".to_string(),
                )),
                Err(error) => Err(EmbedError::Recorder(format!(
                    "bagd thread panicked before signaling readiness: {error:?}"
                ))),
            };
        }
        thread::sleep(Duration::from_millis(10));
    }
    Err(EmbedError::Recorder(
        "bagd did not signal readiness within 10 seconds".to_string(),
    ))
}

/// The topic service config an app publisher on a
/// RECORDED topic creates the service with: the graph build's recorded-topic
/// raise applied to `base` (the caller passes the default config with the
/// app topics' borrow floor set), so the bagd tap gets the same
/// borrow headroom (`RECORDING_SUBSCRIBER_MAX_BORROWED`) and receive-queue
/// depth (`recording_tap_buffer_depth`) a recorded graph output gets.
fn recorded_publisher_config(
    base: TopicServiceConfig,
    slot_len: MaxSliceLen,
) -> TopicServiceConfig {
    let mut config = base;
    config.subscriber_max_borrowed_samples = Some(
        config
            .subscriber_max_borrowed_samples
            .map_or(RECORDING_SUBSCRIBER_MAX_BORROWED, |cur| {
                cur.max(RECORDING_SUBSCRIBER_MAX_BORROWED)
            }),
    );
    config.subscriber_max_buffer_size = config
        .subscriber_max_buffer_size
        .max(recording_tap_buffer_depth(slot_len.get() as usize));
    config
}

/// The failure a component worker recorded in status
/// (`set_thread_error`), rebuilt as the `EmbedError` `start` returns when the
/// final state check finds it, keeping the same code and text the worker stored.
fn recorded_failure(status: &Status) -> EmbedError {
    let message = status.last_error.as_ref().map_or_else(
        || "a runtime component failed during startup".to_string(),
        |error| error.message.clone(),
    );
    match status.last_error.as_ref().map(|error| error.code) {
        Some(codes::GRAPH) => EmbedError::Graph(message),
        Some(codes::NETWORK) => EmbedError::Network(message),
        Some(codes::RECORDER) => EmbedError::Recorder(message),
        Some(codes::SPAWN) => EmbedError::Spawn(message),
        Some(code) => EmbedError::Panic(format!("component failure (code {code}): {message}")),
        None => EmbedError::Panic(message),
    }
}

/// Claim the recording path atomically so no second runtime can pass the
/// same `record.file` and have bagd truncate the first bag: `create_new`
/// refuses an existing path, and `BagWriter::create` later truncates only
/// the empty file reserved here.
fn reserve_recording_path(path: &Path, file: &str) -> Result<(), EmbedError> {
    match std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
    {
        Ok(_) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            Err(EmbedError::Recorder(format!(
                "recording file {file} already exists in files/bags; choose a new record.file"
            )))
        }
        Err(error) => Err(EmbedError::Recorder(format!(
            "cannot create recording file {file}: {error}"
        ))),
    }
}

fn clear_ready_marker(path: &Path) -> Result<(), EmbedError> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(EmbedError::Recorder(format!(
            "failed to clear recorder readiness marker: {error}"
        ))),
    }
}

fn install_iceoryx_config(files_dir: &Path) -> Result<(), EmbedError> {
    let config_path = files_dir.join("iceoryx2.toml");
    let mut iox_config = IoxConfig::default();
    // The stock root lands under /data/local/tmp, which an app process
    // cannot write; root the node/service state inside the app sandbox.
    let root = files_dir.join("iceoryx2");
    iox_config.global.set_root_path(
        &iceoryx2::prelude::Path::new(root.as_os_str().as_encoded_bytes())
            .map_err(|error| EmbedError::Network(format!("invalid iceoryx2 root path: {error}")))?,
    );
    let config_toml = toml::to_string(&iox_config)
        .map_err(|error| EmbedError::Network(format!("serialize iceoryx2 config: {error}")))?;
    fs::write(&config_path, config_toml)
        .map_err(|error| EmbedError::Network(format!("write iceoryx2 config: {error}")))?;
    let config_file = FilePath::new(config_path.as_os_str().as_encoded_bytes())
        .map_err(|error| EmbedError::Network(format!("invalid iceoryx2 config path: {error}")))?;
    IoxConfig::setup_global_config_from_file(&config_file)
        .map_err(|error| EmbedError::Network(format!("load iceoryx2 config: {error}")))?;
    Ok(())
}

/// The in-process transport an app runs on cannot deliver frames to or from
/// a node loaded from a separate library, so a graph with native nodes must
/// take them from the node registry there.
fn refuse_node_lib_dir_in_process(
    kind: TransportKind,
    native_nodes: usize,
    has_node_lib_dir: bool,
) -> Result<(), EmbedError> {
    if kind == TransportKind::InProcess && native_nodes > 0 && has_node_lib_dir {
        return Err(EmbedError::Graph(
            "node_lib_dir is not supported by the in-process app runtime; link the nodes into the app library with cerulion_android::embed_nodes!".to_string(),
        ));
    }
    Ok(())
}

fn output_topics(config: &GraphConfig) -> Vec<String> {
    let mut topics = Vec::new();
    for node in &config.nodes {
        for output in &node.outputs {
            let topic = resolve_output_topic(&config.prefix, &node.id, output);
            if !topics.contains(&topic) {
                topics.push(topic);
            }
        }
    }
    topics
}

fn schema_hash_map(schemas: &SchemaSet) -> HashMap<String, u64> {
    schemas
        .schemas()
        .iter()
        .filter_map(|schema| {
            let name = schema.qualified_name();
            schemas.schema_hash(&name).map(|hash| (name, hash))
        })
        .collect()
}

/// Register the mDNS beacon for the first TCP listen endpoint. A failure is logged
/// and leaves the gateway reachable by address, matching the daemon beacon.
fn advertise_beacon(robot: &str, listen: &[String]) -> Option<MdnsAdvertiseGuard> {
    let Some(port) = cerulion_mdns::srv_port_from_listen_endpoints(listen) else {
        tracing::info!(
            "mDNS beacon not raised: no LAN-dialable nonzero TCP listen port; port 0 and loopback hosts are not advertised"
        );
        return None;
    };
    match cerulion_mdns::advertise_gateway(robot, port) {
        Ok(guard) => {
            tracing::info!(port, "mDNS beacon registered");
            Some(guard)
        }
        Err(error) => {
            tracing::warn!(
                port,
                error = %error,
                "mDNS beacon not raised; desks reach this device by address only"
            );
            None
        }
    }
}

/// Whether a spawned worker is past the startup barrier: it reported ready
/// (its startup checks passed) or it already exited (so any failure is
/// recorded).
fn worker_settled(ready: &AtomicBool, thread: &JoinHandle<Result<(), String>>) -> bool {
    ready.load(Ordering::Acquire) || thread.is_finished()
}

fn lock_status(status: &Arc<Mutex<Status>>) -> MutexGuard<'_, Status> {
    status.lock().unwrap_or_else(|error| error.into_inner())
}

fn set_state(status: &Arc<Mutex<Status>>, state: RuntimeState) {
    lock_status(status).state = state;
}

fn set_error(status: &Arc<Mutex<Status>>, error: &EmbedError) {
    let mut status = lock_status(status);
    status.state = RuntimeState::Failed;
    status.last_error = Some(LastError {
        code: error.code(),
        message: error.to_string(),
    });
}

fn lock_gate(gate: &Mutex<()>) -> MutexGuard<'_, ()> {
    gate.lock().unwrap_or_else(|error| error.into_inner())
}

/// Record a worker's failure under the run's commit gate, so it is ordered
/// against the commit (see [`commit`]).
fn record_worker_failure(gate: &Mutex<()>, status: &Arc<Mutex<Status>>, code: i32, message: &str) {
    let _commit = lock_gate(gate);
    set_thread_error(status, code, message);
}

fn set_thread_error(status: &Arc<Mutex<Status>>, code: i32, message: &str) {
    let mut status = lock_status(status);
    status.state = RuntimeState::Failed;
    status.last_error = Some(LastError {
        code,
        message: message.to_string(),
    });
}

fn record_graph_exit(
    status: &Arc<Mutex<Status>>,
    running: &AtomicBool,
    result: &Result<(), String>,
) {
    if let Err(error) = result {
        set_thread_error(status, codes::GRAPH, error);
        return;
    }

    let mut status = lock_status(status);
    if running.load(Ordering::Relaxed)
        && matches!(status.state, RuntimeState::Starting | RuntimeState::Running)
    {
        status.state = RuntimeState::Finished;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A process-global transport manager another party initialized (no
    /// runtime identity latched) must not be silently reused by a start that
    /// asked for network listen endpoints - that start's listen config would
    /// be dropped. Assert the clear error, then that a listen-less start MAY
    /// share it (which latches the identity for the rest of the binary).
    #[test]
    fn an_existing_transport_manager_without_network_refuses_a_listen_start() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        let runtime =
            EmbeddedRuntime::new(temp.path().to_path_buf(), "tm_reuse_test".into()).unwrap();
        // Another party initialized the process manager with no network.
        let _ = TransportManager::init(TransportConfig {
            node_name: "tm_reuse_test".into(),
            network: None,
            ..TransportConfig::default()
        });
        // If some earlier test latched an identity this still exercises the
        // same contract: an existing manager the start cannot configure.
        let error = match runtime.transport_manager(&["tcp/127.0.0.1:7447".to_string()]) {
            Ok(_) => panic!("a listen start cannot reuse an unconfigured manager"),
            Err(error) => error,
        };
        assert!(
            matches!(error, EmbedError::Network(_) | EmbedError::BadState(_)),
            "expected a clear failure, got {error}"
        );
        // A listen-less start may share it (latches identity last).
        if TransportManager::get().is_ok() && PROCESS_TRANSPORT_IDENTITY.get().is_none() {
            runtime
                .transport_manager(&[])
                .expect("a listen-less start shares the existing manager");
        }
    }

    /// Serializes the tests that take the process-wide gateway lease.
    static GATEWAY_TEST_LOCK: Mutex<()> = Mutex::new(());

    fn gateway_test_lock() -> MutexGuard<'static, ()> {
        GATEWAY_TEST_LOCK
            .lock()
            .unwrap_or_else(|error| error.into_inner())
    }

    /// A private manager (not the process singleton) with a network plane
    /// and a robot identity, so announces and the query surface work.
    fn network_manager(name: &str) -> Arc<TransportManager> {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("{name}_{}", std::process::id()),
                network: Some(NetworkConfig {
                    robot_identity: Some(name.to_string()),
                    ..NetworkConfig::default()
                }),
                ..TransportConfig::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init test manager")
    }

    fn local_manager(name: &str) -> Arc<TransportManager> {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("{name}_{}", std::process::id()),
                network: None,
                ..TransportConfig::default()
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("init test manager")
    }

    /// The quiescent baseline: paired, nothing announced, no bridge flag,
    /// the latched serving empty and the gateway lease free.
    fn assert_baseline(manager: &TransportManager) {
        let network = manager.network().expect("network");
        assert_eq!(network.lan_access(), LanAccess::Paired);
        assert!(
            network.announced_topics().is_empty(),
            "announced: {:?}",
            network.announced_topics()
        );
        assert_eq!(
            manager
                .bridge_manager()
                .registered_topic_count()
                .expect("count"),
            0
        );
        if let Some(live) = network.query_surface_serving_handles() {
            assert!(live.is_serving_nothing_for_test());
        }
        assert!(!GATEWAY_OWNED.load(Ordering::Acquire));
    }

    fn bagd_summary(messages: u64) -> BagdSummary {
        BagdSummary {
            bag_paths: vec![std::path::PathBuf::from("first.mcap")],
            messages,
            bytes: 0,
            chunks: 0,
            per_topic: BTreeMap::new(),
            ring_records: 0,
            rings_unavailable: vec![],
            frozen_burst_markers: 0,
            dropped_unwritten: 0,
            headerless: 0,
            status_unavailable: false,
            frames_lost: 0,
            record_health: cerulion_bagd::RecordHealth {
                version: cerulion_bagd::RECORD_HEALTH_VERSION,
                dropped_unwritten: 0,
                drive_passes: 0,
                max_pass_duration_us: 0,
                drive_span_us: 0,
                drain_gaps: cerulion_bagd::DrainGapHistogram::default(),
                loss_counting_basis: None,
                topics: BTreeMap::new(),
            },
            record_coverage: cerulion_bagd::RecordCoverage::default(),
            channel_set_closed_after: None,
            drive_passes: 0,
            max_pass_duration: std::time::Duration::ZERO,
            drive_span: std::time::Duration::ZERO,
            drain_gaps: cerulion_bagd::DrainGapHistogram::default(),
            run_ended: None,
            run_vanished_captures_lost: 0,
            state_records: 0,
            state_coverage: None,
            shm_pinned_bytes: 0,
            unpriced_taps: 0,
            capture_close_max: std::time::Duration::ZERO,
            captures_closed: 0,
        }
    }

    /// A gateway plan with nothing to announce: the build takes the
    /// ownership (step 1) and the test fails a later step.
    fn recorder_only_gateway_plan(recorder: RecorderPlan) -> StartPlan {
        let (schemas, _) = SchemaSet::from_schemas(Vec::new()).expect("empty schema set");
        StartPlan {
            graph: None,
            graph_name: None,
            nodes: 0,
            schemas,
            publishes: Vec::new(),
            recorded_topics: None,
            recorded_graph_topics: None,
            listen: Vec::new(),
            gateway: Some(GatewayStartPlan {
                plan: GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: Vec::new(),
                    ingress: Vec::new(),
                },
                serving: SchemaServing::default(),
                lan_access: LanAccess::Open,
                mdns: false,
            }),
            recorder: Some(recorder),
        }
    }

    /// Put stray gateway state on `manager`: an open posture, an enabled
    /// bridge flag and an announce for `topic`.
    fn leave_stray_state(manager: &TransportManager, topic: &str) {
        let network = manager.network().expect("network");
        network.set_lan_access(LanAccess::Open);
        let flag = manager
            .bridge_manager()
            .register_topic(topic)
            .expect("register stray flag");
        flag.store(true, Ordering::Relaxed);
        manager.announce_egress_topic(topic).expect("announce");
        assert!(network.announced_topics().contains(&topic.to_string()));
    }

    /// The teardown releases the lease only after the recorder joined: a
    /// second gateway runtime that starts once the lease is free must never
    /// share the manager with a live tap of the stopped run. Oracle: the
    /// stub recorder sees the lease still held when it is told to stop.
    #[test]
    fn shutdown_releases_the_lease_only_after_the_recorder_joins() {
        let _serial = gateway_test_lock();
        let manager = local_manager("lease_order");
        let mut run = Run::empty(Arc::clone(&manager));
        run.ownership = Some(GatewayOwnership::acquire(&manager).expect("lease"));
        let shutdown = Arc::new(AtomicBool::new(false));
        let thread_shutdown = Arc::clone(&shutdown);
        run.recorder = Some(RecorderWorker {
            shutdown,
            thread: std::thread::spawn(move || {
                while !thread_shutdown.load(Ordering::Relaxed) {
                    std::thread::yield_now();
                }
                if GATEWAY_OWNED.load(Ordering::Acquire) {
                    Ok(bagd_summary(1))
                } else {
                    Err("the lease was released before the recorder joined".to_string())
                }
            }),
        });

        let (summary, error) = run.shutdown();

        assert!(error.is_none(), "{error:?}");
        assert_eq!(summary.expect("recorder summary").messages, 1);
        assert!(!GATEWAY_OWNED.load(Ordering::Acquire));
    }

    /// The ready marker cannot be cleared (it is a non-empty directory):
    /// the build fails after the recording file was reserved, so the file
    /// is removed and the manager is back at the baseline.
    #[test]
    fn a_failed_ready_marker_clear_releases_the_reserved_file_and_restores_baseline() {
        let _serial = gateway_test_lock();
        let manager = network_manager("readyclear");
        leave_stray_state(&manager, "/ready_clear/stray");
        let temp = tempfile::tempdir().expect("temporary files directory");
        let record_path = temp.path().join("blocked.mcap");
        let ready_file = temp.path().join("blocked.mcap.ready");
        std::fs::create_dir(&ready_file).expect("create marker directory");
        std::fs::write(ready_file.join("child"), b"content").expect("make directory non-empty");
        let mut plan = recorder_only_gateway_plan(RecorderPlan {
            config: BagdConfig::new(&record_path, Vec::new()),
            file: "blocked.mcap".to_string(),
            record_path: record_path.clone(),
            ready_file,
        });
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));

        let failure = match build(&mut plan, Arc::clone(&manager), &status) {
            Ok(_) => panic!("a ready marker that cannot be cleared fails the build"),
            Err(failure) => failure,
        };

        assert!(
            failure
                .error
                .to_string()
                .starts_with("failed to clear recorder readiness marker:"),
            "{}",
            failure.error
        );
        assert!(!record_path.exists(), "the reserved file must be released");
        assert_baseline(&manager);
    }

    /// bagd exits before it reports ready (it refuses a tap-less config):
    /// the file was handed to bagd at spawn, so it stays, and the manager
    /// is back at the baseline.
    #[test]
    fn a_recorder_that_exits_before_ready_keeps_its_bag_and_restores_baseline() {
        let _serial = gateway_test_lock();
        let manager = network_manager("earlyexit");
        leave_stray_state(&manager, "/early_exit/stray");
        let temp = tempfile::tempdir().expect("temporary files directory");
        let record_path = temp.path().join("early.mcap");
        let ready_file = temp.path().join("early.mcap.ready");
        let mut config = BagdConfig::new(&record_path, Vec::new());
        config.ready_file = Some(ready_file.clone());
        let mut plan = recorder_only_gateway_plan(RecorderPlan {
            config,
            file: "early.mcap".to_string(),
            record_path: record_path.clone(),
            ready_file,
        });
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));

        let failure = match build(&mut plan, Arc::clone(&manager), &status) {
            Ok(_) => panic!("a recorder that exits before ready fails the build"),
            Err(failure) => failure,
        };

        assert!(
            matches!(failure.error, EmbedError::Recorder(_)),
            "{}",
            failure.error
        );
        assert!(record_path.exists(), "bagd owned the file once it spawned");
        assert_baseline(&manager);
    }

    /// `quiesce` brings a manager with a stopped gateway's leftovers (an
    /// open posture, flags, announces and a latched serving) to the
    /// baseline, and a second call changes nothing.
    #[test]
    fn quiesce_is_idempotent_and_repairs_a_partial_teardown() {
        let _serial = gateway_test_lock();
        let manager = network_manager("quiescetest");
        let served = format!("/quiesce/served/{}", std::process::id());
        let gateway = GatewayRuntime::new_with_schema_serving(
            Arc::clone(&manager),
            GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec![served.clone()],
                ingress: Vec::new(),
            },
            SchemaServing {
                topic_schemas: vec![TopicSchema {
                    topic: served.clone(),
                    schema_name: "std_msgs/String".to_string(),
                }],
                served_topics: vec![served.clone()],
                ..SchemaServing::default()
            },
        )
        .expect("gateway boot latches the query surface");
        drop(gateway);
        leave_stray_state(&manager, "/quiesce/stray");
        let network = manager.network().expect("network");
        let live = network
            .query_surface_serving_handles()
            .expect("the gateway latched the query surface");
        assert!(!live.is_serving_nothing_for_test());

        quiesce(&manager);
        assert_baseline(&manager);
        quiesce(&manager);
        assert_baseline(&manager);
    }

    /// The commit raises the beacon with no lock held; when the gateway
    /// thread exited in between, the beacon is handed back for dropping and
    /// the status keeps reporting no beacon.
    #[test]
    fn a_beacon_raised_after_the_gateway_exited_is_dropped_and_not_reported() {
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        let closed = Mutex::new(BeaconSlot::<()> {
            closed: true,
            guard: None,
        });
        assert_eq!(install_beacon(&closed, &status, Some(())), Some(()));
        assert!(lock_slot(&closed).guard.is_none());
        assert!(!lock_status(&status).mdns_advertised);

        let open = Mutex::new(BeaconSlot::<()> {
            closed: false,
            guard: None,
        });
        assert_eq!(install_beacon(&open, &status, Some(())), None);
        assert!(lock_slot(&open).guard.is_some());
        assert!(lock_status(&status).mdns_advertised);
    }

    #[test]
    fn a_panicking_gateway_thread_clears_mdns_advertised() {
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        lock_status(&status).mdns_advertised = true;
        let slot = Arc::new(Mutex::new(BeaconSlot {
            closed: false,
            guard: Some(()),
        }));
        let release = BeaconRelease {
            slot: Arc::clone(&slot),
            status: Arc::clone(&status),
        };

        let result = std::thread::spawn(move || {
            let _held = release;
            panic!("gateway unwound");
        })
        .join();

        assert!(result.is_err());
        assert!(!lock_status(&status).mdns_advertised);
        let slot = lock_slot(&slot);
        assert!(slot.closed);
        assert!(slot.guard.is_none());
    }

    #[test]
    fn clearing_a_missing_ready_marker_succeeds() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        clear_ready_marker(&temp.path().join("missing.ready"))
            .expect("a missing marker is already clear");
    }

    #[test]
    fn clearing_a_non_file_ready_marker_returns_a_recorder_error() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        let marker = temp.path().join("ready.marker");
        std::fs::create_dir(&marker).expect("create marker directory");
        std::fs::write(marker.join("child"), b"content").expect("make directory non-empty");

        let error = clear_ready_marker(&marker).expect_err("directory cannot be removed as a file");

        assert!(matches!(error, EmbedError::Recorder(_)));
        assert!(error
            .to_string()
            .starts_with("failed to clear recorder readiness marker:"));
    }

    #[test]
    fn the_first_reserve_creates_an_empty_recording_file() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        let path = temp.path().join("first.mcap");

        reserve_recording_path(&path, "first.mcap").expect("reserve a fresh path");

        assert_eq!(
            std::fs::read(&path).expect("reserved file"),
            Vec::<u8>::new()
        );
    }

    #[test]
    fn a_second_reserve_refuses_the_same_recording_file() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        let path = temp.path().join("taken.mcap");
        reserve_recording_path(&path, "taken.mcap").expect("first reserve");

        let error = reserve_recording_path(&path, "taken.mcap").expect_err("second reserve");

        assert!(matches!(error, EmbedError::Recorder(_)));
        assert!(error
            .to_string()
            .contains("recording file taken.mcap already exists"));
    }

    #[test]
    fn a_reserve_refuses_and_preserves_a_written_recording_file() {
        let temp = tempfile::tempdir().expect("temporary files directory");
        let path = temp.path().join("kept.mcap");
        let original = b"sentinel bytes that must survive";
        std::fs::write(&path, original).expect("seed existing recording");

        assert!(matches!(
            reserve_recording_path(&path, "kept.mcap"),
            Err(EmbedError::Recorder(_))
        ));
        assert_eq!(std::fs::read(&path).expect("existing file"), original);
    }

    #[test]
    fn install_iceoryx_config_roots_the_state_inside_the_app_sandbox() {
        let files_dir = tempfile::tempdir().expect("temporary files directory");
        install_iceoryx_config(files_dir.path()).expect("install iceoryx2 config");

        let path = files_dir.path().join("iceoryx2.toml");
        assert!(path.is_file());
        let file_path =
            FilePath::new(path.as_os_str().as_encoded_bytes()).expect("iceoryx2 config path");
        let mut expected = IoxConfig::default();
        let root_path = iceoryx2::prelude::Path::new(
            files_dir
                .path()
                .join("iceoryx2")
                .as_os_str()
                .as_encoded_bytes(),
        )
        .expect("sandbox root path");
        expected.global.set_root_path(&root_path);
        assert_eq!(
            IoxConfig::from_file(&file_path).expect("read iceoryx2 config"),
            expected
        );
    }

    const NODE_LIB_DIR_IN_PROCESS: &str = "node_lib_dir is not supported by the in-process app runtime; link the nodes into the app library with cerulion_android::embed_nodes!";

    #[test]
    fn in_process_transport_refuses_node_lib_dir_with_native_nodes() {
        let error = refuse_node_lib_dir_in_process(TransportKind::InProcess, 1, true)
            .expect_err("node_lib_dir with a native node must be refused");
        assert_eq!(error.code(), codes::GRAPH);
        assert_eq!(error.to_string(), NODE_LIB_DIR_IN_PROCESS);
    }

    #[test]
    fn in_process_transport_accepts_node_lib_dir_without_native_nodes() {
        assert!(refuse_node_lib_dir_in_process(TransportKind::InProcess, 0, true).is_ok());
    }

    #[test]
    fn in_process_transport_accepts_native_nodes_without_node_lib_dir() {
        assert!(refuse_node_lib_dir_in_process(TransportKind::InProcess, 1, false).is_ok());
    }

    #[test]
    fn cross_process_transport_accepts_node_lib_dir_with_native_nodes() {
        assert!(refuse_node_lib_dir_in_process(TransportKind::CrossProcess, 1, true).is_ok());
    }

    #[test]
    fn status_json_lists_linked_nodes() {
        let mut status = Status::new("robot".to_string());
        status.linked_nodes = vec!["printer".to_string(), "relay".to_string()];
        let json = serde_json::to_value(&status).expect("status JSON");
        assert_eq!(
            json["linked_nodes"],
            serde_json::json!(["printer", "relay"])
        );
    }

    #[test]
    fn status_json_lists_no_node_failures_before_a_graph_runs() {
        let status = Status::new("robot".to_string());
        assert!(status.node_failures.list().is_empty());
        let json = serde_json::to_value(&status).expect("status JSON");
        assert_eq!(json["node_failures"], serde_json::json!([]));
    }

    /// A graph whose one node `wheel` is the panic fixture: it panics on its
    /// third 50 ms tick.
    fn panic_fixture_graph() -> GraphRuntime {
        let config = parse_graph(
            "prefix: status_unit
nodes:
  - id: wheel
    type: test_node_macro_period_panic_cdylib
    outputs:
      - name: cmd
        schema: geometry_msgs/Vector3
",
        )
        .expect("parse the panic graph");
        let mut factories: indexmap::IndexMap<String, Box<dyn NodeEntry>> =
            indexmap::IndexMap::new();
        factories.insert(
            "wheel".to_string(),
            Box::new(test_node_macro_period_panic_cdylib::PeriodNodeEntry::new()),
        );
        let clock = Arc::new(cerulion_core::clock::VirtualClock::new());
        GraphRuntime::build_for_test(config, factories, clock, 4096).expect("build the panic graph")
    }

    #[test]
    fn node_failures_read_live_and_a_clone_keeps_what_it_read() {
        let mut graph = panic_fixture_graph();
        let live = NodeFailures::live(graph.node_tick_panic_records());
        let before = live.clone();
        // Ticks 1 and 2 publish; tick 3 panics.
        for _ in 0..3 {
            graph.step(Duration::from_millis(50));
        }
        let expected = vec![NodeFailure {
            node_id: "wheel".to_string(),
            message: "injected candidate crash at tick 3 (exit-3 twin)".to_string(),
        }];
        assert_eq!(live.list(), expected);
        assert!(before.list().is_empty(), "a clone is a snapshot");
        assert_ne!(live, before);
        assert_eq!(live.clone(), live);
        assert_eq!(
            format!("{live:?}"),
            "[NodeFailure { node_id: \"wheel\", message: \"injected candidate crash at tick 3 (exit-3 twin)\" }]"
        );
    }

    #[test]
    fn recorded_publisher_config_raises_default_base_to_tap_depths() {
        let transport = cerulion_core::testing::TestTransport::default();
        let base = transport.default_topic_config();
        let slot_len = MaxSliceLen::try_new(WireHeader::SIZE as u32 + 4096).unwrap();

        let raised = recorded_publisher_config(base, slot_len);

        assert_eq!(
            raised.subscriber_max_borrowed_samples,
            Some(RECORDING_SUBSCRIBER_MAX_BORROWED)
        );
        assert_eq!(
            raised.subscriber_max_buffer_size,
            recording_tap_buffer_depth(slot_len.get() as usize)
        );
    }

    #[test]
    fn recorded_publisher_config_keeps_a_base_already_above_the_raise() {
        let transport = cerulion_core::testing::TestTransport::default();
        let mut base = transport.default_topic_config();
        base.subscriber_max_borrowed_samples = Some(RECORDING_SUBSCRIBER_MAX_BORROWED + 10);
        let slot_len = MaxSliceLen::try_new(WireHeader::SIZE as u32 + 4096).unwrap();
        base.subscriber_max_buffer_size = recording_tap_buffer_depth(slot_len.get() as usize) + 10;

        let raised = recorded_publisher_config(base, slot_len);

        assert_eq!(raised, base);
    }

    #[test]
    fn recorded_failure_rebuilds_the_component_code_and_message() {
        for (code, expect) in [
            (codes::GRAPH, codes::GRAPH),
            (codes::NETWORK, codes::NETWORK),
            (codes::RECORDER, codes::RECORDER),
            (codes::SPAWN, codes::SPAWN),
            (codes::PANIC, codes::PANIC),
        ] {
            let mut status = Status::new("t".to_string());
            status.state = RuntimeState::Failed;
            status.last_error = Some(LastError {
                code,
                message: "boom".to_string(),
            });
            let error = recorded_failure(&status);
            assert_eq!(error.code(), expect);
            assert!(error.to_string().contains("boom"));
        }
        let mut status = Status::new("t".to_string());
        status.state = RuntimeState::Failed;
        let error = recorded_failure(&status);
        assert_eq!(error.code(), codes::PANIC);
        assert_eq!(
            error.to_string(),
            "a runtime component failed during startup"
        );
    }

    /// What a gateway constructor leaves when it fails after registering its
    /// announce topics: an enabled flag, an open posture and no thread. The
    /// teardown closes the posture and drops the flag.
    #[test]
    fn a_half_built_gateway_teardown_revokes_its_bridges_and_posture() {
        let _serial = gateway_test_lock();
        let manager = network_manager("halfbuilt");
        let mut run = Run::empty(Arc::clone(&manager));
        run.ownership = Some(GatewayOwnership::acquire(&manager).expect("lease"));
        let network = manager.network().expect("network");
        network.set_lan_access(LanAccess::Open);
        let flag = manager
            .bridge_manager()
            .register_topic("/half_built/topic")
            .expect("register bridge");
        flag.store(true, Ordering::Relaxed);

        let (_, error) = run.shutdown();

        assert!(error.is_none(), "{error:?}");
        assert!(
            !flag.load(Ordering::Relaxed),
            "a failed gateway construction must not leave its bridge enabled"
        );
        assert_baseline(&manager);
    }

    /// The commit holds the run's commit gate, not the status lock, from its
    /// last failure check through opening LAN access and replaying
    /// announces, so a worker failure cannot land between the check and the
    /// open while a status read still returns at once. Hand oracle: during
    /// the replay (the after-insert hook) the gate is held and the status
    /// lock is free; a worker failure raised there waits on the gate and
    /// lands only after the run is Running, so the commit succeeds and the
    /// failure is a started run's failure.
    #[test]
    fn a_worker_failure_cannot_land_between_the_commit_check_and_the_open() {
        let _serial = gateway_test_lock();
        let manager = network_manager("commitrace");
        let network = manager.network().expect("network");
        network.set_lan_access(LanAccess::Paired);
        manager
            .bridge_manager()
            .register_topic("/commit_race/topic")
            .expect("register bridge");
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        let mut run = Run::empty(Arc::clone(&manager));
        let gate_held = Arc::new(AtomicBool::new(false));
        let status_free = Arc::new(AtomicBool::new(false));
        let failer: Arc<Mutex<Option<JoinHandle<()>>>> = Arc::new(Mutex::new(None));
        {
            let status = Arc::clone(&status);
            let gate = Arc::clone(&run.commit_gate);
            let gate_held = Arc::clone(&gate_held);
            let status_free = Arc::clone(&status_free);
            let failer = Arc::clone(&failer);
            network.set_after_announce_insert_for_test(Some(Arc::new(move || {
                gate_held.store(gate.try_lock().is_err(), Ordering::SeqCst);
                status_free.store(status.try_lock().is_ok(), Ordering::SeqCst);
                let status = Arc::clone(&status);
                let gate = Arc::clone(&gate);
                *failer.lock().unwrap() = Some(thread::spawn(move || {
                    record_worker_failure(&gate, &status, codes::NETWORK, "late worker failure");
                }));
            })));
        }
        let (schemas, _) = SchemaSet::from_schemas(Vec::new()).expect("empty schema set");
        let plan = StartPlan {
            graph: None,
            graph_name: None,
            nodes: 0,
            schemas,
            publishes: Vec::new(),
            recorded_topics: None,
            recorded_graph_topics: None,
            listen: Vec::new(),
            gateway: Some(GatewayStartPlan {
                plan: GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: Vec::new(),
                    ingress: Vec::new(),
                },
                serving: SchemaServing::default(),
                lan_access: LanAccess::Open,
                mdns: false,
            }),
            recorder: None,
        };

        let result = commit(&mut run, &plan, &status, "t");
        network.set_after_announce_insert_for_test(None);
        let failer = failer.lock().unwrap().take().expect("the replay announced");
        failer.join().expect("failer thread");

        assert!(result.is_ok(), "{result:?}");
        assert!(
            gate_held.load(Ordering::SeqCst),
            "the commit gate must be held while LAN access opens"
        );
        assert!(
            status_free.load(Ordering::SeqCst),
            "a status read must not wait out the commit's network steps"
        );
        assert_eq!(network.lan_access(), LanAccess::Open);
        assert_eq!(lock_status(&status).state, RuntimeState::Failed);
        let (_, error) = run.shutdown();
        assert!(error.is_none(), "{error:?}");
    }

    /// A worker that failed before the commit keeps LAN access paired: the
    /// commit refuses before it writes the posture.
    #[test]
    fn a_worker_failure_before_the_commit_keeps_lan_access_paired() {
        let _serial = gateway_test_lock();
        let manager = network_manager("commitfailed");
        let network = manager.network().expect("network");
        network.set_lan_access(LanAccess::Paired);
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        set_thread_error(&status, codes::GRAPH, "early worker failure");
        let (schemas, _) = SchemaSet::from_schemas(Vec::new()).expect("empty schema set");
        let plan = StartPlan {
            graph: None,
            graph_name: None,
            nodes: 0,
            schemas,
            publishes: Vec::new(),
            recorded_topics: None,
            recorded_graph_topics: None,
            listen: Vec::new(),
            gateway: Some(GatewayStartPlan {
                plan: GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: Vec::new(),
                    ingress: Vec::new(),
                },
                serving: SchemaServing::default(),
                lan_access: LanAccess::Open,
                mdns: false,
            }),
            recorder: None,
        };
        let mut run = Run::empty(Arc::clone(&manager));

        let result = commit(&mut run, &plan, &status, "t");

        assert!(matches!(result, Err(EmbedError::Graph(_))), "{result:?}");
        assert_eq!(network.lan_access(), LanAccess::Paired);
        let (_, error) = run.shutdown();
        assert!(error.is_none(), "{error:?}");
    }

    #[test]
    fn a_panicked_worker_still_yields_the_recorder_summary() {
        let mut run = Run::empty(local_manager("panicked_worker"));
        run.graph = Some(GraphWorker {
            running: Arc::new(AtomicBool::new(true)),
            ready: Arc::new(AtomicBool::new(true)),
            thread: std::thread::spawn(|| -> Result<(), String> {
                panic!("graph worker unwound");
            }),
        });
        run.recorder = Some(RecorderWorker {
            shutdown: Arc::new(AtomicBool::new(false)),
            thread: std::thread::spawn(|| Ok(bagd_summary(3))),
        });

        let (summary, error) = run.shutdown();

        let summary = summary.expect("a worker panic must not lose the recorder summary");
        assert_eq!(summary.messages, 3);
        assert!(matches!(error, Some(EmbedError::Graph(_))));
    }

    #[test]
    fn graph_exit_records_errors_as_graph_failures() {
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        let running = AtomicBool::new(true);

        record_graph_exit(&status, &running, &Err("graph error".to_string()));

        let status = lock_status(&status);
        assert_eq!(status.state, RuntimeState::Failed);
        assert_eq!(
            status.last_error,
            Some(LastError {
                code: codes::GRAPH,
                message: "graph error".to_string(),
            })
        );
    }

    #[test]
    fn graph_exit_finishes_running_and_starting_graphs() {
        for state in [RuntimeState::Running, RuntimeState::Starting] {
            let status = Arc::new(Mutex::new(Status::new("t".to_string())));
            lock_status(&status).state = state;
            let running = AtomicBool::new(true);

            record_graph_exit(&status, &running, &Ok(()));

            assert_eq!(lock_status(&status).state, RuntimeState::Finished);
        }
    }

    #[test]
    fn graph_exit_preserves_status_when_shutdown_is_in_progress_or_failed() {
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        lock_status(&status).state = RuntimeState::Running;
        let running = AtomicBool::new(false);

        record_graph_exit(&status, &running, &Ok(()));

        assert_eq!(lock_status(&status).state, RuntimeState::Running);

        lock_status(&status).state = RuntimeState::Stopping;
        running.store(true, Ordering::Relaxed);
        record_graph_exit(&status, &running, &Ok(()));
        assert_eq!(lock_status(&status).state, RuntimeState::Stopping);

        let failure = LastError {
            code: codes::NETWORK,
            message: "existing failure".to_string(),
        };
        {
            let mut status = lock_status(&status);
            status.state = RuntimeState::Failed;
            status.last_error = Some(failure.clone());
        }
        record_graph_exit(&status, &running, &Ok(()));
        let status = lock_status(&status);
        assert_eq!(status.state, RuntimeState::Failed);
        assert_eq!(status.last_error, Some(failure));
    }
}

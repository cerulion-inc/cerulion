// SPDX-License-Identifier: AGPL-3.0-only
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use cerulion_bagd::{run_bagd, BagdConfig, BagdSummary, RecordedTopic, RECORDED_TOPIC_FORMAT};
use cerulion_cli_engine::graph_cmd::{cdylib_name, resolve_recorded_topics};
use cerulion_cli_engine::schema_cmd::parse_builtin_schemas;
use cerulion_core::clock::{Clock, RealClock};
use cerulion_core::dynamic::{SchemaSet, WireLayout};
use cerulion_core::graph::{
    parse_graph, resolve_output_topic, DylibNodeEntry, GraphConfig, NodeEntry,
};
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, TopicSchema,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{
    recording_tap_buffer_depth, TopicServiceConfig, TransportConfig, TransportKind,
    TransportManager, RECORDING_SUBSCRIBER_MAX_BORROWED, TRANSPORT_KIND,
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
}

impl Status {
    fn new(robot: String) -> Self {
        Self {
            state: RuntimeState::Idle,
            robot,
            graph: None,
            nodes: 0,
            listen: Vec::new(),
            lan_access: None,
            mdns_advertised: false,
            record_path: None,
            published: BTreeMap::new(),
            recorder: None,
            last_error: None,
        }
    }
}

/// Synchronous owner of embedded graph, gateway, recorder, and publisher resources.
/// Dropping it stops and joins whatever is still running.
pub struct EmbeddedRuntime {
    files_dir: PathBuf,
    robot_name: String,
    status: Arc<Mutex<Status>>,
    resources: Resources,
}

#[derive(Default)]
struct Resources {
    manager: Option<Arc<TransportManager>>,
    publishers: BTreeMap<String, PublisherState>,
    graph_running: Option<Arc<AtomicBool>>,
    graph_thread: Option<JoinHandle<Result<(), String>>>,
    gateway_exit: Option<Arc<AtomicBool>>,
    gateway_thread: Option<JoinHandle<Result<(), String>>>,
    recorder_shutdown: Option<Arc<AtomicBool>>,
    recorder_thread: Option<JoinHandle<Result<BagdSummary, String>>>,
}

/// The gateway thread's mDNS beacon; dropping it withdraws the beacon and
/// clears `mdns_advertised`, on a normal return and on unwind alike.
struct AdvertisedBeacon {
    beacon: Option<MdnsAdvertiseGuard>,
    status: Arc<Mutex<Status>>,
}

impl Drop for AdvertisedBeacon {
    fn drop(&mut self) {
        drop(self.beacon.take());
        lock_status(&self.status).mdns_advertised = false;
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
            resources: Resources::default(),
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

    pub(crate) fn start_accepted(&mut self, config_json: &str) -> Result<(), EmbedError> {
        // A component of a failed run or a finished graph may have left
        // siblings running; tear them down BEFORE the new config is
        // validated, so a rejected retry cannot leave the old gateway or
        // recorder active behind the refusal. A teardown failure is already
        // recorded in status, so warn and continue.
        if self.resources.has_any() {
            let (_summary, teardown_error) =
                stop_resource_set(&mut self.resources, &self.status, false);
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
        {
            let mut status = lock_status(&self.status);
            status.state = RuntimeState::Starting;
            status.graph = None;
            status.nodes = 0;
            status.listen = config.listen.clone();
            status.lan_access = None;
            status.mdns_advertised = false;
            status.record_path = None;
            status.published.clear();
            status.recorder = None;
            status.last_error = None;
        }
        match self.start_inner(config) {
            Ok(()) => Ok(()),
            Err(error) => {
                set_error(&self.status, &error);
                Err(error)
            }
        }
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
            .resources
            .publishers
            .get(topic)
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
            .resources
            .publishers
            .get_mut(topic)
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
        let (summary, error) = self.stop_resources();
        set_state(&self.status, RuntimeState::Stopped);
        // The summary is observable state (Principle #3): it is recorded even
        // when teardown itself reports an error.
        lock_status(&self.status).recorder = summary.clone();
        if let Some(error) = error {
            return Err(error);
        }
        Ok(summary)
    }

    fn start_inner(&mut self, config: EmbeddedConfig) -> Result<(), EmbedError> {
        install_iceoryx_config(&self.files_dir)?;
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

        let mut graph_inputs = graph
            .map(|graph| self.load_graph(graph, &config))
            .transpose()?;
        let resolved_publish = config
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
        if let Some(topic) = resolved_publish
            .iter()
            .map(|publish| publish.topic.as_str())
            .find(|topic| graph_topic_set.contains(topic))
        {
            return Err(EmbedError::InvalidArgument(format!(
                "publish topic '{topic}' is a graph output; graph topics are single-writer"
            )));
        }
        let publish_topics: HashSet<&str> = resolved_publish
            .iter()
            .map(|item| item.topic.as_str())
            .collect();
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
                .chain(resolved_publish.iter().map(|item| item.topic.clone()))
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

        let extra = resolved_publish
            .iter()
            .map(|publish| TopicSchema {
                topic: publish.topic.clone(),
                schema_name: publish.schema.clone(),
            })
            .collect::<Vec<_>>();
        let serving = cerulion_cli_engine::schema_serve::build_app_schema_serving(
            graph_inputs.as_ref().map(|inputs| &inputs.config),
            &schema_dir,
            &extra,
        );
        let manager = self.transport_manager(&config.listen)?;
        // Only a start that creates the gateway may replace the serving
        // snapshot or change manager-wide LAN state: a listen-less app start
        // leaves the prior run's catalog and posture untouched. For a
        // gateway start the new catalog/schema snapshot installs BEFORE LAN
        // access opens - the process-wide query surface may still be serving
        // the prior run's registry, and an open gate would otherwise answer
        // a demand with a stale topic for the window between. The prior
        // serving is kept for rollback if the start fails.
        let mut prior_serving = None;
        // The applied posture exists only on a gateway start; a listen-less
        // start reports no LAN access of its own.
        let applied_lan_access =
            (!config.listen.is_empty()).then(|| LanAccess::from(config.lan_access));
        if !config.listen.is_empty() {
            if let Some(live) = manager
                .network()
                .and_then(|network| network.query_surface_serving_handles())
            {
                prior_serving = Some((live.snapshot_for_rollback(), live));
            }
            // The replace snapshot must cover EVERY served topic - including
            // schema-less outputs, which carry no topic_schemas binding - or
            // the filtered catalog would drop them on a restart.
            let mut served = serving.clone();
            {
                let mut announce = graph_topics.clone();
                for publish in &resolved_publish {
                    if !announce.contains(&publish.topic) {
                        announce.push(publish.topic.clone());
                    }
                }
                served.served_topics = announce;
            }
            if let Some((_, live)) = &prior_serving {
                live.replace(&served);
            }
            if let Some(network) = manager.network() {
                network.set_lan_access(applied_lan_access.unwrap_or(LanAccess::Paired));
            }
        }

        let mut resources = Resources {
            manager: Some(Arc::clone(&manager)),
            ..Resources::default()
        };
        let result = (|| {
            self.create_publishers(
                &resolved_publish,
                &manager,
                recorded_topics.as_ref(),
                &mut resources,
            )?;

            let mut graph_runtime = if let Some(inputs) = graph_inputs.as_mut() {
                let clock: Arc<dyn Clock> = Arc::new(RealClock);
                let schema_hashes = schema_hash_map(&schemas);
                let mut graph_runtime =
                    GraphRuntime::build_live_with_schema_hashes_policy_and_recorded(
                        inputs.config.clone(),
                        std::mem::take(&mut inputs.factories),
                        &manager,
                        Arc::clone(&clock),
                        Some(&schema_hashes),
                        cerulion_cli_engine::graph_cmd::embedded_live_monitor_wait_policy(),
                        recorded_graph_topics.as_ref(),
                    )
                    .map_err(|error| EmbedError::Graph(error.to_string()))?;
                graph_runtime.set_live_transport(Arc::clone(&manager));
                Some(graph_runtime)
            } else {
                None
            };

            // Workers bump this as their closure's first line; the starter
            // waits for every spawned worker to begin before reporting
            // success, so a worker that failed during startup has already
            // recorded Failed before we check `status.state` below.
            let workers_began = Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let mut workers_expected = 0usize;
            if !config.listen.is_empty() {
                let mut announce = graph_topics;
                for publish in &resolved_publish {
                    if !announce.contains(&publish.topic) {
                        announce.push(publish.topic.clone());
                    }
                }
                // The replace snapshot must cover EVERY served topic -
                // including schema-less outputs, which carry no topic_schemas
                // binding - or the filtered catalog would drop them.
                let mut serving_clone = serving.clone();
                serving_clone.served_topics = announce.clone();
                let plan = GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce,
                    ingress: Vec::new(),
                };
                // The process-wide query surface may outlive the prior
                // gateway: install the new snapshot BEFORE construction
                // registers topics and exposes the restarted gateway, or the
                // latched callback keeps filtering through the prior snapshot.
                if let Some(live) = manager
                    .network()
                    .and_then(|network| network.query_surface_serving_handles())
                {
                    live.replace(&serving_clone);
                }
                let gateway =
                    GatewayRuntime::new_with_schema_serving(Arc::clone(&manager), plan, serving)
                        .map_err(|error| EmbedError::Network(error.to_string()))?;
                let exit = Arc::new(AtomicBool::new(false));
                let thread_exit = Arc::clone(&exit);
                let beacon = if config.mdns {
                    advertise_beacon(&self.robot_name, &config.listen)
                } else {
                    None
                };
                lock_status(&self.status).mdns_advertised = beacon.is_some();
                let status = Arc::clone(&self.status);
                let advertised = AdvertisedBeacon {
                    beacon,
                    status: Arc::clone(&status),
                };
                let began = Arc::clone(&workers_began);
                workers_expected += 1;
                resources.gateway_thread = Some(
                    thread::Builder::new()
                        .name("cerulion-gateway".to_string())
                        .spawn(move || {
                            began.fetch_add(1, std::sync::atomic::Ordering::Release);
                            let mut gateway = gateway;
                            // A panic must not leave the status reporting
                            // Running: record it as a worker failure the same
                            // way a returned error is recorded.
                            let result =
                                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    gateway.run(&thread_exit).map_err(|error| error.to_string())
                                }))
                                .unwrap_or_else(|_| Err("gateway worker panicked".to_string()));
                            drop(advertised);
                            if let Err(error) = &result {
                                set_thread_error(&status, codes::NETWORK, error);
                            }
                            result
                        })
                        .map_err(|error| EmbedError::Network(error.to_string()))?,
                );
                resources.gateway_exit = Some(exit);
            }

            let bagd_plan =
                self.prepare_recorder(&config, graph_inputs.as_ref(), &resolved_publish)?;
            if let Some((bagd_config, ready_file, record_path)) = bagd_plan {
                let shutdown = Arc::new(AtomicBool::new(false));
                let thread_shutdown = Arc::clone(&shutdown);
                let recorder_manager = Arc::clone(&manager);
                let status = Arc::clone(&self.status);
                let began = Arc::clone(&workers_began);
                workers_expected += 1;
                let spawn = thread::Builder::new()
                    .name("cerulion-bagd".to_string())
                    .spawn(move || {
                        began.fetch_add(1, std::sync::atomic::Ordering::Release);
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            run_bagd(recorder_manager, bagd_config, thread_shutdown)
                                .map_err(|error| error.to_string())
                        }))
                        .unwrap_or_else(|_| Err("recorder worker panicked".to_string()));
                        if let Err(error) = &result {
                            set_thread_error(&status, codes::RECORDER, error);
                        }
                        result
                    });
                match spawn {
                    Ok(thread) => resources.recorder_thread = Some(thread),
                    Err(error) => {
                        // The reserved path would read as taken to the next
                        // start, so release it before returning the error.
                        // Best-effort cleanup of an already-failing start.
                        let _ = std::fs::remove_file(&record_path);
                        return Err(EmbedError::Recorder(error.to_string()));
                    }
                }
                resources.recorder_shutdown = Some(shutdown);
                lock_status(&self.status).record_path =
                    Some(record_path.to_string_lossy().into_owned());
                self.wait_recorder_ready(&ready_file, &mut resources)?;
            }

            if let Some(mut graph_runtime) = graph_runtime.take() {
                let running = Arc::new(AtomicBool::new(true));
                let thread_running = Arc::clone(&running);
                let status = Arc::clone(&self.status);
                let began = Arc::clone(&workers_began);
                workers_expected += 1;
                resources.graph_thread = Some(
                    thread::Builder::new()
                        .name("cerulion-graph".to_string())
                        .spawn(move || {
                            began.fetch_add(1, std::sync::atomic::Ordering::Release);
                            // A panic unwinds past `run_live`'s own exit
                            // bookkeeping, so surface it as a worker error:
                            // `record_graph_exit` then marks the runtime
                            // Failed instead of leaving it Running.
                            let result =
                                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    graph_runtime
                                        .run_live(&thread_running)
                                        .map_err(|error| error.to_string())
                                }))
                                .unwrap_or_else(|_| Err("graph worker panicked".to_string()));
                            record_graph_exit(&status, &thread_running, &result);
                            result
                        })
                        .map_err(|error| EmbedError::Graph(error.to_string()))?,
                );
                resources.graph_running = Some(running);
            }

            {
                let mut status = lock_status(&self.status);
                status.graph = graph_inputs_name(&graph_inputs);
                status.nodes = graph_inputs
                    .as_ref()
                    .map_or(0, |inputs| inputs.config.nodes.len());
                status.listen = config.listen;
                status.lan_access = applied_lan_access.map(|access| access.as_str().to_string());
                // The startup barrier: wait until every spawned worker has
                // entered its run loop (or exited). Without it a worker can
                // fail after spawn but before its error lands, and this block
                // would overwrite Failed with Running.
                let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
                while workers_began.load(std::sync::atomic::Ordering::Acquire)
                    < workers_expected
                    && std::time::Instant::now() < deadline
                {
                    // A worker that already exited finished past its error
                    // recording, so it counts as begun for this barrier.
                    let exited = usize::from(
                        resources
                            .gateway_thread
                            .as_ref()
                            .is_some_and(|thread| thread.is_finished()),
                    ) + usize::from(
                        resources
                            .recorder_thread
                            .as_ref()
                            .is_some_and(|thread| thread.is_finished()),
                    ) + usize::from(
                        resources
                            .graph_thread
                            .as_ref()
                            .is_some_and(|thread| thread.is_finished()),
                    );
                    if workers_began.load(std::sync::atomic::Ordering::Acquire) + exited
                        >= workers_expected
                    {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
                // A worker thread that failed during startup already
                // flipped the state to Failed; returning Ok here would leave
                // a half-started runtime reporting success, so surface the
                // recorded failure and let the caller tear down below.
                if matches!(status.state, RuntimeState::Failed) {
                    return Err(recorded_failure(&status));
                }
                if matches!(status.state, RuntimeState::Finished) {
                    return Ok(());
                }
                status.state = RuntimeState::Running;
                status.last_error = None;
            }
            Ok(())
        })();
        if let Err(error) = result {
            let _ = stop_resource_set(&mut resources, &self.status, true);
            // Roll the serving snapshot back to the pre-start state: the
            // failed start must not leave the query surface cataloging this
            // run's topics.
            if let Some((prior, live)) = prior_serving {
                live.restore_snapshot(prior);
            }
            return Err(error);
        }
        self.resources = resources;
        Ok(())
    }

    fn load_graph(
        &self,
        config: GraphConfig,
        embedded: &EmbeddedConfig,
    ) -> Result<GraphInputs, EmbedError> {
        let native_nodes = config.native_nodes().count();
        refuse_dlopen_nodes(TRANSPORT_KIND, native_nodes)?;
        if native_nodes != config.nodes.len() {
            return Err(EmbedError::Graph(
                "embedded runtime supports only native cdylib graph nodes".to_string(),
            ));
        }
        let node_lib_dir = embedded.node_lib_dir.as_ref().ok_or_else(|| {
            EmbedError::Graph(
                "node_lib_dir is required when graph_yaml contains native nodes".to_string(),
            )
        })?;
        if !node_lib_dir.is_absolute() || !node_lib_dir.is_dir() {
            return Err(EmbedError::InvalidArgument(
                "node_lib_dir must be an existing absolute directory".to_string(),
            ));
        }
        let mut factories = indexmap::IndexMap::new();
        let mut infos = indexmap::IndexMap::new();
        for node in &config.nodes {
            let path = node_lib_dir.join(cdylib_name(&node.node_type));
            let entry = DylibNodeEntry::load(&path)
                .map_err(|error| EmbedError::Graph(error.to_string()))?;
            let info = entry
                .info()
                .map_err(|error| EmbedError::Graph(error.to_string()))?;
            infos.insert(node.id.clone(), info);
            factories.insert(node.id.clone(), Box::new(entry) as Box<dyn NodeEntry>);
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
        *identity = Some((listen.to_vec(), self.robot_name.clone()));
        Ok(manager)
    }

    fn create_publishers(
        &self,
        config: &[ResolvedPublish],
        manager: &TransportManager,
        recorded_topics: Option<&HashSet<String>>,
        resources: &mut Resources,
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
            // A RECORDED app topic gets the same tap
            // provisioning the graph build applies to recorded graph outputs;
            // a non-recorded publisher keeps the default config.
            let publisher = if recorded_topics.is_some_and(|set| set.contains(&item.topic)) {
                manager.create_publisher_with_topic_config(
                    &item.topic,
                    slot_len,
                    0,
                    recorded_publisher_config(manager.default_topic_config(), slot_len),
                )
            } else {
                manager.create_publisher(&item.topic, slot_len, 0)
            }
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
            resources.publishers.insert(
                item.topic.clone(),
                PublisherState {
                    publisher,
                    layout: item.layout.clone(),
                    max_payload_bytes: item.max_payload_bytes,
                    sequence: 0,
                    frame: Vec::new(),
                },
            );
        }
        Ok(())
    }

    fn prepare_recorder(
        &self,
        embedded: &EmbeddedConfig,
        graph: Option<&GraphInputs>,
        publishes: &[ResolvedPublish],
    ) -> Result<Option<(BagdConfig, PathBuf, PathBuf)>, EmbedError> {
        let Some(record) = &embedded.record else {
            return Ok(None);
        };
        let bags_dir = self.files_dir.join("bags");
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
                    &self.files_dir,
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
        let schema_dir = self.files_dir.join("schemas");
        config.schema_catalog = cerulion_cli_engine::bag_cmd::build_record_schema_catalog(
            schema_dir.is_dir().then_some(schema_dir.as_path()),
            None,
        );
        reserve_recording_path(&record_path, &record.file)?;
        if let Err(error) = clear_ready_marker(&ready_file) {
            // The reservation belongs to this start, so leave the name
            // available when marker cleanup prevents startup.
            let _ = std::fs::remove_file(&record_path);
            return Err(error);
        }
        Ok(Some((config, ready_file, record_path)))
    }

    fn wait_recorder_ready(
        &self,
        ready_file: &Path,
        resources: &mut Resources,
    ) -> Result<(), EmbedError> {
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            if ready_file.exists() {
                return Ok(());
            }
            if resources
                .recorder_thread
                .as_ref()
                .is_some_and(JoinHandle::is_finished)
            {
                let Some(thread) = resources.recorder_thread.take() else {
                    return Err(EmbedError::Recorder(
                        "bagd exited before signaling readiness".to_string(),
                    ));
                };
                return match thread.join() {
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

    fn stop_resources(&mut self) -> (Option<RecorderSummary>, Option<EmbedError>) {
        stop_resource_set(&mut self.resources, &self.status, false)
    }
}

impl Drop for EmbeddedRuntime {
    fn drop(&mut self) {
        if !self.resources.has_any() {
            return;
        }
        set_state(&self.status, RuntimeState::Stopping);
        let (summary, error) = self.stop_resources();
        match error {
            None => {
                set_state(&self.status, RuntimeState::Stopped);
                lock_status(&self.status).recorder = summary;
            }
            // A failure here already recorded its own status through
            // `set_thread_error`; do not overwrite it with Stopped.
            Some(error) => {
                tracing::warn!(error = %error, "embedded runtime: stop on drop failed");
            }
        }
    }
}

impl Resources {
    /// Whether any component or handle from a previous start survives.
    fn has_any(&self) -> bool {
        self.manager.is_some()
            || !self.publishers.is_empty()
            || self.graph_running.is_some()
            || self.graph_thread.is_some()
            || self.gateway_exit.is_some()
            || self.gateway_thread.is_some()
            || self.recorder_shutdown.is_some()
            || self.recorder_thread.is_some()
    }
}

/// The topic service config an app publisher on a
/// RECORDED topic creates the service with: the graph build's recorded-topic
/// raise applied to a default-opener base, so the bagd tap gets the same
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

fn graph_inputs_name(graph: &Option<GraphInputs>) -> Option<String> {
    graph
        .as_ref()
        .map(|inputs| inputs.config.identity().to_string())
}

/// Claim the recording path atomically so no second runtime can pass the
/// same `record.file` and have bagd truncate the first bag: `create_new`
/// refuses an existing path, and `BagWriter::create` later truncates only
/// the empty file reserved here. Called only after every fallible
/// validation step, so a refused config leaves the name free; the one
/// failure after it (the bagd spawn) releases the file.
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

fn refuse_dlopen_nodes(kind: TransportKind, native_nodes: usize) -> Result<(), EmbedError> {
    if kind == TransportKind::InProcess && native_nodes > 0 {
        return Err(EmbedError::Graph(
            "graph nodes loaded from libraries cannot deliver frames under the in-process transport; run the app without graph_yaml".to_string(),
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

/// Stop every component and join its thread. Returns the recorder summary
/// (if any) and the first teardown error separately, so the caller can store
/// the summary in status BEFORE propagating the error.
fn stop_resource_set(
    resources: &mut Resources,
    status: &Arc<Mutex<Status>>,
    starting_failure: bool,
) -> (Option<RecorderSummary>, Option<EmbedError>) {
    if let Some(network) = resources
        .manager
        .as_ref()
        .and_then(|manager| manager.network())
    {
        // Close LAN access before any join: a join that blocks must not leave
        // an open gateway serving LAN peers. Process-global demand workers
        // outlive this runtime; the closed posture revokes grants until the next
        // start applies its configured posture.
        network.set_lan_access(LanAccess::Paired);
    }
    let mut first_error = None;
    if let Some(running) = resources.graph_running.take() {
        running.store(false, Ordering::Relaxed);
    }
    if let Some(thread) = resources.graph_thread.take() {
        match thread.join() {
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

    let mut summary = None;
    if starting_failure {
        summary = stop_recorder(resources, &mut first_error);
    }
    if let Some(exit) = resources.gateway_exit.take() {
        exit.store(true, Ordering::Relaxed);
        if let Some(manager) = &resources.manager {
            manager.bridge_manager().demand_signal().signal();
        }
    }
    if let Some(thread) = resources.gateway_thread.take() {
        match thread.join() {
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
    if let Some(manager) = &resources.manager {
        let bridges = manager.bridge_manager();
        match bridges.registered_topics() {
            Ok(topics) => {
                for (topic, _) in topics {
                    if let Err(error) = bridges.disable_bridge(&topic) {
                        tracing::warn!(
                            topic = %topic,
                            error = %error,
                            "could not disable the network bridge while stopping the runtime"
                        );
                    }
                }
            }
            Err(error) => tracing::warn!(
                error = %error,
                "could not enumerate network bridges while stopping the runtime"
            ),
        }
    }
    lock_status(status).mdns_advertised = false;
    if !starting_failure {
        summary = stop_recorder(resources, &mut first_error);
    }
    resources.publishers.clear();
    resources.manager = None;
    if let Some(error) = first_error {
        set_error(status, &error);
        if starting_failure {
            return (summary, None);
        }
        return (summary, Some(error));
    }
    (summary, None)
}

fn stop_recorder(
    resources: &mut Resources,
    first_error: &mut Option<EmbedError>,
) -> Option<RecorderSummary> {
    if let Some(shutdown) = resources.recorder_shutdown.take() {
        shutdown.store(true, Ordering::Relaxed);
    }
    let thread = resources.recorder_thread.take()?;
    match thread.join() {
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

fn lock_status(status: &Arc<Mutex<Status>>) -> std::sync::MutexGuard<'_, Status> {
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

    #[test]
    fn a_panicking_gateway_thread_clears_mdns_advertised() {
        let status = Arc::new(Mutex::new(Status::new("t".to_string())));
        lock_status(&status).mdns_advertised = true;
        let guard = AdvertisedBeacon {
            beacon: None,
            status: Arc::clone(&status),
        };

        let result = std::thread::spawn(move || {
            let _held = guard;
            panic!("gateway unwound");
        })
        .join();

        assert!(result.is_err());
        assert!(!lock_status(&status).mdns_advertised);
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

    #[test]
    fn in_process_transport_refuses_native_nodes() {
        let error = refuse_dlopen_nodes(TransportKind::InProcess, 1)
            .expect_err("library node must be refused");
        assert_eq!(error.code(), codes::GRAPH);
        assert_eq!(
            error.to_string(),
            "graph nodes loaded from libraries cannot deliver frames under the in-process transport; run the app without graph_yaml"
        );
    }

    #[test]
    fn in_process_transport_accepts_zero_native_nodes() {
        assert!(refuse_dlopen_nodes(TransportKind::InProcess, 0).is_ok());
    }

    #[test]
    fn cross_process_transport_accepts_native_nodes() {
        assert!(refuse_dlopen_nodes(TransportKind::CrossProcess, 1).is_ok());
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

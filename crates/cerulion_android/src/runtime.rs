// SPDX-License-Identifier: AGPL-3.0-only
use std::collections::{BTreeMap, HashMap, HashSet};
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
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{CerulionPublisher, GraphRuntime};
use serde::Serialize;

use crate::config::{validate_robot_name, EmbeddedConfig, PublishConfig};
use crate::encode::encode_json;
use crate::error::{codes, EmbedError};

static PROCESS_LISTEN_ENDPOINTS: OnceLock<Mutex<Option<Vec<String>>>> = OnceLock::new();

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
pub struct RecorderStatus {
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
    pub state: String,
    /// Robot identity.
    pub robot: String,
    /// Parsed graph name, if configured.
    pub graph: Option<String>,
    /// Number of graph nodes.
    pub nodes: usize,
    /// Network listen endpoints.
    pub listen: Vec<String>,
    /// Configured recording path.
    pub record_path: Option<String>,
    /// Successful publish count by topic.
    pub published: BTreeMap<String, u64>,
    /// Recorder totals, available after stop.
    pub recorder: Option<RecorderStatus>,
    /// Most recent runtime error.
    pub last_error: Option<LastError>,
}

impl Status {
    fn new(robot: String) -> Self {
        Self {
            state: "idle".to_string(),
            robot,
            graph: None,
            nodes: 0,
            listen: Vec::new(),
            record_path: None,
            published: BTreeMap::new(),
            recorder: None,
            last_error: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
/// Totals returned when the runtime stops.
pub struct StopSummary {
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

/// Synchronous owner of embedded graph, gateway, recorder, and publisher resources.
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

struct PublisherState {
    publisher: CerulionPublisher,
    layout: WireLayout,
    schema_hash: u64,
    max_payload_bytes: usize,
    sequence: u32,
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
            self.snapshot().state.as_str(),
            "starting" | "running" | "stopping"
        ) {
            return Err(EmbedError::BadState(
                "runtime is already starting or running".to_string(),
            ));
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
            status.state = "starting".to_string();
            status.graph = None;
            status.nodes = 0;
            status.listen = config.listen.clone();
            status.record_path = None;
            status.published.clear();
            status.recorder = None;
            status.last_error = None;
        }
        match self.start_inner(config) {
            Ok(()) => Ok(()),
            Err(error) => {
                let _ = self.stop_resources(true);
                set_error(&self.status, &error);
                Err(error)
            }
        }
    }

    /// Publish a JSON object and return its wire sequence number.
    pub fn publish(&mut self, topic: &str, json: &str) -> Result<u32, EmbedError> {
        if self.snapshot().state != "running" {
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
        let frame = encode_json(&state.layout, json, state.max_payload_bytes, timestamp)?;
        let encoded_header = WireHeader::read_from_buf(&frame[..WireHeader::SIZE])
            .ok_or_else(|| EmbedError::Encode("encoder returned a short wire header".into()))?;
        let payload = &frame[WireHeader::SIZE..];
        let total_len = WireHeader::SIZE + payload.len();
        let mut loan = state
            .publisher
            .loan_raw_uninit(total_len)
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
        for (destination, source) in loan.bytes_uninit_mut()[WireHeader::SIZE..]
            .iter_mut()
            .zip(payload)
        {
            destination.write(*source);
        }
        for byte in &mut loan.bytes_uninit_mut()[..WireHeader::SIZE] {
            byte.write(0);
        }
        // SAFETY: the body loop initializes every payload byte and the zero-fill
        // initializes the complete header range before the loan is assumed initialized.
        let mut loan = unsafe { loan.assume_init() };
        let mut header = WireHeader::new(state.schema_hash, state.sequence, timestamp);
        header.total_size = total_len as u32;
        header.offset_table_offset = encoded_header.offset_table_offset;
        header.offset_table_count = encoded_header.offset_table_count;
        header.write_to_buf(&mut loan.bytes_mut()[..WireHeader::SIZE]);
        state
            .publisher
            .send_raw_loan(loan)
            .map_err(|error| EmbedError::Publish(error.to_string()))?;
        state.sequence = state.sequence.wrapping_add(1);
        if let Err(error) = state.publisher.notify_sent_sample() {
            tracing::debug!(topic, error = %error, "notify failed after embedded publish");
        }
        let mut status = lock_status(&self.status);
        *status.published.entry(topic.to_string()).or_default() += 1;
        Ok(state.sequence.wrapping_sub(1))
    }

    /// Return the shared runtime status handle.
    pub fn status_handle(&self) -> Arc<Mutex<Status>> {
        Arc::clone(&self.status)
    }

    /// Stop all started components and join their threads.
    pub fn stop(&mut self) -> Result<StopSummary, EmbedError> {
        if self.snapshot().state != "running" && self.snapshot().state != "failed" {
            return Err(EmbedError::BadState(
                "stop requires a started runtime".to_string(),
            ));
        }
        set_state(&self.status, "stopping");
        let summary = self.stop_resources(false)?;
        set_state(&self.status, "stopped");
        if let Some(summary) = &summary {
            let snapshot = RecorderStatus {
                messages: summary.messages,
                channels: summary.channels,
                bytes: summary.bytes,
                per_topic: summary.per_topic.clone(),
                bag_paths: summary.bag_paths.clone(),
                out_path: summary.out_path.clone(),
            };
            lock_status(&self.status).recorder = Some(snapshot);
        }
        Ok(summary.unwrap_or(StopSummary {
            messages: 0,
            channels: 0,
            bytes: 0,
            per_topic: BTreeMap::new(),
            bag_paths: Vec::new(),
            out_path: None,
        }))
    }

    fn snapshot(&self) -> Status {
        lock_status(&self.status).clone()
    }

    fn start_inner(&mut self, config: EmbeddedConfig) -> Result<(), EmbedError> {
        let graph = config
            .graph_yaml
            .as_deref()
            .map(parse_graph)
            .transpose()
            .map_err(|error| EmbedError::Graph(error.to_string()))?;
        let schema_dir = self.files_dir.join("schemas");
        let builtins = parse_builtin_schemas();
        let (schemas, _) = if schema_dir.is_dir() {
            SchemaSet::from_workspace_with_builtins(&schema_dir, builtins)
        } else {
            SchemaSet::from_schemas(builtins)
        }
        .map_err(|error| EmbedError::InvalidArgument(error.to_string()))?;

        let mut graph_inputs = graph
            .map(|graph| self.load_graph(graph, &config, &schema_dir))
            .transpose()?;
        for publish in &config.publish {
            if schemas.layout(&publish.schema).is_none() {
                return Err(EmbedError::InvalidArgument(format!(
                    "unknown publish schema '{}'",
                    publish.schema
                )));
            }
        }

        let graph_topics = graph_inputs
            .as_ref()
            .map(|inputs| output_topics(&inputs.config))
            .unwrap_or_default();
        let publish_topics: HashSet<&str> = config
            .publish
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

        let extra = config
            .publish
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

        let mut resources = Resources {
            manager: Some(Arc::clone(&manager)),
            ..Resources::default()
        };
        let result = (|| {
            self.create_publishers(&config.publish, &schemas, &manager, &mut resources)?;

            let mut graph_runtime = if let Some(inputs) = graph_inputs.as_mut() {
                let clock: Arc<dyn Clock> = Arc::new(RealClock);
                let schema_hashes = schema_hash_map(&schemas);
                let mut graph_runtime = GraphRuntime::build_live_with_schema_hashes_and_policy(
                    inputs.config.clone(),
                    std::mem::take(&mut inputs.factories),
                    &manager,
                    Arc::clone(&clock),
                    Some(&schema_hashes),
                    cerulion_cli_engine::graph_cmd::embedded_live_monitor_wait_policy(),
                )
                .map_err(|error| EmbedError::Graph(error.to_string()))?;
                graph_runtime.set_live_transport(Arc::clone(&manager));
                Some(graph_runtime)
            } else {
                None
            };

            if !config.listen.is_empty() {
                let mut announce = graph_topics;
                for publish in &config.publish {
                    if !announce.contains(&publish.topic) {
                        announce.push(publish.topic.clone());
                    }
                }
                let plan = GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce,
                    ingress: Vec::new(),
                };
                let gateway =
                    GatewayRuntime::new_with_schema_serving(Arc::clone(&manager), plan, serving)
                        .map_err(|error| EmbedError::Network(error.to_string()))?;
                let exit = Arc::new(AtomicBool::new(false));
                let thread_exit = Arc::clone(&exit);
                let status = Arc::clone(&self.status);
                resources.gateway_thread = Some(
                    thread::Builder::new()
                        .name("cerulion-gateway".to_string())
                        .spawn(move || {
                            let mut gateway = gateway;
                            let result =
                                gateway.run(&thread_exit).map_err(|error| error.to_string());
                            if let Err(error) = &result {
                                set_thread_error(&status, codes::NETWORK, error);
                            }
                            result
                        })
                        .map_err(|error| EmbedError::Network(error.to_string()))?,
                );
                resources.gateway_exit = Some(exit);
            }

            let bagd_plan = self.prepare_recorder(&config, graph_inputs.as_ref(), &schemas)?;
            if let Some((bagd_config, ready_file, record_path)) = bagd_plan {
                let shutdown = Arc::new(AtomicBool::new(false));
                let thread_shutdown = Arc::clone(&shutdown);
                let recorder_manager = Arc::clone(&manager);
                let status = Arc::clone(&self.status);
                resources.recorder_thread = Some(
                    thread::Builder::new()
                        .name("cerulion-bagd".to_string())
                        .spawn(move || {
                            let result = run_bagd(recorder_manager, bagd_config, thread_shutdown)
                                .map_err(|error| error.to_string());
                            if let Err(error) = &result {
                                set_thread_error(&status, codes::RECORDER, error);
                            }
                            result
                        })
                        .map_err(|error| EmbedError::Recorder(error.to_string()))?,
                );
                resources.recorder_shutdown = Some(shutdown);
                lock_status(&self.status).record_path =
                    Some(record_path.to_string_lossy().into_owned());
                self.wait_recorder_ready(&ready_file, &mut resources)?;
            }

            if let Some(mut graph_runtime) = graph_runtime.take() {
                let running = Arc::new(AtomicBool::new(true));
                let thread_running = Arc::clone(&running);
                let status = Arc::clone(&self.status);
                resources.graph_thread = Some(
                    thread::Builder::new()
                        .name("cerulion-graph".to_string())
                        .spawn(move || {
                            let result = graph_runtime
                                .run_live(&thread_running)
                                .map_err(|error| error.to_string());
                            if let Err(error) = &result {
                                set_thread_error(&status, codes::GRAPH, error);
                            }
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
                if status.state != "failed" {
                    status.state = "running".to_string();
                    status.last_error = None;
                }
            }
            Ok(())
        })();
        if let Err(error) = result {
            let _ = stop_resource_set(&mut resources, &self.status, true);
            return Err(error);
        }
        self.resources = resources;
        Ok(())
    }

    fn load_graph(
        &self,
        config: GraphConfig,
        embedded: &EmbeddedConfig,
        _schemas_dir: &Path,
    ) -> Result<GraphInputs, EmbedError> {
        let native_nodes = config.native_nodes().count();
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
        let endpoints = PROCESS_LISTEN_ENDPOINTS.get_or_init(|| Mutex::new(None));
        let mut endpoints = endpoints.lock().unwrap_or_else(|error| error.into_inner());
        if let Some(existing) = endpoints.as_ref() {
            if existing != listen {
                return Err(EmbedError::BadState(
                    "network listen endpoints are fixed for the process lifetime".to_string(),
                ));
            }
            return TransportManager::get().map_err(|error| EmbedError::Network(error.to_string()));
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
        *endpoints = Some(listen.to_vec());
        Ok(manager)
    }

    fn create_publishers(
        &self,
        config: &[PublishConfig],
        schemas: &SchemaSet,
        manager: &TransportManager,
        resources: &mut Resources,
    ) -> Result<(), EmbedError> {
        for item in config {
            let layout = schemas.layout(&item.schema).ok_or_else(|| {
                EmbedError::InvalidArgument(format!("unknown publish schema '{}'", item.schema))
            })?;
            let schema_hash = schemas.schema_hash(&item.schema).ok_or_else(|| {
                EmbedError::InvalidArgument(format!("unknown publish schema '{}'", item.schema))
            })?;
            let slot_len = WireHeader::SIZE
                .checked_add(item.max_payload_bytes)
                .and_then(|length| u32::try_from(length).ok())
                .and_then(MaxSliceLen::try_new)
                .ok_or_else(|| {
                    EmbedError::InvalidArgument(
                        "publisher slot size exceeds the wire transport limit".to_string(),
                    )
                })?;
            let publisher = manager
                .create_publisher(&item.topic, slot_len, 0)
                .map_err(|error| EmbedError::Publish(error.to_string()))?;
            resources.publishers.insert(
                item.topic.clone(),
                PublisherState {
                    publisher,
                    layout: layout.clone(),
                    schema_hash,
                    max_payload_bytes: item.max_payload_bytes,
                    sequence: 0,
                },
            );
        }
        Ok(())
    }

    fn prepare_recorder(
        &self,
        embedded: &EmbeddedConfig,
        graph: Option<&GraphInputs>,
        schemas: &SchemaSet,
    ) -> Result<Option<(BagdConfig, PathBuf, PathBuf)>, EmbedError> {
        let Some(record) = &embedded.record else {
            return Ok(None);
        };
        let bags_dir = self.files_dir.join("bags");
        std::fs::create_dir_all(&bags_dir)
            .map_err(|error| EmbedError::Recorder(error.to_string()))?;
        let record_path = bags_dir.join(&record.file);
        let ready_file = bags_dir.join(format!("{}.ready", record.file));
        let _ = std::fs::remove_file(&ready_file);
        let mut recorded = Vec::new();
        if let Some(graph) = graph {
            #[cfg(unix)]
            {
                recorded = resolve_recorded_topics(&self.files_dir, &graph.config, &graph.infos)
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
        for publish in &embedded.publish {
            let layout = schemas.layout(&publish.schema).ok_or_else(|| {
                EmbedError::InvalidArgument(format!("unknown publish schema '{}'", publish.schema))
            })?;
            let schema_hash = schemas.schema_hash(&publish.schema).ok_or_else(|| {
                EmbedError::InvalidArgument(format!("unknown publish schema '{}'", publish.schema))
            })?;
            let wire_fixed_size = u32::try_from(layout.fixed_size).map_err(|_| {
                EmbedError::InvalidArgument(format!(
                    "schema '{}' fixed wire size exceeds u32",
                    publish.schema
                ))
            })?;
            recorded.push(RecordedTopic {
                format: RECORDED_TOPIC_FORMAT,
                topic: publish.topic.clone(),
                schema_name: publish.schema.clone(),
                schema_hash,
                wire_fixed_size,
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
        config.ready_file = Some(ready_file.clone());
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

    fn stop_resources(
        &mut self,
        starting_failure: bool,
    ) -> Result<Option<StopSummary>, EmbedError> {
        stop_resource_set(&mut self.resources, &self.status, starting_failure)
    }
}

fn graph_inputs_name(graph: &Option<GraphInputs>) -> Option<String> {
    graph
        .as_ref()
        .map(|inputs| inputs.config.identity().to_string())
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

fn stop_resource_set(
    resources: &mut Resources,
    status: &Arc<Mutex<Status>>,
    starting_failure: bool,
) -> Result<Option<StopSummary>, EmbedError> {
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
    if !starting_failure {
        summary = stop_recorder(resources, &mut first_error);
    }
    resources.publishers.clear();
    resources.manager = None;
    if let Some(error) = first_error {
        set_error(status, &error);
        if starting_failure {
            return Ok(summary);
        }
        return Err(error);
    }
    Ok(summary)
}

fn stop_recorder(
    resources: &mut Resources,
    first_error: &mut Option<EmbedError>,
) -> Option<StopSummary> {
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
            Some(StopSummary {
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

fn set_state(status: &Arc<Mutex<Status>>, state: &str) {
    lock_status(status).state = state.to_string();
}

fn set_error(status: &Arc<Mutex<Status>>, error: &EmbedError) {
    let mut status = lock_status(status);
    status.state = "failed".to_string();
    status.last_error = Some(LastError {
        code: error.code(),
        message: error.to_string(),
    });
}

fn set_thread_error(status: &Arc<Mutex<Status>>, code: i32, message: &str) {
    let mut status = lock_status(status);
    status.state = "failed".to_string();
    status.last_error = Some(LastError {
        code,
        message: message.to_string(),
    });
}

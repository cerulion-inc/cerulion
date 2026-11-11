// SPDX-License-Identifier: AGPL-3.0-only
//! Checked native node lifecycle with host-owned transport and leased frames.
use super::node::{
    ExternalSource, InputConsume, NodeContext, NodeEntry, NodeInfo, PublisherReconStat,
    TriggerProbe,
};
use super::node_host_abi::{self as host, HostContext, HostPortSpec, HOST_API};
use crate::error::{NodeErrorKind, TransportError, TransportResult};
use crate::graph::params::ParamValues;
use crate::scheduler::sync_stamp::{HeaderStampLocation, StampDecodeError, SyncFrameSlot};
use crate::scheduler::{SyncHeadOp, SyncOpAnswer};
use crate::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use crate::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
use cerulion_node_abi as abi;
use std::path::Path;

// Preserve the native log byte through a verified compile-time constant.
const NATIVE_LOG_DASH: &str = "\u{2014}";

#[derive(serde::Deserialize)]
struct PortFact {
    name: String,
    schema_hash: u64,
    wire_fixed_size: u32,
    variable_count: u32,
    alignment: u32,
    #[serde(default)]
    accumulate_cap: Option<usize>,
}
#[derive(serde::Deserialize)]
struct PortFacts {
    inputs: Vec<PortFact>,
    outputs: Vec<PortFact>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Init,
    Tick,
    Shutdown,
    Destroy,
    Shape,
    External,
}
impl Phase {
    fn name(self) -> &'static str {
        match self {
            Self::Init => "init",
            Self::Tick => "tick",
            Self::Shutdown => "shutdown",
            Self::Destroy => "destroy",
            Self::Shape => "state shape",
            Self::External => "external source",
        }
    }
}

/// Production native node, discovered through the compiler-independent table.
pub struct DylibNodeEntry {
    library: Option<abi::CheckedNodeLibrary>,
    instance: Option<abi::NodeHandle>,
    context: Option<u64>,
    info: NodeInfo,
    metadata: String,
    ports: Option<Vec<HostPortSpec>>,
    params: String,
    label: String,
    snapshot_configured: bool,
    snapshot_failed: bool,
    retain_process: bool,
    doorbell: bool,
    shape: Option<u64>,
    inline_type: bool,
    maintenance: FailureRegimeLatch,
    snapshot_errors: FailureRegimeLatch,
    head_errors: FailureRegimeLatch,
    probe_errors: Vec<(String, FailureRegimeLatch)>,
    unknown_probe_errors: FailureRegimeLatch,
    external_errors: FailureRegimeLatch,
}

impl DylibNodeEntry {
    /// Discover a trusted native node and reject incompatible interfaces before init.
    /// # Errors
    /// Returns a load, metadata, capability or boundary refusal with a rebuild remedy.
    // hot-path-alloc-ok-fn: library discovery and graph construction only
    pub fn load(path: &Path) -> TransportResult<Self> {
        let label = path.display().to_string();
        let error = |reason| TransportError::NodeError {
            node_id: label.clone(),
            reason,
        };
        if cfg!(feature = "local-transport") {
            return Err(error("native cdylib nodes are not supported under `local-transport`; run a cross-process host".to_owned()));
        }
        super::node::cdylib_platform_check().map_err(error)?;
        // SAFETY: loading a user-selected native node authorizes its native code.
        // The stable export protocol requires immutable readable descriptors;
        // the SDK checks every header/callback before any instance is created.
        let library = unsafe { abi::CheckedNodeLibrary::load(path, HOST_API.capabilities) }
            .map_err(|e| {
                error(match e {
                    abi::AbiError::Platform | abi::AbiError::Library { .. } => e.to_string(),
                    _ => format!("{e}; rebuild the node (`cerulion node build <type>`)"),
                })
            })?;
        let span = library.api().metadata;
        // SAFETY: descriptor validation bounded this nonempty span; the trusted
        // node retains initialized immutable metadata while library stays owned.
        let bytes = unsafe { std::slice::from_raw_parts(span.ptr, span.len as usize) };
        let metadata = std::str::from_utf8(bytes)
            .map_err(|e| error(format!("node metadata is not UTF-8: {e}")))?
            .to_owned();
        let mut author_metadata: serde_json::Value = serde_json::from_str(&metadata)
            .map_err(|parse| {
                tracing::error!(node_id = %label, error = %parse, "node info returned unparseable JSON");
                TransportError::NodeInfoParse {
                    diag_label: label.clone(), json_len: metadata.len(),
                    prefix: metadata.chars().take(80).collect(), reason: parse.to_string(),
                }
            })?;
        let retain_process = metadata_bool(&author_metadata, "retain_process").map_err(error)?;
        let inline_type = metadata_bool(&author_metadata, "inline_safe").map_err(error)?;
        author_metadata
            .as_object_mut()
            .ok_or_else(|| error("node metadata must be an object".to_owned()))?
            .remove("retain_process");
        author_metadata
            .as_object_mut()
            .expect("metadata object validated")
            .remove("inline_safe");
        for (direction, keys) in [
            (
                "inputs",
                &[
                    "wire_fixed_size",
                    "variable_count",
                    "alignment",
                    "accumulate_cap",
                ][..],
            ),
            ("outputs", &["variable_count", "alignment"][..]),
        ] {
            if let Some(ports) = author_metadata[direction].as_array_mut() {
                for port in ports {
                    if let Some(port) = port.as_object_mut() {
                        for key in keys {
                            port.remove(*key);
                        }
                    }
                }
            }
        }
        let author_metadata = serde_json::to_string(&author_metadata)
            .map_err(|e| error(format!("node metadata could not be decoded: {e}")))?;
        let info = Self::parse_info_json_labeled(&author_metadata, &label).map_err(|reason| {
            TransportError::NodeInfoParse {
                diag_label: label.clone(),
                json_len: metadata.len(),
                prefix: metadata.chars().take(80).collect(),
                reason,
            }
        })?;
        let facts: PortFacts = serde_json::from_str(&metadata)
            .map_err(|e| error(format!("stable node port descriptors are invalid: {e}")))?;
        if facts.inputs.len() != info.input_names().len()
            || facts.outputs.len() != info.output_names().len()
        {
            return Err(error(
                "stable port descriptors do not match node metadata".to_owned(),
            ));
        }
        let mut ports = Vec::with_capacity(facts.inputs.len() + facts.outputs.len());
        for (index, fact) in facts.inputs.into_iter().enumerate() {
            let input = &info.input_meta()[index];
            if fact.name != input.name || fact.schema_hash != input.schema_hash {
                return Err(error(
                    "stable input descriptor order/schema differs from metadata".to_owned(),
                ));
            }
            let read_mode = if !input.trigger {
                abi::read_mode::HELD
            } else if input.consume == InputConsume::Latest {
                abi::read_mode::LATEST
            } else {
                abi::read_mode::ONE
            };
            ports.push(port_spec(fact, LeaseDirection::Read, read_mode).map_err(error)?);
        }
        for (index, fact) in facts.outputs.into_iter().enumerate() {
            if fact.name != info.output_meta()[index].name || fact.accumulate_cap.is_some() {
                return Err(error(
                    "stable output descriptor order differs from metadata".to_owned(),
                ));
            }
            ports.push(port_spec(fact, LeaseDirection::Write, abi::read_mode::ONE).map_err(error)?);
        }
        // One preallocated row per declared input keeps probe regimes independent
        // without allocating during publish, receive or scheduler maintenance.
        let probe_errors = info
            .input_meta()
            .iter()
            .map(|port| (port.name.clone(), FailureRegimeLatch::new()))
            .collect();
        let mut entry = Self {
            library: Some(library),
            instance: None,
            context: None,
            info,
            metadata: author_metadata,
            ports: Some(ports),
            params: "{}".to_owned(),
            label,
            snapshot_configured: false,
            snapshot_failed: false,
            retain_process,
            doorbell: false,
            shape: None,
            inline_type,
            maintenance: FailureRegimeLatch::new(),
            snapshot_errors: FailureRegimeLatch::new(),
            head_errors: FailureRegimeLatch::new(),
            probe_errors,
            unknown_probe_errors: FailureRegimeLatch::new(),
            external_errors: FailureRegimeLatch::new(),
        };
        entry.shape = entry.query_state_shape();
        Ok(entry)
    }
    fn api(&self) -> &abi::NodeApiV1 {
        self.library
            .as_ref()
            .expect("library retained for entry lifetime")
            .api()
    }
    /// Test support for exercising real host admission refusals.
    #[doc(hidden)]
    pub fn __test_host_context_handle(&self) -> Option<u64> {
        // Tests hold this context's actual lock; no foreign pointer is exposed.
        self.context
    }
    // hot-path-alloc-ok-fn: invalid lifecycle admission allocates only on refusal
    fn handle(&self) -> TransportResult<abi::NodeHandle> {
        self.instance
            .ok_or_else(|| self.error("node has not been initialized".to_owned()))
    }
    // hot-path-alloc-ok-fn: returned diagnostics copy the label only after failure
    fn error(&self, reason: String) -> TransportError {
        TransportError::NodeError {
            node_id: self.label.clone(),
            reason,
        }
    }
    // hot-path-alloc-ok-fn: diagnostic copies occur only after a boundary failure
    fn result(&self, status: abi::Status, phase: Phase) -> TransportResult<()> {
        if status.code == abi::status_code::OK {
            return Ok(());
        }
        let mut reason = self.diagnostic(status, phase.name());
        if phase == Phase::Init {
            reason = format!("init failed: {reason}");
        }
        if status.code == abi::status_code::PANICKED {
            return Err(if phase == Phase::Tick {
                TransportError::NodeTickPanicked {
                    node_id: self.label.clone(),
                    reason,
                }
            } else {
                self.error(reason)
            });
        }
        if let Some(kind) = NodeErrorKind::from_code(status.kind)
            .filter(|_| (abi::error_kind::LOGIC..=abi::error_kind::IO).contains(&status.kind))
        {
            return Err(TransportError::NodeFailed {
                node_id: self.label.clone(),
                kind,
                reason,
            });
        }
        Err(self.error(reason))
    }
    // hot-path-alloc-ok-fn: diagnostic text copies only after failure.
    fn diagnostic(&self, status: abi::Status, operation: &str) -> String {
        if status.error == 0 {
            return format!("node {operation} failed with status {}", status.code);
        }
        self.copy_node_diagnostic(status.error)
            .unwrap_or_else(|failure| format!("node {operation} diagnostic copy failed: {failure}"))
    }
    // hot-path-alloc-ok-fn: complete node diagnostics copy only on a failure, using host-owned storage.
    fn copy_node_diagnostic(&self, token: u64) -> Result<String, String> {
        let mut required = 0u64;
        // SAFETY: checked live callback borrows an exclusive length scalar;
        // a null zero-length buffer is the frozen size query and empty-copy case.
        let queried = self
            .invoke(|| unsafe {
                (self.api().copy_error)(
                    token,
                    abi::MutableBytes {
                        ptr: std::ptr::null_mut(),
                        len: 0,
                    },
                    &mut required,
                )
            })
            .map_err(|error| error.to_string())?;
        if queried.code == abi::status_code::OK && required == 0 {
            return Ok(String::new());
        }
        if queried.code != abi::status_code::BUFFER_TOO_SMALL || required == 0 {
            return Err(format!(
                "size query returned status {} and length {required}",
                queried.code
            ));
        }
        let length = usize::try_from(required)
            .ok()
            .filter(|length| *length <= isize::MAX as usize)
            .ok_or_else(|| format!("length {required} exceeds addressable host storage"))?;
        let mut bytes = Vec::new();
        bytes.try_reserve_exact(length).map_err(|error| {
            format!("cannot allocate complete {length}-byte diagnostic: {error}")
        })?;
        bytes.resize(length, 0);
        // SAFETY: the host owns exclusive initialized storage of exactly the
        // requested length. The node copies its bytes without retaining/freeing it.
        let copied = self
            .invoke(|| unsafe {
                (self.api().copy_error)(
                    token,
                    abi::MutableBytes {
                        ptr: bytes.as_mut_ptr(),
                        len: length as u64,
                    },
                    &mut required,
                )
            })
            .map_err(|error| error.to_string())?;
        if copied.code != abi::status_code::OK || required != length as u64 {
            return Err(format!(
                "complete copy returned status {} and length {required}; expected {length}",
                copied.code
            ));
        }
        String::from_utf8(bytes).map_err(|error| format!("diagnostic is not UTF-8: {error}"))
    }

    // hot-path-alloc-ok-fn: admission diagnostics allocate only on contract refusal.
    fn invoke<R>(&self, call: impl FnOnce() -> R) -> TransportResult<R> {
        let Some(context) = self.context else {
            // Library-only shape discovery before create has no host context.
            return Ok(call());
        };
        host::with_context(context, HostContext::begin_node_call)
            .map_err(|error| self.error(format!("node call admission failed: {error:?}")))?;
        // The context admission gate is released before foreign execution;
        // synchronous HostApi callbacks may re-enter it on this thread.
        let result = call();
        host::with_context(context, HostContext::end_node_call)
            .map_err(|error| self.error(format!("node call completion failed: {error:?}")))?;
        Ok(result)
    }

    fn query_state_shape(&self) -> Option<u64> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return None;
        }
        let mut shape = 0;
        // SAFETY: SDK validated complete immutable state table retained by library;
        // exclusive stack output is live through synchronous, handle-free query.
        let status = self.invoke(|| unsafe { ((*ptr).shape)(&mut shape) }).ok()?;
        match self.result(status, Phase::Shape) {
            Ok(()) => Some(shape),
            Err(error) => {
                tracing::warn!(node_id = %self.label, error = %error, "cerulion_node_state_shape failed; node state is unavailable");
                None
            }
        }
    }
    /// Borrow the already validated metadata JSON.
    // hot-path-alloc-ok-fn: metadata inspection is cold CLI graph discovery
    pub fn info_json(&self) -> Option<String> {
        Some(self.metadata.clone())
    }
    /// Return the checked metadata JSON.
    /// # Errors
    /// Discovery already checked this metadata; no callback is invoked here.
    // hot-path-alloc-ok-fn: metadata inspection is cold CLI graph discovery
    pub fn checked_info_json(&self) -> TransportResult<String> {
        Ok(self.metadata.clone())
    }
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn leaks_library_on_drop(&self) -> bool {
        self.retain_process
    }
}

// hot-path-alloc-ok-fn: metadata validation only runs during cold library loading.
fn metadata_bool(metadata: &serde_json::Value, key: &str) -> Result<bool, String> {
    match metadata.get(key) {
        None => Ok(false),
        Some(value) => value
            .as_bool()
            .ok_or_else(|| format!("{key} metadata must be a bool")),
    }
}

// hot-path-alloc-ok-fn: port validation happens during cold library discovery
fn port_spec(
    fact: PortFact,
    direction: LeaseDirection,
    read_mode: u32,
) -> Result<HostPortSpec, String> {
    let layout = PluginWireLayout {
        schema_hash: fact.schema_hash,
        fixed_size: fact.wire_fixed_size,
        variable_fields: fact.variable_count,
        alignment: fact.alignment,
    };
    if layout.prefix_len().is_none() || fact.accumulate_cap == Some(0) {
        return Err("stable port layout or accumulate capacity is invalid".to_owned());
    }
    Ok(HostPortSpec {
        name: fact.name,
        direction,
        layout,
        read_mode,
        accumulate_cap: fact.accumulate_cap,
    })
}
// Native state diagnostic codes remain stable even though transport statuses use
// their own frozen namespace. They describe the same encoder/decoder failures.
fn state_failure_code(status: abi::Status) -> i32 {
    match status.code {
        abi::status_code::BUFFER_TOO_SMALL => -6,
        abi::status_code::PANICKED => -4,
        abi::status_code::INVALID_HANDLE => -2,
        abi::status_code::INVALID_ARGUMENT => -3,
        _ => -5,
    }
}
fn observe(label: &str, site: &str, latch: &mut FailureRegimeLatch, failed: bool) {
    if failed {
        match latch.on_failure() {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                tracing::error!(node_id = %label, service = site, total_failures = latch.total_failures(), "native node host service failed")
            }
            RegimeDecision::Suppressed { .. } => {
                tracing::debug!(node_id = %label, service = site, total_failures = latch.total_failures(), "native node host service failure continues")
            }
        }
    } else if let Some(suppressed) = latch.on_success() {
        tracing::info!(node_id = %label, service = site, suppressed, "native node host service recovered");
    }
}

impl NodeEntry for DylibNodeEntry {
    // hot-path-alloc-ok-fn: metadata is cloned only at graph construction
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(self.info.clone())
    }
    // hot-path-alloc-ok-fn: parameter serialization occurs only before initialization
    fn set_params(&mut self, params: &ParamValues) -> TransportResult<()> {
        if self.info.params().is_empty() && !params.is_empty() {
            return Err(self.error(format!(
                "node declares no params; supplied {}",
                params
                    .keys()
                    .map(String::as_str)
                    .collect::<Vec<_>>()
                    .join(", ")
            )));
        }
        self.params = crate::graph::params::params_to_json(params)
            .map_err(|e| self.error(format!("params could not be encoded: {e}")))?;
        Ok(())
    }
    // hot-path-alloc-ok-fn: context registration and bounded arena allocation happen once at init
    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        if self.instance.is_some() || self.context.is_some() {
            return Err(self.error("already initialized (double init)".to_owned()));
        }
        let ports = self
            .ports
            .take()
            .ok_or_else(|| self.error("node initialization already attempted".to_owned()))?;
        let budget = ports
            .iter()
            .fold(
                ports.len().checked_mul(4).and_then(|n| n.checked_add(1)),
                |total, port| {
                    total.and_then(|n| {
                        port.accumulate_cap
                            .unwrap_or(0)
                            .checked_mul(2)
                            .and_then(|cap| n.checked_add(cap))
                    })
                },
            )
            .ok_or_else(|| self.error("node lease budget overflows".to_owned()))?;
        let environment = context.environment_for_plugin();
        let native = HostContext::new(context, ports, environment, budget)
            .map_err(|e| self.error(format!("host context initialization failed: {e:?}")))?;
        let handle = host::register_context(native)
            .map_err(|e| self.error(format!("host context registration failed: {e:?}")))?;
        self.context = Some(handle);
        let mut instance = abi::NodeHandle::default();
        // SAFETY: checked code/table retained, process-lifetime HOST_API; host
        // context remains registered. Params and exclusive output last this call.
        // No host context lock is held while plugin callbacks re-enter services.
        let status = self.invoke(|| unsafe {
            (self.api().create)(
                &HOST_API,
                abi::ContextHandle { value: handle },
                abi::Bytes {
                    ptr: self.params.as_ptr(),
                    len: self.params.len() as u64,
                },
                &mut instance,
            )
        });
        let status = match status {
            Ok(status) => status,
            Err(error) => {
                // A refused completion may follow successful create. Retain code
                // and host mappings until safe teardown can settle that instance.
                self.retain_process = true;
                self.instance = (instance.value != 0).then_some(instance);
                return Err(error);
            }
        };
        if let Err(error) = self.result(status, Phase::Init) {
            if !self.retain_process {
                match host::remove_context(handle) {
                    Ok(()) => self.context = None,
                    Err(error) => {
                        self.retain_process = true;
                        tracing::warn!(node_id = %self.label, error = ?error, "failed node creation retained its library and host context");
                    }
                }
            }
            return Err(error);
        }
        if instance.value == 0 {
            if !self.retain_process {
                match host::remove_context(handle) {
                    Ok(()) => self.context = None,
                    Err(error) => {
                        self.retain_process = true;
                        tracing::warn!(node_id = %self.label, error = ?error, "failed node creation retained its library and host context");
                    }
                }
            }
            return Err(self.error("node init returned an invalid zero handle".to_owned()));
        }
        self.instance = Some(instance);
        Ok(())
    }
    // hot-path-alloc-ok-fn: fire admission diagnostics allocate only after rejection
    fn tick(&mut self) -> TransportResult<()> {
        let instance = self.handle()?;
        let result = self.invoke(|| {
            host::with_context(self.context.unwrap_or(0), HostContext::begin_fire)
                .map_err(|error| self.error(format!("node fire admission failed: {error:?}")))?;
            // SAFETY: exclusive admitted entry, retained instance/code/context;
            // each outstanding lease keeps its mapping until its own release.
            Ok::<abi::Status, TransportError>(unsafe { (self.api().tick)(instance) })
        })??;
        self.result(result, Phase::Tick)
    }
    // hot-path-alloc-ok-fn: node teardown diagnostics are outside publish and receive
    fn shutdown(&mut self) -> TransportResult<()> {
        let Some(instance) = self.instance.take() else {
            return Ok(());
        };
        // SAFETY: exclusive lifecycle call while context/services/code remain
        // live; shutdown runs before node allocator destroys its own state.
        let result = self
            .invoke(|| unsafe { (self.api().shutdown)(instance) })
            .and_then(|status| self.result(status, Phase::Shutdown));
        // SAFETY: all synchronous node calls have returned; node frees only its
        // own instance allocation, never the host context or sample owners.
        let destroyed = match self.invoke(|| unsafe { (self.api().destroy)(instance) }) {
            Ok(status) => status,
            Err(error) => {
                self.retain_process = true;
                return Err(error);
            }
        };
        if destroyed.code != abi::status_code::OK {
            self.retain_process = true;
            return self.result(destroyed, Phase::Destroy);
        }
        if !self.retain_process {
            if let Some(context) = self.context {
                match host::remove_context(context) {
                    Ok(()) => self.context = None,
                    Err(error) => {
                        // An outstanding owner may still call its finalizer. Keep
                        // code, callbacks and mappings live rather than unloading.
                        self.retain_process = true;
                        return Err(self.error(format!("host context teardown failed: {error:?}")));
                    }
                }
            }
        }
        result
    }
    fn pump_history(&mut self) {
        if self.context.is_none() {
            return;
        }
        let failed =
            host::with_context(self.context.unwrap_or(0), HostContext::runtime_pump_history)
                .is_err();
        observe(&self.label, "history", &mut self.maintenance, failed);
    }
    fn pump_history_at(&mut self, now: std::time::Instant) {
        if self.context.is_none() {
            return;
        }
        let failed = host::with_context(self.context.unwrap_or(0), |ctx| {
            ctx.runtime_pump_history_at(now)
        })
        .is_err();
        observe(&self.label, "history", &mut self.maintenance, failed);
    }
    fn collect_publisher_recon_stats(&self, out: &mut Vec<PublisherReconStat>) {
        if let Some(context) = self.context {
            let _ = host::with_context(context, |ctx| {
                ctx.native_for_runtime().collect_publisher_recon_stats(out);
                Ok(())
            });
        }
    }
    fn snapshot_inputs(&mut self, inputs: &[String]) {
        if self.snapshot_failed || self.context.is_none() {
            return;
        }
        let context = self.context.unwrap_or(0);
        if !self.snapshot_configured {
            let configured = serde_json::to_vec(inputs).ok().is_some_and(|bytes| {
                host::with_context(context, |ctx| ctx.configure_snapshots(&bytes)).is_ok()
            });
            self.snapshot_configured = configured;
            self.snapshot_failed = !configured;
            if !configured {
                observe(
                    &self.label,
                    "snapshot registration",
                    &mut self.snapshot_errors,
                    true,
                );
                return;
            }
        }
        let failed = host::with_context(context, HostContext::snapshot_configured).is_err();
        observe(&self.label, "snapshot", &mut self.snapshot_errors, failed);
    }
    fn holds_input_snapshot(&self) -> bool {
        !self.snapshot_failed
    }
    fn unifies_trigger_drain(&self) -> bool {
        true
    }
    fn refills_trigger_input(&self) -> bool {
        true
    }
    fn drain_trigger_input(&mut self, name: &str) -> (u64, Option<u64>) {
        let result = host::with_context(self.context.unwrap_or(0), |ctx| {
            ctx.runtime_drain_trigger_input(name)
        });
        observe(&self.label, "drain", &mut self.head_errors, result.is_err());
        result.unwrap_or((0, None))
    }
    fn refill_trigger_input(&mut self, name: &str) -> (u64, Option<u64>) {
        let result = host::with_context(self.context.unwrap_or(0), |ctx| {
            ctx.runtime_refill_trigger_input(name)
        });
        observe(
            &self.label,
            "refill",
            &mut self.head_errors,
            result.is_err(),
        );
        result.unwrap_or((0, None))
    }
    fn sync_head_op(&mut self, name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        let result = host::with_context(self.context.unwrap_or(0), |ctx| {
            ctx.runtime_sync_head_op(name, op)
        });
        let code = result
            .as_ref()
            .err()
            .map_or(abi::status_code::FAILED, |error| error.status);
        let answer = result.unwrap_or(SyncOpAnswer::Failed);
        if answer == SyncOpAnswer::Failed {
            match self.head_errors.on_failure() {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                    tracing::error!(node = %self.label, input = %name, op = ?op, code, total_failures = self.head_errors.total_failures(), "cdylib sync_head_op FFI call failed; the per-set Sync alignment pass ends here {NATIVE_LOG_DASH} no set member fabricated (suppressing further to debug until it recovers)");
                }
                RegimeDecision::Suppressed { .. } => {
                    tracing::debug!(node = %self.label, input = %name, op = ?op, code, "cdylib sync_head_op still failing (suppressed)");
                }
            }
        } else if self.head_errors.on_success().is_some() {
            tracing::info!(node = %self.label, input = %name, "cdylib sync_head_op recovered");
        }
        answer
    }
    fn supports_sync_head_ops(&self) -> bool {
        true
    }
    fn probes_trigger_input(&self) -> bool {
        true
    }
    fn probe_trigger_input(&mut self, name: &str) -> TriggerProbe {
        let result = host::with_context(self.context.unwrap_or(0), |ctx| {
            ctx.runtime_sync_head_op(name, SyncHeadOp::ProbeNext)
        });
        let code = result
            .as_ref()
            .err()
            .map_or(abi::status_code::FAILED, |error| error.status);
        let answer = result.unwrap_or(SyncOpAnswer::Failed);
        let latch = self
            .probe_errors
            .iter_mut()
            .find(|(input, _)| input == name)
            .map_or(&mut self.unknown_probe_errors, |(_, latch)| latch);
        if !matches!(answer, SyncOpAnswer::Present | SyncOpAnswer::Nothing) {
            match latch.on_failure() {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                    tracing::error!(node = %self.label, input = %name, code, total_failures = latch.total_failures(), "cdylib consume = latest body probe got no answer (op code 0 of cerulion_node_sync_head_op failed or answered a kind this host does not know); this input fires on every arrival its trigger subscriber counts, including a frame an earlier read already took, until the probe answers (suppressing further to debug until it recovers)");
                }
                RegimeDecision::Suppressed { .. } => {
                    tracing::debug!(node = %self.label, input = %name, code, "cdylib consume = latest body probe still failing (suppressed)");
                }
            }
        } else if latch.on_success().is_some() {
            tracing::info!(node = %self.label, input = %name, "cdylib consume = latest body probe recovered");
        }
        match answer {
            SyncOpAnswer::Present => TriggerProbe::Unread,
            SyncOpAnswer::Nothing => TriggerProbe::Empty,
            _ => TriggerProbe::Unknown,
        }
    }
    fn supports_sync_capture_stamps(&self) -> bool {
        true
    }
    fn sync_capture_stamp(
        &mut self,
        name: &str,
        slot: SyncFrameSlot,
        location: HeaderStampLocation,
    ) -> Option<Result<u64, StampDecodeError>> {
        host::with_context(self.context.unwrap_or(0), |ctx| {
            Ok(ctx
                .native_for_runtime()
                .sync_capture_stamp(name, slot, location))
        })
        .ok()
        .flatten()
    }
    fn state_shape(&self) -> Option<u64> {
        self.shape
    }
    // hot-path-alloc-ok-fn: anchor restoration and its diagnostics are cold
    fn restore_state(&mut self, bytes: &[u8]) -> TransportResult<()> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return Err(TransportError::GraphError { reason: format!("node '{}' declares no restorable state; implement the stable state table, or use #[cerulion_node] to generate cerulion_node_restore_state behavior; refusing to resume from its constructor's defaults", self.label) });
        }
        // SAFETY: checked state table/code and instance retained, exclusive entry;
        // payload is host-owned and borrowed only through this synchronous call.
        let instance = self.instance.ok_or_else(|| TransportError::GraphError {
            reason: format!("node '{}' has not been initialized", self.label),
        })?;
        let status = self.invoke(|| unsafe {
            ((*ptr).restore)(
                instance,
                abi::Bytes {
                    ptr: bytes.as_ptr(),
                    len: bytes.len() as u64,
                },
            )
        })?;
        if status.code == abi::status_code::OK {
            return Ok(());
        }
        Err(super::node::state_restore_failure(
            &self.label,
            state_failure_code(status),
            &self.diagnostic(status, "restore state"),
        ))
    }
    // hot-path-alloc-ok-fn: state capture diagnostics allocate only on refusal
    fn capture_state(&self, sink: &mut dyn crate::state::StateSink) -> TransportResult<()> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return Err(TransportError::GraphError {
                reason: format!("node '{}' declares no capturable state", self.label),
            });
        }
        let capacity = sink.remaining_hint().map_or(u64::MAX, |n| n as u64);
        let mut bridge = SinkBridge {
            sink,
            full: false,
            panicked: false,
        };
        let output = abi::StateSinkV1 {
            context: (&mut bridge as *mut SinkBridge<'_>).cast(),
            write: sink_write,
        };
        // SAFETY: checked callback/instance remain live; bridge and record are
        // exclusive borrowed stack objects, never retained past synchronous call.
        let instance = self.instance.ok_or_else(|| TransportError::GraphError {
            reason: format!("node '{}' has not been initialized", self.label),
        })?;
        let status = self.invoke(|| unsafe { ((*ptr).capture)(instance, &output, capacity) })?;
        if status.code == abi::status_code::OK && !bridge.full && !bridge.panicked {
            return Ok(());
        }
        if bridge.full || (status.code == abi::status_code::BUFFER_TOO_SMALL && !bridge.panicked) {
            bridge.sink.refuse();
        }
        if status.code == abi::status_code::OK {
            return Err(TransportError::GraphError { reason: format!(
                "node '{}' state capture REFUSED: the host's sink {}; yet its exporter returned 0 (success). This exporter is BROKEN: it must stop at the first refusal and report BUFFER_TOO_SMALL (8). Only a PREFIX of this node's state reached the sink", self.label,
                if bridge.panicked { "PANICKED" } else { "REFUSED a chunk" }) });
        }
        let cause = if bridge.panicked {
            "the host sink panicked while accepting a chunk"
        } else if bridge.full || status.code == abi::status_code::BUFFER_TOO_SMALL {
            "the host sink refused the bytes (its buffer is too small for this node's state)"
        } else {
            "the cdylib could not encode this node's state"
        };
        Err(super::node::state_capture_failure(
            &self.label,
            state_failure_code(status),
            cause,
            &self.diagnostic(status, "capture state"),
        ))
    }
    fn inline_safe(&self) -> bool {
        let ptr = self.api().state;
        let Some(instance) = self.instance else {
            return self.inline_type;
        };
        if ptr.is_null() {
            return false;
        }
        let mut safe = 0;
        // SAFETY: checked library/state/instance live; stack scalar exclusively
        // borrowed for synchronous query, graph serializes instance calls.
        let Ok(status) = self.invoke(|| unsafe { ((*ptr).inline_safe)(instance, &mut safe) })
        else {
            return false;
        };
        status.code == abi::status_code::OK && safe == 1
    }
    fn cer_probe(&self) -> bool {
        let ptr = self.api().state;
        // A node declaring no state graph cannot veto another node's anchor.
        if ptr.is_null() {
            return true;
        }
        let Some(instance) = self.instance else {
            return false;
        };
        // SAFETY: checked state table and instance live; graph serializes the
        // synchronous probe with all other calls and retains library until return.
        self.invoke(|| unsafe { ((*ptr).probe)(instance) })
            .is_ok_and(|status| status.code == abi::status_code::OK)
    }
    // hot-path-alloc-ok-fn: live source discovery runs once before polling; only a refused descriptor allocates text.
    fn external_source(&mut self) -> Option<ExternalSource> {
        let ptr = self.api().external;
        let instance = self.instance?;
        if ptr.is_null() {
            return None;
        }
        let mut source = abi::ExternalSourceV1 {
            fd: -1,
            kind: 0,
            flags: 0,
        };
        // SAFETY: validated external table/code and instance live; exclusive stack
        // output lasts query. Graph asks once at live entry, never replay stepping.
        let status = self
            .invoke(|| unsafe { ((*ptr).describe)(instance, &mut source) })
            .ok()?;
        if source.flags & abi::external::RETAIN_PROCESS != 0 {
            self.retain_process = true;
        }
        let fd = i32::try_from(source.fd).ok().filter(|fd| *fd >= 0);
        let valid = status.code == abi::status_code::OK
            && match source.kind {
                abi::external::DEVICE_FD => source.flags == 0 && fd.is_some(),
                abi::external::DOORBELL_FD => {
                    source.flags == (abi::external::HOST_CLOSES | abi::external::RETAIN_PROCESS)
                        && fd.is_some()
                }
                abi::external::HOST_DRIVEN => source.flags == 0,
                _ => false,
            };
        if !valid {
            let error = self
                .result(status, Phase::External)
                .err()
                .unwrap_or_else(|| self.error("external source descriptor is invalid".to_owned()));
            match self.external_errors.on_failure() {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                    tracing::error!(node_id = %self.label, error = %error,
                    total_failures = self.external_errors.total_failures(), "node has no external source")
                }
                RegimeDecision::Suppressed { .. } => tracing::debug!(node_id = %self.label,
                    total_failures = self.external_errors.total_failures(), "external source failure continues"),
            }
        } else if let Some(suppressed) = self.external_errors.on_success() {
            tracing::info!(node_id = %self.label, suppressed, "external source recovered");
        }
        if !valid {
            return None;
        }
        if source.kind == abi::external::HOST_DRIVEN {
            return Some(ExternalSource::HostDriven);
        }
        if source.kind == abi::external::DOORBELL_FD {
            self.retain_process = true;
            self.doorbell = true;
        }
        fd.map(ExternalSource::Fd)
    }
    fn external_source_is_drained_doorbell_fd(&self) -> bool {
        self.doorbell
    }
}
impl Drop for DylibNodeEntry {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            tracing::error!(node_id = %self.label, error = %error, "native node teardown failed");
        }
        if self.retain_process {
            if let Some(library) = self.library.take() {
                tracing::debug!(node_id = %self.label, "leaking cdylib library handle for process-lifetime helpers");
                library.retain_for_process();
            }
        }
    }
}
struct SinkBridge<'a> {
    sink: &'a mut dyn crate::state::StateSink,
    full: bool,
    panicked: bool,
}
unsafe extern "C" fn sink_write(context: *mut u8, bytes: abi::Bytes) -> abi::Status {
    let failure = |code| abi::Status {
        error: 0,
        code,
        kind: 0,
    };
    if context.is_null()
        || !(context as usize).is_multiple_of(std::mem::align_of::<SinkBridge<'_>>())
    {
        return failure(abi::status_code::INVALID_ARGUMENT);
    }
    // SAFETY: capture passes its live exclusive stack bridge. Node callback
    // contract forbids retaining the context or concurrent writes to the sink.
    let bridge = unsafe { &mut *context.cast::<SinkBridge<'_>>() };
    let Ok(length) = usize::try_from(bytes.len) else {
        return failure(abi::status_code::INVALID_ARGUMENT);
    };
    if length > isize::MAX as usize || (length != 0 && bytes.ptr.is_null()) {
        return failure(abi::status_code::INVALID_ARGUMENT);
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // SAFETY: trusted node lends initialized readable bytes for this call;
        // null/length bound checked. Empty spans avoid null raw-slice creation.
        let chunk = if length == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(bytes.ptr, length) }
        };
        bridge.sink.write(chunk)
    }));
    match result {
        Ok(Ok(())) => abi::Status::default(),
        Ok(Err(_)) => {
            bridge.full = true;
            failure(abi::status_code::BUFFER_TOO_SMALL)
        }
        Err(_) => {
            bridge.panicked = true;
            failure(abi::status_code::PANICKED)
        }
    }
}

#[cfg(test)]
#[cfg(not(feature = "local-transport"))]
mod tests {
    use super::*;

    #[test]
    fn state_failure_codes_preserve_the_native_export_contract() {
        for (code, native) in [
            (abi::status_code::BUFFER_TOO_SMALL, -6),
            (abi::status_code::PANICKED, -4),
            (abi::status_code::INVALID_HANDLE, -2),
            (abi::status_code::INVALID_ARGUMENT, -3),
            (abi::status_code::FAILED, -5),
        ] {
            assert_eq!(
                state_failure_code(abi::Status {
                    code,
                    error: 0,
                    kind: 0,
                }),
                native
            );
        }
    }

    #[test]
    fn only_tick_panics_have_the_structural_tick_panic_class() {
        let entry = DylibNodeEntry::load(&crate::testing::find_fixture_cdylib("test_node_cdylib"))
            .expect("build test_node_cdylib first");
        for phase in [
            Phase::Init,
            Phase::Tick,
            Phase::Shutdown,
            Phase::Destroy,
            Phase::Shape,
            Phase::External,
        ] {
            let error = entry
                .result(
                    abi::Status {
                        code: abi::status_code::PANICKED,
                        kind: 0,
                        error: 0,
                    },
                    phase,
                )
                .expect_err("a panic cannot succeed");
            assert_eq!(
                matches!(error, TransportError::NodeTickPanicked { .. }),
                phase == Phase::Tick
            );
            if phase != Phase::Tick {
                assert!(matches!(error, TransportError::NodeError { .. }));
            }
            if phase == Phase::Init {
                assert!(error.to_string().contains("init failed: ")); // hot-path-alloc-ok: cold test-only assertion formats a diagnostic, never publish or receive.
            }
        }
    }
}

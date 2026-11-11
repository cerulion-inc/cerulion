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
    maintenance: FailureRegimeLatch,
    snapshot_errors: FailureRegimeLatch,
    head_errors: FailureRegimeLatch,
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
                error(format!(
                    "{e}; rebuild the node (`cerulion node build <type>`)"
                ))
            })?;
        let span = library.api().metadata;
        // SAFETY: descriptor validation bounded this nonempty span; the trusted
        // node retains initialized immutable metadata while library stays owned.
        let bytes = unsafe { std::slice::from_raw_parts(span.ptr, span.len as usize) };
        let metadata = std::str::from_utf8(bytes)
            .map_err(|e| error(format!("node metadata is not UTF-8: {e}")))?
            .to_owned();
        let mut author_metadata: serde_json::Value = serde_json::from_str(&metadata)
            .map_err(|e| error(format!("node metadata is invalid: {e}")))?;
        let retain_process = match author_metadata.get("retain_process") {
            None => false,
            Some(value) => value
                .as_bool()
                .ok_or_else(|| error("retain_process metadata must be a bool".to_owned()))?,
        };
        author_metadata
            .as_object_mut()
            .ok_or_else(|| error("node metadata must be an object".to_owned()))?
            .remove("retain_process");
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
        Ok(Self {
            library: Some(library),
            instance: None,
            context: None,
            info,
            metadata,
            ports: Some(ports),
            params: "{}".to_owned(),
            label,
            snapshot_configured: false,
            snapshot_failed: false,
            retain_process,
            doorbell: false,
            maintenance: FailureRegimeLatch::new(),
            snapshot_errors: FailureRegimeLatch::new(),
            head_errors: FailureRegimeLatch::new(),
            external_errors: FailureRegimeLatch::new(),
        })
    }
    fn api(&self) -> &abi::NodeApiV1 {
        self.library
            .as_ref()
            .expect("library retained for entry lifetime")
            .api()
    }
    // hot-path-alloc-ok-fn: invalid lifecycle admission allocates only on refusal
    fn handle(&self) -> TransportResult<abi::NodeHandle> {
        self.instance
            .ok_or_else(|| self.error("node not initialized".to_owned()))
    }
    // hot-path-alloc-ok-fn: returned diagnostics copy the label only after failure
    fn error(&self, reason: String) -> TransportError {
        TransportError::NodeError {
            node_id: self.label.clone(),
            reason,
        }
    }
    // hot-path-alloc-ok-fn: diagnostic copies occur only after a boundary failure
    fn result(&self, status: abi::Status, operation: &str) -> TransportResult<()> {
        if status.code == abi::status_code::OK {
            return Ok(());
        }
        let mut reason = format!("node {operation} failed with status {}", status.code);
        if status.error != 0 {
            let mut required = 0u64;
            // SAFETY: library retains callback, caller owns output scalar. Null
            // zero-length buffer is the frozen non-consuming size query.
            let queried = unsafe {
                (self.api().copy_error)(
                    status.error,
                    abi::MutableBytes {
                        ptr: std::ptr::null_mut(),
                        len: 0,
                    },
                    &mut required,
                )
            };
            if matches!(
                queried.code,
                abi::status_code::OK | abi::status_code::BUFFER_TOO_SMALL
            ) && required > 0
                && required <= 1024 * 1024
            {
                let mut bytes = vec![0u8; required as usize];
                // SAFETY: exclusive host buffer is initialized and remains live
                // through synchronous copy; node never retains or frees it.
                let copied = unsafe {
                    (self.api().copy_error)(
                        status.error,
                        abi::MutableBytes {
                            ptr: bytes.as_mut_ptr(),
                            len: bytes.len() as u64,
                        },
                        &mut required,
                    )
                };
                if copied.code == abi::status_code::OK && required as usize <= bytes.len() {
                    reason = String::from_utf8_lossy(&bytes[..required as usize]).into_owned();
                }
            }
        }
        if status.code == abi::status_code::PANICKED {
            return Err(TransportError::NodeTickPanicked {
                node_id: self.label.clone(),
                reason,
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
    fn native<R>(
        &self,
        op: impl FnOnce(&mut NodeContext) -> R,
    ) -> Result<R, host::HostCallFailure> {
        host::with_context(self.context.unwrap_or(0), |ctx| {
            Ok(op(ctx.native_mut_for_runtime()?))
        })
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
        let status = unsafe {
            (self.api().create)(
                &HOST_API,
                abi::ContextHandle { value: handle },
                abi::Bytes {
                    ptr: self.params.as_ptr(),
                    len: self.params.len() as u64,
                },
                &mut instance,
            )
        };
        if let Err(error) = self.result(status, "init") {
            if !self.retain_process {
                let _ = host::remove_context(handle);
                self.context = None;
            }
            return Err(error);
        }
        if instance.value == 0 {
            if !self.retain_process {
                let _ = host::remove_context(handle);
                self.context = None;
            }
            return Err(self.error("node init returned an invalid zero handle".to_owned()));
        }
        self.instance = Some(instance);
        Ok(())
    }
    // hot-path-alloc-ok-fn: fire admission diagnostics allocate only after rejection
    fn tick(&mut self) -> TransportResult<()> {
        let instance = self.handle()?;
        host::with_context(self.context.unwrap_or(0), HostContext::begin_fire)
            .map_err(|e| self.error(format!("node fire admission failed: {e:?}")))?;
        // SAFETY: graph holds exclusive NodeEntry access; node instance and host
        // context are live, all previous loans returned, checked library retained.
        let result = unsafe { (self.api().tick)(instance) };
        self.result(result, "tick")
    }
    // hot-path-alloc-ok-fn: node teardown diagnostics are outside publish and receive
    fn shutdown(&mut self) -> TransportResult<()> {
        let Some(instance) = self.instance.take() else {
            return Ok(());
        };
        // SAFETY: exclusive lifecycle call while context/services/code remain
        // live; shutdown runs before node allocator destroys its own state.
        let status = unsafe { (self.api().shutdown)(instance) };
        let result = self.result(status, "shutdown");
        // SAFETY: all synchronous node calls have returned; node frees only its
        // own instance allocation, never the host context or sample owners.
        let destroyed = unsafe { (self.api().destroy)(instance) };
        if destroyed.code != abi::status_code::OK {
            self.retain_process = true;
            return self.result(destroyed, "destroy");
        }
        if !self.retain_process {
            if let Some(context) = self.context.take() {
                host::remove_context(context)
                    .map_err(|e| self.error(format!("host context teardown failed: {e:?}")))?;
            }
        }
        result
    }
    fn pump_history(&mut self) {
        if self.context.is_none() {
            return;
        }
        let failed = self.native(NodeContext::pump_history).is_err();
        observe(&self.label, "history", &mut self.maintenance, failed);
    }
    fn pump_history_at(&mut self, now: std::time::Instant) {
        if self.context.is_none() {
            return;
        }
        let failed = self.native(|ctx| ctx.pump_history_at(now)).is_err();
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
        let result = self.native(|ctx| ctx.drain_trigger_input(name));
        observe(&self.label, "drain", &mut self.head_errors, result.is_err());
        result.unwrap_or((0, None))
    }
    fn refill_trigger_input(&mut self, name: &str) -> (u64, Option<u64>) {
        let result = self.native(|ctx| ctx.refill_trigger_input(name));
        observe(
            &self.label,
            "refill",
            &mut self.head_errors,
            result.is_err(),
        );
        result.unwrap_or((0, None))
    }
    fn sync_head_op(&mut self, name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        let answer = self
            .native(|ctx| ctx.sync_head_op(name, op))
            .unwrap_or(SyncOpAnswer::Failed);
        observe(
            &self.label,
            "sync head",
            &mut self.head_errors,
            answer == SyncOpAnswer::Failed,
        );
        answer
    }
    fn supports_sync_head_ops(&self) -> bool {
        true
    }
    fn probes_trigger_input(&self) -> bool {
        true
    }
    fn probe_trigger_input(&mut self, name: &str) -> TriggerProbe {
        match self.sync_head_op(name, SyncHeadOp::ProbeNext) {
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
        self.native(|ctx| ctx.sync_capture_stamp(name, slot, location))
            .ok()
            .flatten()
    }
    fn state_shape(&self) -> Option<u64> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return None;
        }
        let mut shape = 0;
        // SAFETY: SDK validated complete immutable state table retained by library;
        // exclusive stack output is live through synchronous, handle-free query.
        let status = unsafe { ((*ptr).shape)(&mut shape) };
        (status.code == abi::status_code::OK).then_some(shape)
    }
    // hot-path-alloc-ok-fn: anchor restoration and its diagnostics are cold
    fn restore_state(&mut self, bytes: &[u8]) -> TransportResult<()> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return Err(self.error("node declares no restorable state".to_owned()));
        }
        // SAFETY: checked state table/code and instance retained, exclusive entry;
        // payload is host-owned and borrowed only through this synchronous call.
        let status = unsafe {
            ((*ptr).restore)(
                self.handle()?,
                abi::Bytes {
                    ptr: bytes.as_ptr(),
                    len: bytes.len() as u64,
                },
            )
        };
        self.result(status, "restore state")
    }
    // hot-path-alloc-ok-fn: state capture diagnostics allocate only on refusal
    fn capture_state(&self, sink: &mut dyn crate::state::StateSink) -> TransportResult<()> {
        let ptr = self.api().state;
        if ptr.is_null() {
            return Err(self.error("node declares no capturable state".to_owned()));
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
        let status = unsafe { ((*ptr).capture)(self.handle()?, &output, capacity) };
        if bridge.panicked {
            return Err(self.error("host state sink panicked".to_owned()));
        }
        if bridge.full || status.code == abi::status_code::BUFFER_TOO_SMALL {
            bridge.sink.refuse();
            return Err(self.error("host state sink refused capture".to_owned()));
        }
        self.result(status, "capture state")
    }
    fn inline_safe(&self) -> bool {
        let ptr = self.api().state;
        let Some(instance) = self.instance else {
            return false;
        };
        if ptr.is_null() {
            return false;
        }
        let mut safe = 0;
        // SAFETY: checked library/state/instance live; stack scalar exclusively
        // borrowed for synchronous query, graph serializes instance calls.
        let status = unsafe { ((*ptr).inline_safe)(instance, &mut safe) };
        status.code == abi::status_code::OK && safe == 1
    }
    fn cer_probe(&self) -> bool {
        let ptr = self.api().state;
        let Some(instance) = self.instance else {
            return false;
        };
        if ptr.is_null() {
            return false;
        }
        // SAFETY: checked state table and instance live; graph serializes the
        // synchronous probe with all other calls and retains library until return.
        unsafe { ((*ptr).probe)(instance).code == abi::status_code::OK }
    }
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
        let status = unsafe { ((*ptr).describe)(instance, &mut source) };
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
        observe(
            &self.label,
            "external source",
            &mut self.external_errors,
            !valid,
        );
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

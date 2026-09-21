// SPDX-License-Identifier: AGPL-3.0-only
//! Error types for cerulion_core.

/// Transport layer errors.
///
/// `#[non_exhaustive]`: future variants are
/// additive without breaking downstream pattern matches. Existing public
/// variants are stable; the attribute only forces a `_` arm in
/// `match` expressions across crate boundaries.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum TransportError {
    /// Failed to create iceoryx2 node.
    #[error("Failed to create node '{node_name}': {reason}. Check that no other Cerulion process is using the same node name, and that shared memory permissions are correct.")]
    NodeCreation { node_name: String, reason: String },
    /// Failed to create publisher.
    #[error("Failed to create publisher on topic '{topic}': {reason}. Verify the topic name in your graph YAML and ensure no conflicting publisher exists.")]
    PublisherCreation { topic: String, reason: String },
    /// Failed to create subscriber.
    #[error("Failed to create subscriber on topic '{topic}': {reason}. Verify the topic name in your graph YAML and that the publisher for this topic is configured.")]
    SubscriberCreation { topic: String, reason: String },
    /// Failed to loan memory for publishing.
    #[error("Failed to loan shared memory on topic '{topic}': {reason}. The shared memory buffer may be exhausted — try reducing publish rate or increasing max_slice_len in your graph YAML.")]
    Loan { topic: String, reason: String },
    /// Failed to publish message.
    #[error("Failed to publish on topic '{topic}': {reason}. Check that subscribers are draining messages and the transport layer is healthy.")]
    Publish { topic: String, reason: String },
    /// Failed to receive message.
    #[error("Failed to receive on topic '{topic}': {reason}. Ensure a publisher exists for this topic and the transport layer is running.")]
    Receive { topic: String, reason: String },
    /// Topic not found in the transport layer.
    #[error("Topic '{topic}' not found. Verify the topic name matches your graph YAML and that the publisher has been created before the subscriber.")]
    TopicNotFound { topic: String },
    /// Transport manager not initialized.
    #[error("Transport manager not initialized. Call TransportManager::init() before creating publishers or subscribers.")]
    NotInitialized,
    /// Node already registered with this ID.
    #[error("Duplicate node ID '{node_id}'. Each node in the graph must have a unique ID — check your graph YAML for duplicate entries.")]
    DuplicateNode { node_id: String },
    /// Node not found in scheduler.
    #[error("Node '{node_id}' not found in the scheduler. Verify the node ID matches an entry in your graph YAML.")]
    NodeNotFound { node_id: String },
    /// Scheduler internal error.
    #[error("Scheduler error: {reason}")]
    SchedulerError { reason: String },
    /// Graph configuration or validation error.
    #[error("Graph validation error: {reason}")]
    GraphError { reason: String },
    /// Formerly `HostDrivenExternalOnLivePath`: one or more
    /// `#[cerulion_node(external)]` nodes are provably inert at launch under
    /// `cerulion graph run` — nothing can fire them once the live loop (or the
    /// `--time-source virtual` poll loop) owns the runtime, so the run is refused
    /// at startup rather than left running-but-blind. Each offender carries its
    /// [`InertReason`] (host-driven / no-source / invalid-fd / duplicate-fd /
    /// poisoned-node / doorbell-failed / virtual-time), all discovered at the same
    /// moment (collect-time, before the loop; or the virtual pre-flight) with the
    /// same outcome. All offenders are named in ONE aggregated error so the
    /// operator fixes them in a single pass. The POLLED `step()` seam is
    /// unaffected — a host driving `step()` CAN call `trigger_external`, so a
    /// host-driven node is legitimate there and is never refused.
    ///
    /// `nodes` is the STRUCTURED per-offender list `(node id, reason)` (it
    /// replaced a bare `Vec<String>` so it could carry the reason; the old bare-string
    /// form false-positived on a prefix like `"cam"` vs `"camera"`).
    ///
    /// The Display is rendered by `render_inert_at_launch` and is DYNAMIC:
    /// the opening (graph + `node (reason)` list + the
    /// one-line why) keeps the original wording, and its key substrings are pinned by
    /// the named tests below — then a legend line is emitted ONLY for each
    /// DISTINCT reason actually present, and the fix
    /// menu lists only the applicable remedies (`--time-source real` only when
    /// `virtual time` is among the offenders; the self-source/fd/doorbell repair
    /// line only when a source-class reason is present; the embedding-host
    /// `step()` seam always — it is universally valid). A lone-`HostDriven`
    /// refusal no longer explains fd/doorbell/virtual-time noise it doesn't have.
    /// The exact rendered text is pinned by the exhaustive
    /// `external_nodes_inert_at_launch_display_is_dynamic_per_reason` oracle
    /// (this file), `external_inert_refusal_surfaces_through_cli_error`
    /// (`graph_cmd.rs`), and the live-path refusal tests in
    /// `external_gating_replay_iox2_test.rs` / `external_live_fire_iox2_test.rs`.
    #[error("{}", render_inert_at_launch(.graph, .nodes))]
    ExternalNodesInertAtLaunch {
        graph: String,
        nodes: Vec<(String, InertReason)>,
    },
    /// The live loop (`run_live`) could not
    /// build its iceoryx2 `WaitSet` reactor at startup, so the graph cannot
    /// run on the live path.
    ///
    /// Logging and returning as if the run had completed (`run_live`
    /// returning `()`) would let `cerulion graph run` exit 0 for a
    /// live run that never executed a single step — a false success. So the
    /// underlying iceoryx2 `WaitSetCreateError` is propagated (carried in
    /// `reason`) so the CLI surfaces it and exits non-zero. Only the
    /// non-park run hard-requires the reactor; a monitor-wait park run skips
    /// the build entirely and never produces this error.
    #[error("Failed to build the live-loop WaitSet reactor for graph '{graph}': {reason}. The live event loop cannot run without it — check shared-memory permissions and that the process is not at its file-descriptor / WaitSet limit.")]
    LiveReactorBuild { graph: String, reason: String },
    /// Invalid transport configuration, rejected at transport init
    /// (e.g. a zero subscriber buffer).
    #[error("Invalid transport configuration: {reason}")]
    InvalidTransportConfig { reason: String },
    /// The transport was handed a node identity iceoryx2 cannot
    /// represent as a `NodeName`, so no node can be built for it.
    ///
    /// The name is DERIVED from the graph's identity — `cerulion_{graph}` for a
    /// `graph run`, `cerulion_replay_{identity}` for a resim — so every byte of
    /// it past the prefix is user-supplied, and the two constraints it can break
    /// are iceoryx2's: a byte-length cap, and a charset narrower than UTF-8
    /// (ASCII below U+0080, no NUL — the check lives in `iceoryx2-bb-container`'s
    /// `insert_bytes`, one level below the capacity-only `StaticString`
    /// conversion). A graph named `café`, or one whose name overruns the cap,
    /// breaks it.
    ///
    /// Before this variant existed, `TransportManager::init` `.expect()`ed that
    /// conversion: such a graph — and any bag DECLARING such a `name:` — ABORTED
    /// the process with no exit code and no diagnostic. The resim's bag-stem route
    /// was already closed by asking `NodeName::new` before adopting a stem;
    /// this closes the declared-identity route the same way, at the one place
    /// every `graph run` and every resim shares.
    ///
    /// `cause` says WHICH constraint broke so the message can carry the two
    /// numbers on the length arm (the bag-stem precedent) instead of the generic
    /// charset sentence. It is a CLASSIFICATION of a refusal iceoryx2 has
    /// already made — never a second copy of iceoryx2's rule deciding the
    /// verdict itself, which would be free to drift from it silently.
    #[error("{}", render_unrepresentable_node_name(.node_name, .cause))]
    UnrepresentableNodeName {
        /// The node name as the transport received it, prefix included — what a
        /// reader needs in order to see how much of it they control.
        node_name: String,
        /// Which of iceoryx2's two node-name constraints the name broke.
        cause: NodeNameRefusal,
    },
    /// YAML parse error.
    #[error("Failed to parse graph YAML: {reason}. Check YAML syntax (indentation, colons, dashes) and compare against the graph format in the documentation.")]
    GraphParseError { reason: String },
    /// Node init/tick/shutdown failure.
    #[error("Node '{node_id}' error: {reason}")]
    NodeError { node_id: String, reason: String },
    /// A node's tick suffered a PANIC-CLASS failure (the exit-3
    /// widening): the cdylib FFI caught a panic in the user tick (FFI code
    /// 2) or the cdylib is poisoned-dead after one (`NODES` mutex poisoned,
    /// FFI code 3 — the collapse-into-the-originating-panic state). This is
    /// the STRUCTURAL flag threaded from the FFI return code so callers
    /// (the graph runtime, the replay engine) can classify panic-class
    /// failures without matching on message text. A deterministic tick
    /// `Err` (user logic returning `Err`) is NORMAL execution and stays
    /// [`TransportError::NodeError`] — never this variant. In-process
    /// (non-cdylib) panics don't need it: they unwind to the scheduler's
    /// `catch_unwind` and are recorded at the catch site.
    #[error("Node '{node_id}' tick PANICKED (panic-class failure): {reason}")]
    NodeTickPanicked { node_id: String, reason: String },
    /// `cerulion_node_info()` returned metadata that could not be parsed.
    ///
    /// Previously, a cdylib whose `cerulion_node_info()` returned
    /// corrupted JSON silently degraded to `NodeInfo::default()` (empty
    /// wiring, no policy) — the node loaded but never fired. Now
    /// `NodeEntry::info()` is fallible and `GraphRuntime::build` /
    /// `build_in_process` refuse to construct a runtime containing such
    /// a node. The Display carries the diagnostic label (cdylib path),
    /// the raw JSON length, and the first bytes of the offending payload
    /// so operators can identify which cdylib is corrupted.
    #[error("Node '{diag_label}' returned unparseable info metadata from cerulion_node_info(): {reason} (json_len={json_len}, prefix={prefix:?}). Refusing to load the node — rebuild the cdylib so its info JSON matches the documented {{\"inputs\":[...],\"outputs\":[...]}} shape.")]
    NodeInfoParse {
        /// Diagnostic label identifying the node entry (typically the
        /// cdylib path).
        diag_label: String,
        /// Byte length of the raw JSON string returned over FFI.
        json_len: usize,
        /// First bytes (≤ 80, truncated at a UTF-8 char boundary) of
        /// the offending payload.
        prefix: String,
        /// Underlying parse failure description.
        reason: String,
    },
    /// Failed to create zenoh session.
    #[error("Failed to create zenoh session: {reason}. Check network connectivity and zenoh configuration (endpoints, scouting settings).")]
    SessionCreation { reason: String },
    /// Schema hash mismatch during typed deserialization (hash-only, used by subscriber).
    ///
    /// The schema hash is layout-sensitive, so the most common
    /// cause is a schema edit followed by rebuilding only one side — hence
    /// the rebuild hint, which mirrors the CLI's `rebuild_hint`
    /// (`cerulion node build <type>`) emitted by the stale-cdylib advisory.
    #[error(
        "Schema mismatch on topic '{topic}': expected hash 0x{expected_hash:016X}, got 0x{actual_hash:016X}. Check that publisher and subscriber use the same schema; if you recently changed a schema, rebuild BOTH sides (`cerulion node build <type>`)."
    )]
    SchemaMismatch {
        topic: String,
        expected_hash: u64,
        actual_hash: u64,
    },
    /// Schema hash mismatch with full schema names (used when names are available).
    #[error("Schema mismatch on topic '{topic}': expected '{expected_schema}' (hash 0x{expected_hash:016X}) but received '{actual_schema}' (hash 0x{actual_hash:016X}). Check that publisher and subscriber use the same schema in your graph YAML; if you recently changed a schema, rebuild BOTH sides (`cerulion node build <type>`).")]
    SchemaHashMismatch {
        topic: String,
        expected_schema: String,
        expected_hash: u64,
        actual_schema: String,
        actual_hash: u64,
    },
    /// Wire deserialization error (e.g., buffer too small, parse failure).
    #[error("Deserialization error on topic '{topic}': {reason}")]
    Deserialization { topic: String, reason: String },
    /// Message too large for the allocated buffer.
    #[error("Message too large for buffer on topic '{topic}': needs {needed} bytes but buffer is {available} bytes. Increase max_slice_len in your graph YAML.")]
    BufferTooSmall {
        topic: String,
        needed: usize,
        available: usize,
    },
    /// Variable-length proxy field write would exceed the loaned buffer.
    ///
    /// Returned synchronously from `loan_<field>` / `set_<field>` / `push_<field>`
    /// when the cumulative variable payload would exceed the publisher's `max_slice_len`.
    /// Zero allocations on this error path.
    #[error("Variable-field write too large: requested {requested} bytes, only {available} bytes remain in the loaned SHM buffer. Increase max_slice_len for this topic.")]
    ProxyBufferTooSmall { requested: usize, available: usize },
    /// A declared variable field was never written before `OutputProxy` was dropped.
    ///
    /// Logged via `tracing::error!` from `OutputProxy::Drop`; the message is *not*
    /// published when this occurs.
    #[error("OutputProxy dropped without writing required variable field '{field}'. Variable fields have no default — call set_/loan_/push_ before drop, or pass an explicit empty value (e.g. `set_data(&[])`).")]
    MissingVariableField { field: &'static str },
    /// Publisher reported loan-capacity exhaustion.
    ///
    /// Distinct from a generic `Loan` failure. iceoryx2 is the only transport
    /// backend (the in-process heap one was deleted; its
    /// `Vec::try_reserve_exact` OOM used to be the second producer of this
    /// variant), and it reaches here two ways:
    /// - **sample pool exhausted**: iceoryx2's "no more samples available"
    ///   path. Addressed by adding subscriber drain pressure or raising the
    ///   per-publisher slot count.
    /// - **simultaneous-loan budget exhausted**: more outstanding
    ///   `loan_proxy` loans than `publisher_max_loaned_samples` allows
    ///   (iceoryx2's `LoanError::ExceedsMaxLoans`). Addressed by dropping a
    ///   held loan or raising that knob.
    ///
    /// The Display message names both remedies, since the caller cannot tell
    /// the two apart from the error alone.
    #[error("Loan capacity exhausted on topic '{topic}': the publisher could not allocate a slot. Either iceoryx2's sample pool is exhausted (wait for subscribers to drop in-flight samples, or raise the slot count) or this publisher's simultaneous-loan budget is full (drop a held loan, or raise publisher_max_loaned_samples).")]
    LoanCapacity { topic: String },
    /// Variable-schema publisher created without configuring `max_slice_len`.
    ///
    /// For variable-length message types (Image, LaserScan, PointCloud2, ...) the
    /// publisher must know the maximum payload up front so every loan can be
    /// `max_slice_len`-sized (the SHM-backed proxy never resizes after loan).
    #[error("Publisher for variable-schema topic '{topic}' must specify max_slice_len. Set it in the graph YAML or via TransportManager::create_publisher.")]
    MaxSliceLenRequired { topic: String },
    /// `push_<field>` called when the field is not at the cursor tail.
    ///
    /// Variable-field push semantics require the target field to be the most
    /// recently written variable field (i.e., its bytes sit at the writer
    /// cursor's tail). When another variable field has been written since,
    /// extending via `push_` would tear that subsequent field's payload —
    /// the proxy rejects the call instead. The user can switch to
    /// `set_<field>(value)` or `loan_<field>(n)` for non-tail extensions.
    #[error("push_{field} called after a non-tail write to '{field}' — interleaved pushes are unsupported. Use set_{field} or loan_{field} for non-tail writes.")]
    PushAfterNonTail { field: &'static str },
    /// A complex-nested variable field was written by BOTH the
    /// staged leaf sugar AND a whole-field write within one message loan
    /// (either order). The two mechanisms cannot coexist on one field:
    /// staged writes flush at `OutputProxy::Drop`, so "staged then whole"
    /// would let the earlier-in-program-order staged bytes overwrite the
    /// explicit write, and "whole then staged" would flush a fresh scratch
    /// carrying ONLY the sugared leaves, silently discarding the explicit
    /// write's other bytes. Both violate program-order intuition, so the
    /// second-mechanism write is rejected loudly. `detected` names the order.
    ///
    /// Fixed-nested paths get NO such guard — their leaf writes land
    /// immediately in disjoint SHM and are true last-write-wins.
    #[error(
        "Conflicting writes to nested field '{field}': {detected}. Write a nested field by \
         EXACTLY ONE mechanism per message — either the staged leaf sugar \
         (`self.<port>.{field}.<leaf> = …` / `with_{field}(…)`) OR a whole-field write \
         (`self.<port>.{field} = bytes` / `set_{field}_bytes` / `loan_{field}_bytes` / \
         `fill_from_{field}_bytes`), never both. Use exactly one write mechanism per field per \
         message."
    )]
    NestedWriteConflict {
        field: &'static str,
        detected: &'static str,
    },
    /// A STAGED complex-nested field was published with an
    /// unwritten variable child field. The every-variable-field publish gate
    /// applies to nested schemas too: a staged child (e.g. `Image.header`
    /// touched via `self.image.header.stamp.sec = …` but not
    /// `….frame_id = …`) must write EVERY variable-length field of its own
    /// schema, else the frame is discarded — the same rule the top-level
    /// schema obeys. Fixed child leaves stay ungated (scratch-zero default),
    /// mirroring the top-level fixed/variable asymmetry. Names the dotted
    /// location `<parent_field>.<child_field>`.
    ///
    /// `child_field` is an owned `String` because the recursive flush
    /// (recursive staging persistence) PREPENDS each
    /// intermediate level's field name as the error bubbles up — a depth-2
    /// miss surfaces as `parent_field: "joint_trajectory", child_field:
    /// "header.frame_id"`, so the paste-ready remediation path in the
    /// message resolves to the field the user actually writes. Depth-1
    /// misses carry the plain child name unchanged.
    #[error(
        "Nested field '{parent_field}' published with an unwritten variable child field \
         '{parent_field}.{child_field}' — every variable-length field of a nested schema must be \
         written each tick (e.g. `self.<port>.{parent_field}.{child_field} = …`), or write the \
         whole field as bytes. Frame discarded."
    )]
    NestedChildIncomplete {
        parent_field: &'static str,
        child_field: String,
    },
    /// User payload exceeds the publisher's configured `max_slice_len`.
    ///
    /// The spill helper (`<Name>Shm::ensure_capacity_for`) emits
    /// this error when the cumulative variable-field cursor would exceed
    /// the schema's `max_capacity` (= `max_slice_len - WireHeader::SIZE`).
    /// At this point no heap fallback can rescue the write — the user
    /// must raise `max_slice_len:` in graph YAML or use a schema with a
    /// larger `<T as ShmMessage>::MAX_SLICE_LEN` default.
    ///
    /// Previously the equivalent failure path was `ProxyBufferTooSmall`,
    /// which is now retained only as a hand-written-path variant (codegen
    /// no longer emits it — all setter-overflow failures route through
    /// `PayloadTooLarge` or via spill to `Ok`).
    #[error("Payload too large for topic '{topic}': requested {requested} bytes, max_slice_len ceiling is {max} bytes. Increase max_slice_len in graph YAML or use a schema with a larger MAX_SLICE_LEN default.")]
    PayloadTooLarge {
        topic: String,
        requested: usize,
        max: usize,
    },
    /// Heap allocation failed during the fallback-buffer path.
    ///
    /// Returned when `Vec::with_capacity(n)` panics-equivalent (or a
    /// custom allocator returns failure) inside the publisher's overflow
    /// recovery path. The system is in OOM at this point and there is
    /// nothing the framework can do to recover from inside a tick — the
    /// caller should treat this as a fatal-system-state error and
    /// shut the graph down.
    ///
    /// Returned by the in-tick overflow redirect when its heap fallback
    /// buffer cannot be allocated; defined on the shared error surface so
    /// every writer path names the same variant.
    #[error("Heap allocation failed for topic '{topic}': could not allocate {requested} bytes for the overflow fallback buffer. The system is in OOM — shut down the graph and investigate memory usage.")]
    AllocationFailed { topic: String, requested: usize },
    /// Internal error (e.g., poisoned mutex).
    #[error("Internal error: {reason}. This is a bug in Cerulion — please report it.")]
    Internal { reason: String },
}

/// Why an `external`-policy node is provably inert at launch under
/// `cerulion graph run`. Every variant is discovered at graph startup — the
/// collect-time arms (host-driven / no-source / invalid-fd / duplicate-fd /
/// poisoned-node / doorbell-failed) by `GraphRuntime::collect_external_sources`,
/// and `VirtualTime` by the CLI's `--time-source virtual` pre-flight — all with
/// the same outcome: a node that cannot fire while the runtime is owned by the
/// live/poll loop. Carried per-offender in
/// [`TransportError::ExternalNodesInertAtLaunch`] so ONE aggregated launch error
/// names every node AND why nothing can fire it. `Copy` — a plain reason tag,
/// cheap to store alongside the node id in the sticky refusal record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InertReason {
    /// Declared `ExternalSource::HostDriven` — fires only via a host `trigger_external`.
    HostDriven,
    /// The external-policy node returned no `ExternalSource` (`None`).
    NoSource,
    /// The node's `ExternalSource::Fd` is invalid/dead (negative or not a live
    /// descriptor) — it cannot be watched. (A LIVE fd `>= FD_SETSIZE` under the
    /// select path is the DISTINCT `FdAboveSelectLimit`.)
    InvalidFd,
    /// The node's `ExternalSource::Fd` is a LIVE descriptor `>= FD_SETSIZE`
    /// (1024) AND the run watches fds with `select` (the monitor-wait park is off),
    /// so the select-backed WaitSet cannot watch it (HAZARD 2 — out-of-bounds
    /// `FD_SET`). Distinct from `InvalidFd`: the fd is FINE — the WATCH MECHANISM
    /// has the ceiling. Fixed by enabling the park (which polls via `poll(2)`, no
    /// ceiling) OR shrinking the graph so the descriptor is minted below FD_SETSIZE.
    FdAboveSelectLimit,
    /// The node's fd aliases one an already-bound sibling owns (a dup collides on the
    /// WaitSet attachment id — one node would silently never wake).
    DuplicateFd,
    /// The node's entry mutex was poisoned, so its `external_source()` could not be queried.
    PoisonedEntry,
    /// The node's tier-2 `ExternalSource::Blocking` doorbell could not be established
    /// (set-nonblocking failed, doorbell creation failed, or no `TransportManager`).
    DoorbellFailed,
    /// Running under `--time-source virtual`: the deterministic poll loop never sweeps
    /// fds or calls `trigger_external`, so every external-policy node is inert.
    VirtualTime,
}

impl InertReason {
    /// Every variant in discriminant order. Drives the DETERMINISTIC legend
    /// ordering in [`render_inert_at_launch`] and is the source vector for the
    /// exhaustive `external_nodes_inert_at_launch_display_is_dynamic_per_reason`
    /// oracle.
    ///
    /// ENFORCEMENT CHAIN (why a stale `ALL` cannot silently ship): the
    /// `inert_reason_all_is_exhaustive_and_unique` unit test pairs this array
    /// with an exhaustive `match`-based ordinal fn — adding a variant breaks
    /// that `match` at COMPILE time (forcing the update), and the test asserts
    /// `ALL.len() == 8` with every ordinal present exactly once. Independently,
    /// the exhaustive `match`es in [`Self::legend`] and [`std::fmt::Display`]
    /// also refuse to compile without a new arm. Together they make this
    /// manually-maintained list self-correcting.
    const ALL: [InertReason; 8] = [
        Self::HostDriven,
        Self::NoSource,
        Self::InvalidFd,
        Self::FdAboveSelectLimit,
        Self::DuplicateFd,
        Self::PoisonedEntry,
        Self::DoorbellFailed,
        Self::VirtualTime,
    ];

    /// The per-reason legend line rendered by [`render_inert_at_launch`] when
    /// THIS reason is among the offenders (fix 1: only present reasons get a
    /// line). Each begins with its own `` `<display>` = `` marker so the legend
    /// for one reason is never a substring of another's — the oracle test
    /// asserts absent-reason legends never leak into the message.
    ///
    /// PINNED CONTRACT: these strings are asserted verbatim by the tests + the
    /// live-path refusal pins. Editing wording is a message-contract change
    /// (re-bless the oracle); adding an [`InertReason`] variant REQUIRES a new
    /// arm here (and an [`Self::ALL`] entry) — see the Display note below.
    fn legend(&self) -> &'static str {
        match self {
            Self::HostDriven => {
                "`host-driven` = the node fires ONLY via a host `trigger_external` call"
            }
            Self::NoSource => {
                "`no source` = the node declared no `ExternalSource`, so it fires ONLY via a host `trigger_external` call"
            }
            Self::InvalidFd => {
                "`invalid fd` = the declared `ExternalSource::Fd` is invalid/dead/unwatchable, so the live loop can never wake it"
            }
            Self::FdAboveSelectLimit => {
                "`fd above select limit` = the declared `ExternalSource::Fd` is a live descriptor >= FD_SETSIZE (1024); the select-backed WaitSet cannot watch it with the monitor-wait park off"
            }
            Self::DuplicateFd => {
                "`duplicate fd` = the declared `ExternalSource::Fd` aliases another node's fd and cannot be watched independently"
            }
            Self::PoisonedEntry => {
                "`poisoned node` = the node's entry state is poisoned, so its `ExternalSource` could not be queried"
            }
            Self::DoorbellFailed => {
                "`doorbell failed` = its `ExternalSource::Blocking` doorbell could not be established"
            }
            Self::VirtualTime => {
                "`virtual time` = under `--time-source virtual` nothing sweeps fds or calls `trigger_external`"
            }
        }
    }

    /// True for the reasons whose remedy is "give the node a working self-source
    /// / repair its fd/doorbell" — i.e. every reason EXCEPT `VirtualTime` (fixed
    /// by `--time-source real`), `PoisonedEntry` (a node-panic bug: the
    /// node's entry mutex is poisoned, so it can tick on NO seam — NOT even the
    /// polled `step()` one — and instead gets its own "fix the poisoned node and
    /// restart the graph" fix-menu option), and `FdAboveSelectLimit` (the
    /// fd is FINE — the select WATCH MECHANISM has the ceiling, so its remedy is
    /// enabling the park / shrinking the graph, NOT giving the node a new source;
    /// it too gets its own fix-menu option). Gates the self-source line of the
    /// fix menu in [`render_inert_at_launch`] (fix 1).
    fn is_source_class(&self) -> bool {
        matches!(
            self,
            Self::HostDriven
                | Self::NoSource
                | Self::InvalidFd
                | Self::DuplicateFd
                | Self::DoorbellFailed
        )
    }
}

/// Display: the machine-facing SHORT tag (`host-driven`, `invalid fd`, …). Used
/// in the `node (reason)` list of the aggregated refusal and in structured
/// logs.
///
/// PINNED CONTRACT: these tag strings — and the longer per-reason `legend`
/// lines — are a stable message contract asserted by the oracle test and the
/// live-path refusal pins. Adding a variant requires (1) an `ALL` entry, (2) a
/// `legend` arm, (3) an `is_source_class` decision, and (4) a Display arm here;
/// the exhaustive oracle enforces the legend + fix-menu wiring so a new variant
/// cannot silently ship with no legend. Step 1 (the `ALL` entry) is itself
/// compile-forced by the `inert_reason_all_is_exhaustive_and_unique` test's
/// exhaustive ordinal `match`.
impl std::fmt::Display for InertReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::HostDriven => "host-driven",
            Self::NoSource => "no source",
            Self::InvalidFd => "invalid fd",
            Self::FdAboveSelectLimit => "fd above select limit",
            Self::DuplicateFd => "duplicate fd",
            Self::PoisonedEntry => "poisoned node",
            Self::DoorbellFailed => "doorbell failed",
            Self::VirtualTime => "virtual time",
        })
    }
}

/// Which of iceoryx2's two node-name constraints a REFUSED name
/// broke, carried by [`TransportError::UnrepresentableNodeName`].
///
/// A classification, not a verdict: iceoryx2 has already refused the name by the
/// time one of these is minted (the classifier is `transport`'s crate-private
/// `classify_node_name_refusal`, deliberately not linked — a public doc item may
/// not link a private one, and the docs gate is `-D warnings`).
/// Two arms because they have DIFFERENT remedies and different messages — a
/// length overrun can quote the two numbers an operator acts on, while a charset
/// refusal has to explain that valid UTF-8 is not enough.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeNameRefusal {
    /// The name is longer than an iceoryx2 node name's byte capacity.
    TooLong {
        /// The offending name's length in BYTES (the cap is a byte capacity).
        len: usize,
        /// `NodeName::max_len()` — the longest name iceoryx2 admits.
        max: usize,
    },
    /// iceoryx2 refused a name that FITS, so the charset is what broke: a node
    /// name admits only ASCII below U+0080 and no NUL byte.
    ///
    /// Deliberately carries no offending byte or offset. The refusal is
    /// iceoryx2's and it does not report one, so any position named here would
    /// be re-derived from a rule this crate must not own a second copy of.
    Charset,
}

/// Render [`TransportError::UnrepresentableNodeName`] — the offending
/// name, the constraint it broke, and the remedy (the project's rule: an error
/// names the CAUSE and what to do about it).
fn render_unrepresentable_node_name(node_name: &str, cause: &NodeNameRefusal) -> String {
    let why = match cause {
        NodeNameRefusal::TooLong { len, max } => {
            format!("it is {len} bytes, past the {max} an iceoryx2 node name admits")
        }
        NodeNameRefusal::Charset => "an iceoryx2 node name admits only ASCII below U+0080, and no \
             NUL byte — so an accent, an emoji or any other non-ASCII character is refused even \
             though it is valid UTF-8"
            .to_string(),
    };
    format!(
        "Cannot name the iceoryx2 node '{node_name}': {why}. That name is DERIVED from the \
         graph's identity (its `name:` key, or the file or bag it was loaded from) — rename the \
         graph, or the bag, and re-run."
    )
}

/// Render [`TransportError::ExternalNodesInertAtLaunch`]
/// with a dynamic legend + fix menu instead of the original
/// static glossary that explained ALL seven reasons on every refusal (a
/// lone-`HostDriven` refusal used to print fd/doorbell/virtual-time noise + a
/// `--time-source real` remedy with no `VirtualTime` offender).
///
/// Structure:
/// 1. Opening — graph + `node (reason)` list + the one-line why. This is
///    the original wording; its key substrings are pinned by the live-path refusal
///    tests.
/// 2. Legend — one line per DISTINCT reason PRESENT, in [`InertReason::ALL`]
///    (discriminant) order for determinism.
/// 3. Fix menu — the self-source/fd/doorbell-repair line ONLY when a
///    source-class reason is present; the `--time-source real` line ONLY when
///    `VirtualTime` is present; the embedding-host `step()` seam ALWAYS (it is
///    universally valid).
fn render_inert_at_launch(graph: &str, nodes: &[(String, InertReason)]) -> String {
    use std::fmt::Write as _;

    let node_list = nodes
        .iter()
        .map(|(n, r)| format!("{n} ({r})"))
        .collect::<Vec<_>>()
        .join(", ");

    // Distinct reasons present, walked in discriminant order (deterministic).
    let present: Vec<InertReason> = InertReason::ALL
        .iter()
        .copied()
        .filter(|r| nodes.iter().any(|(_, nr)| nr == r))
        .collect();

    let mut msg = format!(
        "Refusing to run graph '{graph}' on the live path: external-policy node(s) [{node_list}] are provably inert at launch — nothing can fire them once `cerulion graph run` owns the runtime."
    );

    // Legend: one clause per DISTINCT present reason (no glossary noise for
    // reasons that aren't here).
    msg.push_str(" Reasons:");
    for r in &present {
        let _ = write!(msg, " {};", r.legend());
    }

    // Fix menu: COLLECT the applicable options, then join with "; OR " so the
    // FIRST option is never prefixed by a dangling "OR" (fix 1). Pushing a
    // pre-fixed " OR …" clause per remedy broke when the leading source-class
    // clause was absent — a VirtualTime-only (the common virtual-arm case) or
    // PoisonedEntry-only refusal rendered "Fix each named node: OR (for …" with
    // an "OR" opening the menu. Building a Vec and joining removes the ordering
    // assumption entirely.
    let mut options: Vec<&'static str> = Vec::new();
    if present.iter().any(InertReason::is_source_class) {
        options.push(
            "give it a working self-source (a readable descriptor via `ExternalSource::Fd`, or a blocking event source via `ExternalSource::Blocking`) and resolve any listed fd/doorbell problem",
        );
    }
    if present.contains(&InertReason::PoisonedEntry) {
        // fix 2: a poisoned node cannot tick on ANY seam (its entry mutex is
        // poisoned), so neither the self-source repair nor the polled `step()`
        // seam heals it — its own remedy is to fix the panicking node + restart.
        options.push(
            "fix the node whose state is poisoned (its init/tick panicked) and restart the graph",
        );
    }
    if present.contains(&InertReason::FdAboveSelectLimit) {
        // The fd is a LIVE descriptor — the select-backed WaitSet just
        // cannot watch a value >= FD_SETSIZE (HAZARD 2). The remedy is the WATCH
        // MECHANISM (enable the `poll(2)`-based monitor-wait park, which has no
        // fd-number ceiling) or a smaller fd number, NOT a new self-source — so
        // this reason is deliberately absent from `is_source_class`. The two
        // workarounds ride ONE menu option joined by "; OR" internally; the option
        // itself does not open with "OR", so the no-dangling-OR invariant holds.
        options.push(
            "(for `fd above select limit`) enable the monitor-wait park — drop `--no-monitor-wait` / unset `CERULION_MONITOR_WAIT=0` (the park polls the fd via poll(2), no FD_SETSIZE ceiling); OR shrink the graph so the descriptor is minted below FD_SETSIZE",
        );
    }
    if present.contains(&InertReason::VirtualTime) {
        options.push("(for `virtual time`) re-run with `--time-source real`");
    }
    // The embedding-host polled seam is universally valid for the CLASS of
    // refusals (a host driving `step()` CAN call `trigger_external`) — always
    // offered as the final option. (It does NOT itself resurrect a poisoned
    // node; that node still needs its own PoisonedEntry remedy above.)
    options.push(
        "drive the graph from an embedding host through the polled `GraphRuntime::step()` seam (which CAN call `trigger_external` between steps) instead of `cerulion graph run`",
    );

    let _ = write!(msg, " Fix each named node: {}.", options.join("; OR "));

    msg
}

/// Result type alias for transport operations.
pub type TransportResult<T> = Result<T, TransportError>;

/// Node-level errors for declarative node tick logic.
///
/// Returned by the user's `tick(&mut self) -> Result<(), NodeError>` method
/// in declarative mode (`#[input]`/`#[output]` field attributes).
///
/// `#[non_exhaustive]`: the variant set is
/// expected to grow over time; the attribute means downstream
/// `match` expressions need a `_` arm but additions don't break SemVer.
/// `Clone` is intentionally not implemented — the `Custom` variant
/// holds a `Box<dyn Error>` that may not be Clone-able. Stringify via
/// `.to_string()` if you need an owned copy.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum NodeError {
    /// Non-fatal logic error.
    #[error("Logic error: {0}")]
    Logic(String),
    /// Input validation failure.
    #[error("Invalid input '{input}': {reason}")]
    InvalidInput { input: String, reason: String },
    /// Fatal error indicating unrecoverable failure — in INTENT. TODAY the
    /// runtime logs and counts it like any other tick error (every variant
    /// collapses to a string at the macro boundary) and the node continues
    /// to be scheduled; a true hard-stop lifecycle is tracked post-launch.
    #[error("Fatal error: {0}")]
    Fatal(String),
    /// Custom error wrapping any error type.
    #[error("{0}")]
    Custom(Box<dyn std::error::Error + Send + Sync>),
    // The phantom `Deactivate(String)` / `Reconfigure(String)`
    // variants were DELETED here (kill-don't-redocument). They had ZERO
    // consumers outside this file — no runtime path ever matched on them
    // (every NodeError collapses to a string at the macro boundary), so they
    // advertised lifecycle semantics (pause / reconfigure) that did not
    // exist. The enum is `#[non_exhaustive]`, so re-adding them alongside a
    // real lifecycle implementation is non-breaking.
    /// Wrapped transport-layer error.
    #[error("Transport error: {0}")]
    Transport(#[from] TransportError),
    /// Wrapped I/O error.
    ///
    /// Lets node bodies use `?` directly on `std::fs` / `std::io`
    /// returns — e.g. `OpenOptions::new().open(&path)?` in a node that
    /// writes a CSV log on shutdown — without `.map_err(|e|
    /// NodeError::Logic(e.to_string()))?` boilerplate.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

/// Result type alias for node tick operations.
pub type NodeResult<T> = Result<T, NodeError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_error_display() {
        let err = NodeError::Logic("division by zero".to_string());
        assert_eq!(err.to_string(), "Logic error: division by zero");

        let err = NodeError::InvalidInput {
            input: "scan".to_string(),
            reason: "stale data".to_string(),
        };
        assert_eq!(err.to_string(), "Invalid input 'scan': stale data");

        let err = NodeError::Fatal("sensor disconnected".to_string());
        assert_eq!(err.to_string(), "Fatal error: sensor disconnected");
    }

    #[test]
    fn test_node_error_from_transport_error() {
        let transport_err = TransportError::TopicNotFound {
            topic: "lidar/scan".to_string(),
        };
        let node_err: NodeError = transport_err.into();
        assert!(matches!(node_err, NodeError::Transport(_)));
        assert!(node_err.to_string().contains("lidar/scan"));
    }

    #[test]
    fn test_node_error_custom() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file missing");
        let err = NodeError::Custom(Box::new(io_err));
        assert!(err.to_string().contains("file missing"));
    }

    #[test]
    fn test_node_error_from_io_error() {
        // Bare `?` on a std::io::Result must convert
        // into NodeError::Io with the source io::Error preserved.
        fn open_missing() -> Result<std::fs::File, NodeError> {
            std::fs::File::open("/this/path/should/never/exist/g.txt")?;
            unreachable!()
        }
        let err = open_missing().expect_err("missing file should fail");
        assert!(matches!(err, NodeError::Io(_)));
        let display = err.to_string();
        assert!(display.starts_with("I/O error: "), "got: {display}");
    }

    // ============================================================
    // PayloadTooLarge / AllocationFailed display
    // ============================================================

    #[test]
    fn test_payload_too_large_display_contains_topic_and_sizes() {
        let err = TransportError::PayloadTooLarge {
            topic: "lidar/scan".to_string(),
            requested: 1_048_576,
            max: 65536,
        };
        let msg = err.to_string();
        assert!(
            msg.contains("lidar/scan"),
            "should contain topic name; got: {msg}"
        );
        assert!(
            msg.contains("1048576"),
            "should contain requested size; got: {msg}"
        );
        assert!(
            msg.contains("65536"),
            "should contain max ceiling; got: {msg}"
        );
        assert!(
            msg.contains("max_slice_len"),
            "should suggest fix; got: {msg}"
        );
    }

    #[test]
    fn test_allocation_failed_display_contains_topic_and_size() {
        let err = TransportError::AllocationFailed {
            topic: "camera/image".to_string(),
            requested: 4 * 1024 * 1024,
        };
        let msg = err.to_string();
        assert!(
            msg.contains("camera/image"),
            "should contain topic name; got: {msg}"
        );
        assert!(
            msg.contains("4194304"),
            "should contain requested size; got: {msg}"
        );
        assert!(msg.contains("OOM"), "should mention OOM; got: {msg}");
    }

    // ============================================================
    // Nested-write conflict + staged-child-incomplete display
    // ============================================================

    #[test]
    fn test_nested_write_conflict_display_staged_then_whole() {
        // Full Display pin, `detected` = staged-then-whole arm (guard A:
        // `loan_<f>_bytes` sees `__cer_staged_<f>.is_some()`). The exact
        // `detected` literal is what codegen's guard emits.
        let err = TransportError::NestedWriteConflict {
            field: "header",
            detected: "a staged nested-field write, then a whole-field write",
        };
        assert_eq!(
            err.to_string(),
            "Conflicting writes to nested field 'header': a staged nested-field write, then a \
             whole-field write. Write a nested field by EXACTLY ONE mechanism per message — \
             either the staged leaf sugar (`self.<port>.header.<leaf> = …` / `with_header(…)`) OR \
             a whole-field write (`self.<port>.header = bytes` / `set_header_bytes` / \
             `loan_header_bytes` / `fill_from_header_bytes`), never both. Use exactly one write \
             mechanism per field per message."
        );
    }

    #[test]
    fn test_nested_write_conflict_display_whole_then_staged() {
        // Full Display pin, `detected` = whole-then-staged arm (guard B: the
        // staged `__cer_with_nested_<f>` sees the field already written).
        let err = TransportError::NestedWriteConflict {
            field: "header",
            detected: "a whole-field write, then a staged nested-field write",
        };
        assert_eq!(
            err.to_string(),
            "Conflicting writes to nested field 'header': a whole-field write, then a staged \
             nested-field write. Write a nested field by EXACTLY ONE mechanism per message — \
             either the staged leaf sugar (`self.<port>.header.<leaf> = …` / `with_header(…)`) OR \
             a whole-field write (`self.<port>.header = bytes` / `set_header_bytes` / \
             `loan_header_bytes` / `fill_from_header_bytes`), never both. Use exactly one write \
             mechanism per field per message."
        );
    }

    #[test]
    fn test_nested_write_conflict_display_names_field_and_both_mechanisms() {
        let err = TransportError::NestedWriteConflict {
            field: "pose",
            detected: "a staged nested-field write, then a whole-field write",
        };
        let msg = err.to_string();
        // Names the field, BOTH mechanisms, the order, and the remedy.
        assert!(msg.contains("'pose'"), "names the field; got: {msg}");
        assert!(
            msg.contains("with_pose") && msg.contains("set_pose_bytes"),
            "names both mechanisms parameterized on the field; got: {msg}"
        );
        assert!(
            msg.contains("a staged nested-field write, then a whole-field write"),
            "names the detected order; got: {msg}"
        );
        assert!(
            msg.contains("exactly one write mechanism per field per message"),
            "states the remedy; got: {msg}"
        );
    }

    #[test]
    fn test_nested_child_incomplete_display() {
        let err = TransportError::NestedChildIncomplete {
            parent_field: "header",
            child_field: "frame_id".to_string(),
        };
        assert_eq!(
            err.to_string(),
            "Nested field 'header' published with an unwritten variable child field \
             'header.frame_id' — every variable-length field of a nested schema must be written \
             each tick (e.g. `self.<port>.header.frame_id = …`), or write the whole field as \
             bytes. Frame discarded."
        );
    }

    #[test]
    fn test_nested_child_incomplete_display_depth2_dotted_path() {
        // The recursive flush PREPENDS each intermediate
        // level's field name as the error bubbles up, so a depth-2 miss
        // renders the FULL user-writable path
        // (`self.<port>.joint_trajectory.header.frame_id`), not the bare
        // deepest pair alone (which would suggest a path
        // that does not compile).
        let err = TransportError::NestedChildIncomplete {
            parent_field: "joint_trajectory",
            child_field: "header.frame_id".to_string(),
        };
        assert_eq!(
            err.to_string(),
            "Nested field 'joint_trajectory' published with an unwritten variable child field \
             'joint_trajectory.header.frame_id' — every variable-length field of a nested schema \
             must be written each tick (e.g. `self.<port>.joint_trajectory.header.frame_id = …`), \
             or write the whole field as bytes. Frame discarded."
        );
    }

    #[test]
    fn test_payload_too_large_is_not_proxy_buffer_too_small() {
        // The two errors are distinct variants — different remediation
        // paths (raise max_slice_len vs increase loan size). Pin via
        // pattern match.
        let payload_err = TransportError::PayloadTooLarge {
            topic: "t".into(),
            requested: 1024,
            max: 512,
        };
        let proxy_err = TransportError::ProxyBufferTooSmall {
            requested: 1024,
            available: 512,
        };
        assert!(matches!(
            payload_err,
            TransportError::PayloadTooLarge { .. }
        ));
        assert!(!matches!(
            payload_err,
            TransportError::ProxyBufferTooSmall { .. }
        ));
        assert!(matches!(
            proxy_err,
            TransportError::ProxyBufferTooSmall { .. }
        ));
        assert!(!matches!(proxy_err, TransportError::PayloadTooLarge { .. }));
    }

    // ============================================================
    // ExternalNodesInertAtLaunch DYNAMIC Display
    // ============================================================

    /// Exhaustive oracle over the DYNAMIC glossary: for each `InertReason`,
    /// render a single-offender refusal and assert (a) `node (its-display)`
    /// appears, (b) its OWN legend line appears, (c) NO absent reason's legend
    /// line appears (a lone-`HostDriven` refusal must not print
    /// fd/doorbell/virtual-time noise), and (d) `--time-source real` appears iff
    /// `VirtualTime` is the offender. Walking `InertReason::ALL` means a new
    /// variant added without a legend arm fails to compile (`ALL` + the
    /// exhaustive `match` in `legend`) or fails this test — the pinned contract.
    #[test]
    fn external_nodes_inert_at_launch_display_is_dynamic_per_reason() {
        for &reason in &InertReason::ALL {
            let err = TransportError::ExternalNodesInertAtLaunch {
                graph: "percep".to_string(),
                nodes: vec![("node1".to_string(), reason)],
            };
            let msg = err.to_string();

            // (a) the `node (reason)` pair renders.
            let pair = format!("node1 ({reason})");
            assert!(
                msg.contains(&pair),
                "reason {reason:?}: message must contain `{pair}`; got: {msg}"
            );
            // opening + graph name are always present.
            assert!(
                msg.contains("percep") && msg.contains("provably inert at launch"),
                "reason {reason:?}: opening/graph must be present; got: {msg}"
            );

            // (b) THIS reason's legend line appears.
            assert!(
                msg.contains(reason.legend()),
                "reason {reason:?}: own legend line must appear; got: {msg}"
            );

            // (c) NO other reason's legend line appears (the dynamic-glossary pin).
            for &other in &InertReason::ALL {
                if other != reason {
                    assert!(
                        !msg.contains(other.legend()),
                        "reason {reason:?}: absent legend for {other:?} must NOT appear; got: {msg}"
                    );
                }
            }

            // (d) `--time-source real` iff VirtualTime is present.
            assert_eq!(
                msg.contains("--time-source real"),
                reason == InertReason::VirtualTime,
                "reason {reason:?}: `--time-source real` must appear IFF VirtualTime; got: {msg}"
            );

            // The universal `step()` seam is ALWAYS offered.
            assert!(
                msg.contains("GraphRuntime::step()"),
                "reason {reason:?}: the universal polled step() seam must always appear; got: {msg}"
            );

            // The self-source repair line appears IFF a source-class reason is present.
            assert_eq!(
                msg.contains("give it a working self-source"),
                reason.is_source_class(),
                "reason {reason:?}: self-source fix line must appear IFF source-class; got: {msg}"
            );

            // fix 2: the poisoned-node fix line appears IFF PoisonedEntry is present.
            assert_eq!(
                msg.contains("fix the node whose state is poisoned"),
                reason == InertReason::PoisonedEntry,
                "reason {reason:?}: poisoned-node fix line must appear IFF PoisonedEntry; got: {msg}"
            );

            // fix 1: NO dangling "OR" opens the menu — the first applicable
            // option is unprefixed however few options are present (this is the
            // single-reason case, where VirtualTime-only / PoisonedEntry-only
            // have no source-class leading clause).
            assert!(
                !msg.contains("Fix each named node: OR"),
                "reason {reason:?}: the fix menu must not open with a dangling `OR`; got: {msg}"
            );
        }
    }

    /// A MIXED refusal (source-class + `VirtualTime`) shows BOTH offenders in the
    /// list, BOTH legend lines, AND both the self-source and `--time-source real`
    /// remedies — proving the dynamic menu unions the present reasons rather than
    /// picking one.
    #[test]
    fn external_nodes_inert_at_launch_mixed_reasons_union_the_menu() {
        let err = TransportError::ExternalNodesInertAtLaunch {
            graph: "g".to_string(),
            nodes: vec![
                ("cam".to_string(), InertReason::HostDriven),
                ("clk".to_string(), InertReason::VirtualTime),
            ],
        };
        let msg = err.to_string();
        assert!(
            msg.contains("cam (host-driven)") && msg.contains("clk (virtual time)"),
            "both `node (reason)` pairs render; got: {msg}"
        );
        assert!(
            msg.contains(InertReason::HostDriven.legend())
                && msg.contains(InertReason::VirtualTime.legend()),
            "both legend lines render; got: {msg}"
        );
        assert!(
            msg.contains("give it a working self-source") && msg.contains("--time-source real"),
            "both remedies (self-source + time-source real) appear; got: {msg}"
        );
        // A reason NOT present (e.g. duplicate fd) leaks no legend.
        assert!(
            !msg.contains(InertReason::DuplicateFd.legend()),
            "an absent reason's legend must not leak; got: {msg}"
        );
        // fix 1: even a mixed refusal opens the menu with a real option, never
        // a dangling `OR`.
        assert!(
            !msg.contains("Fix each named node: OR"),
            "the fix menu must not open with a dangling `OR`; got: {msg}"
        );
    }

    /// fix 1 (exact-render): a VirtualTime-only refusal — THE common
    /// virtual-arm case — has NO source-class leading clause, so the menu's
    /// first option is the `--time-source real` remedy. Pinned by a full
    /// byte-for-byte compare so a regression to `Fix each named node: OR (for …`
    /// fails loudly (not merely a substring check).
    #[test]
    fn external_nodes_inert_virtual_time_only_renders_without_dangling_or() {
        let err = TransportError::ExternalNodesInertAtLaunch {
            graph: "g".to_string(),
            nodes: vec![("clk".to_string(), InertReason::VirtualTime)],
        };
        let expected = "Refusing to run graph 'g' on the live path: external-policy node(s) [clk (virtual time)] are provably inert at launch — nothing can fire them once `cerulion graph run` owns the runtime. Reasons: `virtual time` = under `--time-source virtual` nothing sweeps fds or calls `trigger_external`; Fix each named node: (for `virtual time`) re-run with `--time-source real`; OR drive the graph from an embedding host through the polled `GraphRuntime::step()` seam (which CAN call `trigger_external` between steps) instead of `cerulion graph run`.";
        assert_eq!(err.to_string(), expected);
    }

    /// fix 2 (exact-render): a PoisonedEntry-only refusal has NO source-class
    /// leading clause either, so its own "fix the poisoned node and restart"
    /// remedy opens the menu — never a dangling `OR`. Full byte compare.
    #[test]
    fn external_nodes_inert_poisoned_only_renders_without_dangling_or() {
        let err = TransportError::ExternalNodesInertAtLaunch {
            graph: "g".to_string(),
            nodes: vec![("n".to_string(), InertReason::PoisonedEntry)],
        };
        let expected = "Refusing to run graph 'g' on the live path: external-policy node(s) [n (poisoned node)] are provably inert at launch — nothing can fire them once `cerulion graph run` owns the runtime. Reasons: `poisoned node` = the node's entry state is poisoned, so its `ExternalSource` could not be queried; Fix each named node: fix the node whose state is poisoned (its init/tick panicked) and restart the graph; OR drive the graph from an embedding host through the polled `GraphRuntime::step()` seam (which CAN call `trigger_external` between steps) instead of `cerulion graph run`.";
        assert_eq!(err.to_string(), expected);
    }

    /// Exact render: a `FdAboveSelectLimit`-only refusal renders BOTH
    /// workarounds (enable the park, shrink the graph) as ONE menu option, with NO
    /// `--time-source real` line (it is not `VirtualTime`) and NO self-source line
    /// (it is deliberately not `is_source_class` — the fd is fine, the select watch
    /// mechanism has the ceiling). Full byte compare so a wording/menu-shape
    /// regression fails loudly, plus explicit substring pins for the two
    /// workarounds and the two absences.
    #[test]
    fn external_nodes_inert_fd_above_select_limit_renders_two_workarounds() {
        let err = TransportError::ExternalNodesInertAtLaunch {
            graph: "g".to_string(),
            nodes: vec![("cam".to_string(), InertReason::FdAboveSelectLimit)],
        };
        let expected = "Refusing to run graph 'g' on the live path: external-policy node(s) [cam (fd above select limit)] are provably inert at launch — nothing can fire them once `cerulion graph run` owns the runtime. Reasons: `fd above select limit` = the declared `ExternalSource::Fd` is a live descriptor >= FD_SETSIZE (1024); the select-backed WaitSet cannot watch it with the monitor-wait park off; Fix each named node: (for `fd above select limit`) enable the monitor-wait park — drop `--no-monitor-wait` / unset `CERULION_MONITOR_WAIT=0` (the park polls the fd via poll(2), no FD_SETSIZE ceiling); OR shrink the graph so the descriptor is minted below FD_SETSIZE; OR drive the graph from an embedding host through the polled `GraphRuntime::step()` seam (which CAN call `trigger_external` between steps) instead of `cerulion graph run`.";
        let msg = err.to_string();
        assert_eq!(msg, expected);
        // Both workarounds present.
        assert!(
            msg.contains("enable the monitor-wait park") && msg.contains("shrink the graph"),
            "both FdAboveSelectLimit workarounds must render; got: {msg}"
        );
        // NOT a source-class reason → no self-source line.
        assert!(
            !msg.contains("give it a working self-source"),
            "FdAboveSelectLimit must NOT print the self-source fix line; got: {msg}"
        );
        // NOT VirtualTime → no --time-source real line.
        assert!(
            !msg.contains("--time-source real"),
            "FdAboveSelectLimit must NOT print the --time-source real remedy; got: {msg}"
        );
        // No dangling OR opens the menu.
        assert!(
            !msg.contains("Fix each named node: OR"),
            "the fix menu must not open with a dangling `OR`; got: {msg}"
        );
    }

    /// fix 3: `InertReason::ALL` is the manually-maintained link nothing else
    /// compile-forces. This test closes that gap: `ordinal` is an EXHAUSTIVE
    /// `match` (adding a variant breaks it at compile time), we assert
    /// `ALL.len() == 8`, and every ordinal `0..8` appears EXACTLY once across
    /// `ALL` — so a new variant cannot ship without also being appended here.
    #[test]
    fn inert_reason_all_is_exhaustive_and_unique() {
        // Exhaustive match: a new InertReason variant fails to compile until it
        // gets an arm here — the compile-time forcing function for `ALL`.
        fn ordinal(r: InertReason) -> usize {
            match r {
                InertReason::HostDriven => 0,
                InertReason::NoSource => 1,
                InertReason::InvalidFd => 2,
                InertReason::FdAboveSelectLimit => 3,
                InertReason::DuplicateFd => 4,
                InertReason::PoisonedEntry => 5,
                InertReason::DoorbellFailed => 6,
                InertReason::VirtualTime => 7,
            }
        }
        const EXPECTED: usize = 8;
        assert_eq!(
            InertReason::ALL.len(),
            EXPECTED,
            "InertReason::ALL must list every variant exactly once"
        );

        let mut seen = [false; EXPECTED];
        for &r in &InertReason::ALL {
            let o = ordinal(r);
            assert!(
                !seen[o],
                "ordinal {o} ({r:?}) appears more than once in ALL"
            );
            seen[o] = true;
        }
        assert!(
            seen.iter().all(|&s| s),
            "every InertReason ordinal must appear exactly once in ALL; coverage: {seen:?}"
        );
    }

    #[test]
    fn test_allocation_failed_is_distinct_from_loan() {
        // `Loan` is iceoryx2's pool-exhaustion path; `AllocationFailed`
        // is the heap-fallback OOM path. Distinct remediations.
        let alloc_err = TransportError::AllocationFailed {
            topic: "t".into(),
            requested: 16 * 1024 * 1024,
        };
        let loan_err = TransportError::Loan {
            topic: "t".into(),
            reason: "exhausted".into(),
        };
        assert!(matches!(alloc_err, TransportError::AllocationFailed { .. }));
        assert!(matches!(loan_err, TransportError::Loan { .. }));
        assert!(!matches!(alloc_err, TransportError::Loan { .. }));
    }
}

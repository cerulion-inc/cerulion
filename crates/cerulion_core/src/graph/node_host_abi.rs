// SPDX-License-Identifier: AGPL-3.0-only
//! Host-owned native plugin context services.
//!
//! A plugin gets integer handles and borrowed wire bytes. This module retains
//! the context, transport owners, frozen input heads and diagnostic state.
//! It creates no transport session and never transfers a Rust owner to a node.
//! Raw publisher and clock services run on any thread, independently of ticks.
//! Ordinary callbacks require the admitted node-call thread; concurrency refuses
//! INVALID_STATE without changing leases. BUSY only denotes destroy with leases.
//! The loader serializes NodeApi, StateApi and ExternalApi entries on each handle.

use std::collections::HashMap;
use std::fmt::Write;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::AtomicU32;
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use crate::error::TransportError;
use crate::scheduler::{SyncHeadOp, SyncOpAnswer};
use crate::transport::plugin_lease::{LeaseArena, LeaseDirection, LeaseError, PluginWireLayout};
use crate::transport::publisher::RawShmLoan;
use crate::transport::subscriber::RawInputView;
use crate::wire::WireHeader;

use super::node::{AnyPublisher, NodeContext};
use cerulion_node_abi as abi;

/// A host service refusal; all contained Rust values remain host-local.
#[derive(Debug)]
pub enum HostServiceError {
    /// Invalid, foreign, stale, or wrong-direction handle.
    Lease(LeaseError),
    /// Invalid port descriptor or wire-frame shape.
    InvalidDescriptor,
    /// Native transport failure, distinct from an empty receive.
    Transport(TransportError),
    /// A port has an outstanding read or write accessor.
    InvalidState,
    /// Destruction must wait for caller-held native leases.
    DestroyBusy,
    /// Teardown or a fatal failure has stopped new admissions.
    Terminal,
    /// Caller buffer is short; no diagnostic/event has been consumed.
    BufferTooSmall(usize),
    /// Rust unwinding was caught inside the host context execution gate.
    Panicked,
    /// Optional service instrumentation was not wired for this context.
    MissingCapability,
    /// Canonical host-owned accumulator capture/restore failure.
    State(crate::state::StateError),
}

impl From<LeaseError> for HostServiceError {
    fn from(error: LeaseError) -> Self {
        Self::Lease(error)
    }
}

impl From<TransportError> for HostServiceError {
    fn from(error: TransportError) -> Self {
        Self::Transport(error)
    }
}

impl From<crate::state::StateError> for HostServiceError {
    fn from(error: crate::state::StateError) -> Self {
        Self::State(error)
    }
}

/// Descriptor-derived host-local binding, in node declaration order.
pub struct HostPortSpec {
    /// Field name, resolved once during initialization.
    pub name: String,
    /// Input/read or output/write.
    pub direction: LeaseDirection,
    /// Checked schema and wire layout.
    pub layout: PluginWireLayout,
    /// Explicit native input classification; ignored for outputs.
    pub read_mode: u32,
    /// Existing accumulate cap; absent on ordinary inputs and all outputs.
    pub accumulate_cap: Option<usize>,
}

struct BoundPort {
    spec: HostPortSpec,
    busy: bool,
    accumulated_borrows: usize,
    offered_this_fire: bool,
    observed_frames: Box<[bool]>,
    diagnostics: crate::transport::failure_regime_latch::FailureRegimeLatch,
}

enum NativeLease {
    Write {
        loan: RawShmLoan,
        header: WireHeader,
        spill_attempted: bool,
        spill_submitted: bool,
        invariant_counted: bool,
    },
    Read {
        _owner: Option<RawInputView<'static>>,
        accumulated_index: Option<usize>,
    },
}

/// Primitive result of a native output loan; no Rust owner crosses the ABI.
pub struct HostWriteSlot {
    /// Frame-base address of the actual native mapping.
    pub data: *mut u8,
    /// Unique bounded-arena lease token.
    pub lease: u64,
    /// Actual adaptive capacity, including the header.
    pub capacity: u32,
    /// Configured ceiling, including the header.
    pub max_capacity: u32,
}

/// Primitive result of the ordinary accounted queue/held-input path.
pub struct HostReadSlot {
    /// Immutable frame-base address, valid until release.
    pub data: *const u8,
    /// Unique bounded-arena lease token.
    pub lease: u64,
    /// Complete validated frame length.
    pub size: u32,
}

/// Observable host lease terminal counts. Rejected calls change none of them.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct HostLeaseCounters {
    /// Successful native publishes.
    pub committed: u64,
    /// Explicitly discarded outputs.
    pub discarded: u64,
    /// Released local read borrows, including held-input re-serves.
    pub released: u64,
    /// Successful copied adaptive-spill publishes.
    pub spills: u64,
}

/// Original output completion reason, kept separate from send failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostDiscardReason {
    /// User tick or input preamble did not complete.
    TickFailed,
    /// Required variable writes or nested flush did not complete.
    Incomplete,
    /// An unsent output was abandoned during teardown.
    Teardown,
    /// Original spill slot is released after its writer has been destroyed.
    SpillRelease,
    /// Replay suppression releases storage without live-discard accounting.
    ReplaySuppressed,
    /// A malformed writer/slot contract records the native invariant counter.
    Invariant,
}

/// Host-local QoS event family; the dispatcher copies only primitive facts.
pub enum HostEvent {
    /// Input flow-control/data-loss regime.
    Backpressure(crate::scheduler::BackpressureEvent),
    /// Input silence watchdog.
    ExpectWithin(crate::scheduler::ExpectWithinEvent),
    /// Output publish watchdog.
    PromiseWithin(crate::scheduler::PromiseWithinEvent),
    /// Input publisher-liveliness transition.
    Liveliness(crate::scheduler::LivelinessEvent),
}

/// Explicit event family selected by the node's generated handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostEventKind {
    Backpressure,
    ExpectWithin,
    PromiseWithin,
    Liveliness,
}

/// Observable per-port publisher/consumer counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostCounter {
    OutputDiscards,
    NotifyUndelivered,
    SendFailures,
    OverflowDrops,
    InvariantDrops,
    BlockDrainsClipped,
    DropOldest,
    BlockDeferred,
    BlockRegimes,
    Sampled,
}

/// A native context owned exclusively by the host for its entire lifetime.
pub struct HostContext {
    // Drop leases before ports: their native refcounts must return while the
    // publisher/subscriber owners still exist, even during initialization failure.
    leases: LeaseArena<NativeLease>,
    raw: Arc<raw::RawServices>,
    native: NodeContext,
    ports: Box<[BoundPort]>,
    call_thread: Option<std::thread::ThreadId>,
    environment: Arc<HashMap<String, String>>,
    counters: HostLeaseCounters,
    terminal: bool,
    diagnostic: Option<HostServiceError>,
    diagnostic_token: u64,
    scheduler_handle: Option<crate::scheduler::NodeHandle>,
    accumulators: crate::transport::accumulate::AccumulatorSet,
    snapshot_names: Vec<String>,
    diagnostics: crate::transport::failure_regime_latch::FailureRegimeLatch,
}

impl HostContext {
    /// Admit one native node/state/external entry on the current thread.
    /// The loader retains its separate NodeHandle gate and releases this context
    /// mutex before invoking foreign code. Helpers use only raw services.
    pub fn begin_node_call(&mut self) -> Result<(), HostServiceError> {
        if self.call_thread.is_some() {
            return Err(HostServiceError::InvalidState);
        }
        self.call_thread = Some(std::thread::current().id());
        Ok(())
    }
    /// End admission after the foreign entry and its local accessor drops return.
    pub fn end_node_call(&mut self) -> Result<(), HostServiceError> {
        if self.call_thread != Some(std::thread::current().id()) {
            return Err(HostServiceError::InvalidState);
        }
        self.call_thread = None;
        Ok(())
    }
    fn require_node_call(&self) -> Result<(), HostServiceError> {
        self.admit()?;
        if self.call_thread == Some(std::thread::current().id()) {
            Ok(())
        } else {
            Err(HostServiceError::InvalidState)
        }
    }
    /// Observable count of rejected ordinary callback admissions.
    pub fn admission_violations(&self) -> u64 {
        self.raw
            .violations
            .load(std::sync::atomic::Ordering::Relaxed)
    }
    /// Provision the raw publisher count before registration; lease rows per raw
    /// publisher equal this context's already configured ordinary lease budget.
    // hot-path-alloc-ok-fn: owned context builder before registration only
    pub fn with_raw_publisher_budget(mut self, budget: usize) -> Result<Self, HostServiceError> {
        if Arc::strong_count(&self.raw) != 1 {
            return Err(HostServiceError::InvalidState);
        }
        self.raw = Arc::new(raw::RawServices::new(
            self.native.transport().cloned(),
            Arc::clone(self.native.clock()),
            self.ports.len(),
            self.raw.lease_budget(),
            budget,
        )?);
        Ok(self)
    }
    /// Host runtime maintenance may inspect its own native ports without crossing C.
    pub(crate) fn native_for_runtime(&self) -> &NodeContext {
        &self.native
    }

    /// Exclusive host maintenance runs only after every accessor lease returned.
    pub(crate) fn native_mut_for_runtime(&mut self) -> Result<&mut NodeContext, HostServiceError> {
        self.admit()?;
        if self.leases.active() != 0 {
            return Err(HostServiceError::InvalidState);
        }
        Ok(&mut self.native)
    }

    /// Resolve names once and provision all lease rows before node creation.
    /// The environment is the same immutable snapshot used to build `native`.
    // hot-path-alloc-ok-fn: binding and fixed lease allocation occur at graph build
    pub fn new(
        mut native: NodeContext,
        specs: Vec<HostPortSpec>,
        environment: Arc<HashMap<String, String>>,
        lease_budget: usize,
    ) -> Result<Self, HostServiceError> {
        if specs
            .len()
            .checked_add(128)
            .is_none_or(|n| n >= u32::MAX as usize)
        {
            return Err(HostServiceError::InvalidDescriptor);
        }
        for (index, spec) in specs.iter().enumerate() {
            let prefix = spec
                .layout
                .prefix_len()
                .ok_or(HostServiceError::InvalidDescriptor)?;
            if specs[..index].iter().any(|prior| prior.name == spec.name) || spec.name.is_empty() {
                return Err(HostServiceError::InvalidDescriptor);
            }
            let exists = match spec.direction {
                LeaseDirection::Read => native.subscriber(&spec.name).is_some(),
                LeaseDirection::Write => native.publisher(&spec.name).is_some_and(|publisher| {
                    let AnyPublisher::Ipc(publisher) = publisher;
                    publisher.max_slice_len().get() >= prefix
                }),
            };
            if !exists {
                return Err(HostServiceError::InvalidDescriptor);
            }
            if spec.direction == LeaseDirection::Read
                && !matches!(
                    spec.read_mode,
                    abi::read_mode::LATEST | abi::read_mode::ONE | abi::read_mode::HELD
                )
            {
                return Err(HostServiceError::InvalidDescriptor);
            }
            if spec.direction == LeaseDirection::Write && spec.accumulate_cap.is_some() {
                return Err(HostServiceError::InvalidDescriptor);
            }
        }
        let mut accumulators = crate::transport::accumulate::AccumulatorSet::new();
        for spec in &specs {
            if let Some(cap) = spec.accumulate_cap {
                accumulators.register_raw(&mut native, &spec.name, cap, spec.layout)?;
            }
        }
        let raw = Arc::new(raw::RawServices::new(
            native.transport().cloned(),
            Arc::clone(native.clock()),
            specs.len(),
            lease_budget,
            128,
        )?);
        Ok(Self {
            leases: LeaseArena::new(lease_budget)?,
            native,
            ports: specs
                .into_iter()
                .map(|spec| {
                    let observed_frames =
                        vec![false; spec.accumulate_cap.unwrap_or(0)].into_boxed_slice();
                    BoundPort {
                        spec,
                        busy: false,
                        accumulated_borrows: 0,
                        offered_this_fire: false,
                        observed_frames,
                        diagnostics:
                            crate::transport::failure_regime_latch::FailureRegimeLatch::new(),
                    }
                })
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            raw,
            call_thread: None,
            environment,
            counters: HostLeaseCounters::default(),
            terminal: false,
            diagnostic: None,
            diagnostic_token: 0,
            scheduler_handle: None,
            accumulators,
            snapshot_names: Vec::new(),
            diagnostics: crate::transport::failure_regime_latch::FailureRegimeLatch::new(),
        })
    }

    /// Resolve a descriptor index to a context-scoped port handle.
    pub fn bind_port(
        &self,
        context: u64,
        index: u32,
        schema_hash: u64,
        direction: LeaseDirection,
    ) -> Result<u64, HostServiceError> {
        self.admit()?;
        let port = self
            .ports
            .get(index as usize)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        if context == 0
            || index == u32::MAX
            || port.spec.layout.schema_hash != schema_hash
            || port.spec.direction != direction
        {
            return Err(HostServiceError::InvalidDescriptor);
        }
        // Context generations occupy the upper half; the descriptor index is
        // the lower half. The dispatcher validates the full ContextHandle first.
        Ok((context & 0xffff_ffff_0000_0000) | (u64::from(index) + 1))
    }

    /// Borrow host runtime services for scheduler maintenance. This hook
    /// never crosses the node ABI and excludes every outstanding accessor.
    /// Maintenance must preserve bound port owners and topology; their names
    /// and borrowed topic storage remain fixed for the context lifetime.
    pub fn with_native<R>(
        &mut self,
        operation: impl FnOnce(&mut NodeContext) -> R,
    ) -> Result<R, HostServiceError> {
        self.admit()?;
        if self.leases.active() != 0 {
            return Err(HostServiceError::InvalidState);
        }
        Ok(operation(&mut self.native))
    }

    /// Return cold binding facts using direction-scoped declaration order.
    pub fn binding(
        &self,
        context: u64,
        ordinal: u32,
        direction: LeaseDirection,
    ) -> Result<abi::PortBindingV1, HostServiceError> {
        self.admit()?;
        let (index, bound) = self
            .ports
            .iter()
            .enumerate()
            .filter(|(_, bound)| bound.spec.direction == direction)
            .nth(ordinal as usize)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let handle = self.bind_port(
            context,
            index as u32,
            bound.spec.layout.schema_hash,
            direction,
        )?;
        let (topic, configured_capacity) = match direction {
            LeaseDirection::Read => (
                self.native
                    .subscriber(&bound.spec.name)
                    .ok_or(HostServiceError::InvalidDescriptor)?
                    .topic(),
                0,
            ),
            LeaseDirection::Write => {
                let Some(AnyPublisher::Ipc(publisher)) = self.native.publisher(&bound.spec.name)
                else {
                    return Err(HostServiceError::InvalidDescriptor);
                };
                (publisher.topic(), publisher.max_slice_len().get())
            }
        };
        Ok(abi::PortBindingV1 {
            port: abi::PortHandle { value: handle },
            topic: abi::Bytes {
                ptr: topic.as_ptr(),
                len: topic.len() as u64,
            },
            configured_capacity,
            read_mode: bound.spec.read_mode,
        })
    }

    /// Start one node invocation after the preceding invocation's accessors
    /// returned. The loader calls this once before each user tick.
    pub fn begin_fire(&mut self) -> Result<(), HostServiceError> {
        self.admit()?;
        if self.leases.active() != 0 {
            return Err(HostServiceError::InvalidState);
        }
        for bound in &mut self.ports {
            bound.offered_this_fire = false;
            bound.observed_frames.fill(false);
        }
        Ok(())
    }

    /// Resolve the existing snapshot name list once during cold node setup.
    // hot-path-alloc-ok-fn: JSON names are copied only at node initialization
    pub fn configure_snapshots(&mut self, bytes: &[u8]) -> Result<(), HostServiceError> {
        self.admit()?;
        let names: Vec<String> =
            serde_json::from_slice(bytes).map_err(|_| HostServiceError::InvalidDescriptor)?;
        for (index, name) in names.iter().enumerate() {
            if names[..index].contains(name)
                || !self.ports.iter().any(|bound| {
                    bound.spec.direction == LeaseDirection::Read && bound.spec.name == *name
                })
            {
                return Err(HostServiceError::InvalidDescriptor);
            }
        }
        self.snapshot_names = names;
        Ok(())
    }

    /// Snapshot the explicitly configured non-trigger inputs, including native
    /// accumulators, after checking all borrows before the first queue effect.
    pub fn snapshot_configured(&mut self) -> Result<(), HostServiceError> {
        self.admit()?;
        for name in &self.snapshot_names {
            let index = self
                .ports
                .iter()
                .position(|bound| bound.spec.name == *name)
                .ok_or(HostServiceError::InvalidDescriptor)?;
            self.unborrowed(index)?;
        }
        self.accumulators
            .snapshot(&mut self.native, &self.snapshot_names);
        Ok(())
    }

    /// Offer an accumulated batch once at the generated pre-body site; repeated
    /// queries in this invocation retain the same frames and loss observations.
    pub fn accumulate_info(
        &mut self,
        context: u64,
        port: u64,
    ) -> Result<abi::AccumulateInfoV1, HostServiceError> {
        let index = self.accumulator_index(context, port)?;
        if self.ports[index].busy {
            return Err(HostServiceError::InvalidState);
        }
        let bound = &mut self.ports[index];
        if !bound.offered_this_fire {
            self.accumulators.mark_served(&bound.spec.name);
            bound.offered_this_fire = true;
        }
        let frames = self
            .accumulators
            .frames_raw(&bound.spec.name)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let (dropped, dropped_total) = self
            .accumulators
            .dropped_raw(&bound.spec.name)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        Ok(abi::AccumulateInfoV1 {
            count: frames.len() as u32,
            reserved: 0,
            dropped,
            dropped_total,
        })
    }

    /// Borrow a repeatable indexed host frame. Multiple immutable frame leases
    /// are permitted up to the preallocated arena budget; snapshots are excluded.
    pub fn accumulated_frame(
        &mut self,
        context: u64,
        port: u64,
        frame_index: u32,
    ) -> Result<HostReadSlot, HostServiceError> {
        let index = self.accumulator_index(context, port)?;
        let bound = &self.ports[index];
        if bound.busy || !bound.offered_this_fire {
            return Err(HostServiceError::InvalidState);
        }
        let next_borrows = bound
            .accumulated_borrows
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        let frame = self
            .accumulators
            .frames_raw(&bound.spec.name)
            .ok_or(HostServiceError::InvalidDescriptor)?
            .iter()
            .nth(frame_index as usize)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let token = self.leases.claim(port, LeaseDirection::Read)?;
        let slot = HostReadSlot {
            data: frame.as_ptr(),
            size: frame.len() as u32,
            lease: token,
        };
        self.leases.insert(
            token,
            NativeLease::Read {
                _owner: None,
                accumulated_index: Some(frame_index as usize),
            },
        )?;
        self.ports[index].accumulated_borrows = next_borrows;
        Ok(slot)
    }

    /// Observe each distinct accumulated frame once. Native gather already
    /// owns queue/read-outcome staging; iteration never re-stages that pop.
    pub fn mark_accumulated_served(
        &mut self,
        context: u64,
        token: u64,
    ) -> Result<(), HostServiceError> {
        let (port, direction) = self.leases.owner(token)?;
        if direction != LeaseDirection::Read {
            return Err(LeaseError::WrongOwner.into());
        }
        let index = self.accumulator_index(context, port)?;
        let NativeLease::Read {
            accumulated_index: Some(frame_index),
            ..
        } = self.leases.get(token, port, LeaseDirection::Read)?
        else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        let observed = self.ports[index]
            .observed_frames
            .get_mut(*frame_index)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        *observed = true;
        Ok(())
    }

    /// Capture the existing canonical per-input record synchronously.
    pub fn capture_accumulator(
        &self,
        context: u64,
        port: u64,
        sink: &mut dyn crate::state::StateSink,
    ) -> Result<(), HostServiceError> {
        let index = self.accumulator_index(context, port)?;
        self.accumulators
            .capture_raw_port(&self.ports[index].spec.name, sink)?;
        Ok(())
    }

    /// Validate and restore one canonical input record before any mutation.
    pub fn restore_accumulator(
        &mut self,
        context: u64,
        port: u64,
        bytes: &[u8],
    ) -> Result<(), HostServiceError> {
        let index = self.accumulator_index(context, port)?;
        self.unborrowed(index)?;
        self.accumulators
            .restore_raw_port(&self.ports[index].spec.name, bytes)?;
        self.ports[index].offered_this_fire = false;
        Ok(())
    }

    fn accumulator_index(&self, context: u64, port: u64) -> Result<usize, HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Read)?;
        if self.ports[index].spec.accumulate_cap.is_none() {
            return Err(HostServiceError::InvalidDescriptor);
        }
        Ok(index)
    }

    /// Loan the real adaptive slot after checking admission and borrow budget.
    pub fn loan_output(
        &mut self,
        context: u64,
        port: u64,
    ) -> Result<HostWriteSlot, HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Write)?;
        self.unborrowed(index)?;
        let token = self.leases.claim(port, LeaseDirection::Write)?;
        let bound = &self.ports[index];
        let layout = bound.spec.layout;
        let Some(AnyPublisher::Ipc(publisher)) = self.native.publisher_mut(&bound.spec.name) else {
            self.leases.cancel(token)?;
            return Err(HostServiceError::InvalidDescriptor);
        };
        let mut loan = match publisher.loan_plugin_frame(layout) {
            Ok(loan) => loan,
            Err(error) => {
                self.leases.cancel(token)?;
                return Err(error.into());
            }
        };
        let max_capacity = publisher.max_slice_len().get();
        let bytes = loan.bytes_mut();
        if bytes.as_ptr().align_offset(layout.alignment as usize) != 0 {
            self.leases.cancel(token)?;
            return Err(HostServiceError::InvalidDescriptor);
        }
        let slot = HostWriteSlot {
            data: bytes.as_mut_ptr(),
            lease: token,
            capacity: bytes.len() as u32,
            max_capacity,
        };
        let Some(header) = WireHeader::read_from_buf(bytes) else {
            self.leases.cancel(token)?;
            return Err(HostServiceError::InvalidDescriptor);
        };
        self.leases.insert(
            token,
            NativeLease::Write {
                loan,
                header,
                spill_attempted: false,
                spill_submitted: false,
                invariant_counted: false,
            },
        )?;
        self.ports[index].busy = true;
        Ok(slot)
    }

    /// Validate, stamp and commit without copying the steady-state payload.
    pub fn commit_output(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
        total_size: u32,
    ) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Write)?;
        // Validate through the loan owner's slice, never a plugin-provided
        // address. A normal finish consumes the lease even when malformed;
        // its writer has already returned before this terminal callback.
        let layout = self.ports[index].spec.layout;
        let suppressed = self.output_publisher(index)?.replay_suppress_active();
        let NativeLease::Write {
            loan,
            spill_submitted: false,
            ..
        } = self.leases.get_mut(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        if !suppressed && !layout.accepts(loan.bytes_mut(), total_size) {
            self.discard_output(context, port, token, HostDiscardReason::Invariant)?;
            return Err(HostServiceError::InvalidDescriptor);
        }
        let NativeLease::Write { loan, header, .. } =
            self.leases.take(token, port, LeaseDirection::Write)?
        else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        self.ports[index].busy = false;
        // The protected original header is host-authoritative, including the
        // loan timestamp, provisional sequence and current native step stamp.
        if suppressed {
            return Ok(());
        }
        let next_count = self
            .counters
            .committed
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        let publisher = self.output_publisher(index)?;
        publisher.commit_plugin_frame(loan, header, total_size)?;
        self.counters.committed = next_count;
        Ok(())
    }

    /// Explicit adaptive spill: node bytes are copied synchronously by host.
    /// The original loan remains live until its writer is destroyed and the
    /// caller returns it with SpillRelease. Its timestamp survives the copy.
    pub fn submit_spill(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
        payload: &[u8],
        size: u32,
    ) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Write)?;
        let suppressed = self.output_publisher(index)?.replay_suppress_active();
        let NativeLease::Write {
            header,
            loan,
            spill_submitted,
            ..
        } = self.leases.get_mut(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        if *spill_submitted {
            return Err(LeaseError::InvalidHandle.into());
        }
        *spill_submitted = true;
        let header = *header;
        let capacity = loan.bytes_mut().len();
        if suppressed {
            return Ok(());
        }
        let layout = self.ports[index].spec.layout;
        let max_capacity = self.output_publisher(index)?.max_slice_len().get();
        if payload.len().checked_add(WireHeader::SIZE) != Some(size as usize)
            || size as usize <= capacity
            || size > max_capacity
            || !layout.accepts_payload(payload, size)
        {
            self.count_spill_invariant(index, port, token)?;
            return Err(HostServiceError::InvalidDescriptor);
        }
        let NativeLease::Write {
            spill_attempted, ..
        } = self.leases.get_mut(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        *spill_attempted = true;
        let next_count = self
            .counters
            .committed
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        let next_spills = self
            .counters
            .spills
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        self.output_publisher(index)?
            .commit_plugin_spill(header, payload)?;
        self.counters.committed = next_count;
        self.counters.spills = next_spills;
        Ok(())
    }

    fn count_spill_invariant(
        &mut self,
        index: usize,
        port: u64,
        token: u64,
    ) -> Result<(), HostServiceError> {
        let NativeLease::Write {
            invariant_counted, ..
        } = self.leases.get_mut(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        if !*invariant_counted {
            *invariant_counted = true;
            self.output_publisher(index)?
                .record_invariant_violation_drop();
            self.raw
                .invariants
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            self.counters.discarded = self.counters.discarded.saturating_add(1);
        }
        Ok(())
    }
    fn reject_spill_span(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
    ) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Write)?;
        let NativeLease::Write {
            spill_submitted, ..
        } = self.leases.get_mut(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        if *spill_submitted {
            return Err(LeaseError::InvalidHandle.into());
        }
        *spill_submitted = true;
        if !self.output_publisher(index)?.replay_suppress_active() {
            self.count_spill_invariant(index, port, token)?;
        }
        Ok(())
    }

    /// Discard exactly once without consuming a sequence or notifying readers.
    pub fn discard_output(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
        reason: HostDiscardReason,
    ) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Write)?;
        let NativeLease::Write {
            spill_submitted,
            spill_attempted,
            invariant_counted,
            ..
        } = self.leases.get(token, port, LeaseDirection::Write)?
        else {
            return Err(LeaseError::InvalidHandle.into());
        };
        if *spill_submitted && reason != HostDiscardReason::SpillRelease {
            return Err(LeaseError::InvalidHandle.into());
        }
        let missing_attempt = reason == HostDiscardReason::SpillRelease && !*spill_attempted;
        let submitted = *spill_submitted;
        let counted = *invariant_counted;
        let suppressed = self.output_publisher(index)?.replay_suppress_active();
        let owner = self.leases.take(token, port, LeaseDirection::Write)?;
        self.ports[index].busy = false;
        drop(owner);
        if suppressed || reason == HostDiscardReason::ReplaySuppressed {
            return Ok(());
        }
        if reason == HostDiscardReason::SpillRelease && (!missing_attempt || counted) {
            return Ok(());
        }
        let publisher = self.output_publisher(index)?;
        if reason == HostDiscardReason::TickFailed {
            publisher.bump_discard_signal();
        }
        if reason == HostDiscardReason::Incomplete {
            use crate::transport::output_discard_latch::DiscardLogLevel;
            match publisher.record_output_discard() {
                DiscardLogLevel::Error => tracing::error!(
                    topic = publisher.topic(),
                    "plugin output incomplete; frame discarded"
                ),
                DiscardLogLevel::Debug { suppressed } => tracing::debug!(
                    topic = publisher.topic(),
                    suppressed_count = suppressed,
                    "plugin incomplete-output discard repeated"
                ),
            }
        }
        if reason == HostDiscardReason::Invariant || missing_attempt {
            publisher.record_invariant_violation_drop();
            self.raw
                .invariants
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
        self.counters.discarded = self
            .counters
            .discarded
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        if missing_attempt && !submitted {
            return Err(HostServiceError::InvalidDescriptor);
        }
        Ok(())
    }

    /// Serve the existing queue/frozen/held selection and retain its native owner.
    pub fn receive_input(
        &mut self,
        context: u64,
        port: u64,
    ) -> Result<Option<HostReadSlot>, HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Read)?;
        if self.ports[index].spec.accumulate_cap.is_some() {
            return Err(HostServiceError::InvalidDescriptor);
        }
        self.unborrowed(index)?;
        let token = self.leases.claim(port, LeaseDirection::Read)?;
        let bound = &self.ports[index];
        let layout = bound.spec.layout;
        let Some(subscriber) = self.native.subscriber_mut(&bound.spec.name) else {
            self.leases.cancel(token)?;
            return Err(HostServiceError::InvalidDescriptor);
        };
        let super::node::AnySubscriber::Ipc(subscriber) = subscriber;
        let result = subscriber.view_plugin_frame(layout);
        let view = match result {
            Ok(Some(view)) => view,
            Ok(None) => {
                self.leases.cancel(token)?;
                return Ok(None);
            }
            Err(error) => {
                self.leases.cancel(token)?;
                return Err(error.into());
            }
        };
        let size = u32::try_from(view.len()).map_err(|_| HostServiceError::InvalidDescriptor)?;
        if view.as_ptr().align_offset(layout.alignment as usize) != 0
            || !layout.accepts(&view, size)
        {
            self.leases.cancel(token)?;
            return Err(HostServiceError::InvalidDescriptor);
        }
        let slot = HostReadSlot {
            data: view.as_ptr(),
            lease: token,
            size,
        };
        // No forged static borrow: the subscriber owns the held sample;
        // busy excludes every mutation of that port until lease release.
        let owner = view.into_owned().ok();
        self.leases.insert(
            token,
            NativeLease::Read {
                _owner: owner,
                accumulated_index: None,
            },
        )?;
        self.ports[index].busy = true;
        Ok(Some(slot))
    }

    /// End the node accessor borrow, keeping any host-held snapshot intact.
    pub fn release_input(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
    ) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Read)?;
        let owner = self.leases.take(token, port, LeaseDirection::Read)?;
        if matches!(
            owner,
            NativeLease::Read {
                accumulated_index: Some(_),
                ..
            }
        ) {
            self.ports[index].accumulated_borrows -= 1;
        } else {
            self.ports[index].busy = false;
        }
        drop(owner);
        let next_count = self
            .counters
            .released
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        self.counters.released = next_count;
        Ok(())
    }

    /// Scheduler-owned head operations use the same native read decisions.
    pub fn input_head(
        &mut self,
        context: u64,
        port: u64,
        operation: SyncHeadOp,
    ) -> Result<SyncOpAnswer, HostServiceError> {
        let index = self.port_index(context, port, LeaseDirection::Read)?;
        self.unborrowed(index)?;
        Ok(self
            .native
            .sync_head_op(&self.ports[index].spec.name, operation))
    }

    /// Freeze inputs atomically with respect to outstanding node accessors.
    pub fn snapshot_inputs(
        &mut self,
        context: u64,
        inputs: &[u64],
    ) -> Result<(), HostServiceError> {
        self.admit()?;
        for &port in inputs {
            let index = self.port_index(context, port, LeaseDirection::Read)?;
            self.unborrowed(index)?;
        }
        for &port in inputs {
            let index = self.port_index(context, port, LeaseDirection::Read)?;
            self.accumulators.snapshot(
                &mut self.native,
                std::slice::from_ref(&self.ports[index].spec.name),
            );
        }
        Ok(())
    }

    /// Host pumps its own native publisher history; no node-owned port exists.
    pub fn pump_history(&mut self) -> Result<(), HostServiceError> {
        self.admit()?;
        if self
            .ports
            .iter()
            .any(|port| port.spec.direction == LeaseDirection::Write && port.busy)
        {
            return Err(HostServiceError::InvalidState);
        }
        self.native.pump_history();
        Ok(())
    }

    /// Active clock used by both scheduler and node, including VirtualClock.
    pub fn clock_now(&self) -> Result<u64, HostServiceError> {
        self.admit()?;
        Ok(self.native.clock().now_ns())
    }

    /// Explicit virtual-source presence; never reads wall time as a fallback.
    pub fn clock_virtual(&self) -> Result<Option<u64>, HostServiceError> {
        self.admit()?;
        Ok(self.native.clock().virt_ns())
    }

    /// Explicit external-source presence, from the same active clock.
    pub fn clock_external(&self) -> Result<Option<u64>, HostServiceError> {
        self.admit()?;
        Ok(self.native.clock().ext_ns())
    }

    /// Existing explicit author escape; never used for scheduling decisions.
    pub fn clock_real(&self) -> Result<u64, HostServiceError> {
        self.admit()?;
        Ok(crate::clock::real_ns())
    }

    /// Borrow frozen environment text without allocation or a live-env fallback.
    pub fn environment_value(&self, key: &str) -> Result<Option<&str>, HostServiceError> {
        self.admit()?;
        Ok(self.environment.get(key).map(String::as_str))
    }

    /// Cooperative graph shutdown remains host-owned and observable.
    pub fn request_shutdown(&self) -> Result<(), HostServiceError> {
        self.admit()?;
        self.native.request_shutdown();
        Ok(())
    }

    /// Wire the scheduler's existing observable counters at graph build.
    pub fn set_scheduler_handle(&mut self, handle: crate::scheduler::NodeHandle) {
        self.scheduler_handle = Some(handle);
    }

    /// Poll the native context's exact queued event family, without copying a
    /// Rust event or name allocation across the boundary.
    pub fn poll_event(
        &mut self,
        context: u64,
        port: u64,
        kind: HostEventKind,
    ) -> Result<Option<HostEvent>, HostServiceError> {
        let direction = if kind == HostEventKind::PromiseWithin {
            LeaseDirection::Write
        } else {
            LeaseDirection::Read
        };
        let index = self.port_index(context, port, direction)?;
        let name = &self.ports[index].spec.name;
        Ok(match kind {
            HostEventKind::Backpressure => self
                .native
                .take_backpressure_event(name)
                .map(HostEvent::Backpressure),
            HostEventKind::ExpectWithin => self
                .native
                .take_expect_within_event(name)
                .map(HostEvent::ExpectWithin),
            HostEventKind::PromiseWithin => self
                .native
                .take_promise_within_event(name)
                .map(HostEvent::PromiseWithin),
            HostEventKind::Liveliness => self
                .native
                .take_liveliness_event(name)
                .map(HostEvent::Liveliness),
        })
    }

    /// Read the existing native counter rather than inventing a log-derived
    /// observation. Absent scheduler instrumentation is an explicit refusal.
    pub fn port_counter(
        &self,
        context: u64,
        port: u64,
        counter: HostCounter,
    ) -> Result<u64, HostServiceError> {
        let output = matches!(
            counter,
            HostCounter::OutputDiscards
                | HostCounter::NotifyUndelivered
                | HostCounter::SendFailures
                | HostCounter::OverflowDrops
                | HostCounter::InvariantDrops
        );
        let index = self.port_index(
            context,
            port,
            if output {
                LeaseDirection::Write
            } else {
                LeaseDirection::Read
            },
        )?;
        let name = &self.ports[index].spec.name;
        if output {
            let Some(AnyPublisher::Ipc(publisher)) = self.native.publisher(name) else {
                return Err(HostServiceError::InvalidDescriptor);
            };
            return Ok(match counter {
                HostCounter::OutputDiscards => publisher.output_discard_count(),
                HostCounter::NotifyUndelivered => publisher.notify_undelivered_count(),
                HostCounter::SendFailures => publisher.frames_dropped_send_fail(),
                HostCounter::OverflowDrops => publisher.frames_dropped_overflow(),
                HostCounter::InvariantDrops => publisher.frames_dropped_invariant_violation(),
                _ => return Err(HostServiceError::InvalidDescriptor),
            });
        }
        let Some(super::node::AnySubscriber::Ipc(subscriber)) = self.native.subscriber(name) else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        if counter == HostCounter::BlockDrainsClipped {
            return Ok(subscriber.block_drains_clipped());
        }
        let handle = self
            .scheduler_handle
            .as_ref()
            .ok_or(HostServiceError::MissingCapability)?;
        Ok(match counter {
            HostCounter::DropOldest => handle.backpressure_drop_oldest_count(name),
            HostCounter::BlockDeferred => handle.backpressure_block_fires_deferred_count(name),
            HostCounter::BlockRegimes => handle.backpressure_block_defer_regimes_count(name),
            HostCounter::Sampled => handle.backpressure_sampled_count(name),
            _ => return Err(HostServiceError::InvalidDescriptor),
        })
    }

    /// Current terminal observations, independent of logs.
    pub fn counters(&self) -> HostLeaseCounters {
        HostLeaseCounters {
            committed: self.counters.committed.saturating_add(
                self.raw
                    .committed
                    .load(std::sync::atomic::Ordering::Relaxed),
            ),
            discarded: self.counters.discarded.saturating_add(
                self.raw
                    .discarded
                    .load(std::sync::atomic::Ordering::Relaxed),
            ),
            ..self.counters
        }
    }

    /// Invariant failures, including leaked input and output leases reclaimed at destroy.
    pub fn invariant_failures(&self) -> u64 {
        self.raw
            .invariants
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Node-wide (zero port) or per-port diagnostic failures, observable
    /// independently of whether the tracing subscriber records a log line.
    pub fn diagnostic_failures(&self, context: u64, port: u64) -> Result<u64, HostServiceError> {
        self.admit()?;
        if port == 0 {
            return Ok(self.diagnostics.total_failures());
        }
        let index = (port as u32)
            .checked_sub(1)
            .ok_or(LeaseError::InvalidHandle)? as usize;
        let bound = self.ports.get(index).ok_or(LeaseError::InvalidHandle)?;
        self.port_index(context, port, bound.spec.direction)?;
        Ok(bound.diagnostics.total_failures())
    }

    /// Number of active/reserved native owners.
    pub fn outstanding_leases(&self) -> usize {
        self.leases.active() + self.raw.active.load(std::sync::atomic::Ordering::Acquire)
    }

    /// Copy the complete owner-local diagnostic into caller storage.
    /// A size query/short buffer keeps the diagnostic and never truncates text.
    pub fn copy_diagnostic(
        &self,
        token: u64,
        target: &mut [u8],
    ) -> Result<usize, HostServiceError> {
        if token == 0 || token != self.diagnostic_token {
            return Err(LeaseError::InvalidHandle.into());
        }
        let error = self.diagnostic.as_ref().ok_or(LeaseError::InvalidHandle)?;
        copy_diagnostic_bytes(error, target)
    }

    /// Stop admissions and retire all owners while native ports remain live.
    pub fn revoke(&mut self) {
        self.terminal = true;
        self.call_thread = None;
        let native = &mut self.native;
        let ports = &self.ports;
        let counters = &mut self.counters;
        let raw = &self.raw;
        self.leases.revoke_with(|port, direction, owner| {
            let already_counted = matches!(
                &owner,
                NativeLease::Write {
                    invariant_counted: true,
                    ..
                }
            );
            if !already_counted {
                raw.invariants
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if direction == LeaseDirection::Write {
                    let index = (port as u32 - 1) as usize;
                    if let Some(bound) = ports.get(index) {
                        if let Some(AnyPublisher::Ipc(publisher)) =
                            native.publisher_mut(&bound.spec.name)
                        {
                            publisher.record_invariant_violation_drop();
                        }
                    }
                    counters.discarded = counters.discarded.saturating_add(1);
                }
            }
            drop(owner);
        });
        self.raw.revoke();
        for port in &mut self.ports {
            port.busy = false;
            port.accumulated_borrows = 0;
        }
    }

    fn admit(&self) -> Result<(), HostServiceError> {
        if self.terminal {
            Err(HostServiceError::Terminal)
        } else {
            Ok(())
        }
    }

    fn unborrowed(&self, index: usize) -> Result<(), HostServiceError> {
        if self.ports[index].busy || self.ports[index].accumulated_borrows != 0 {
            Err(HostServiceError::InvalidState)
        } else {
            Ok(())
        }
    }

    fn port_index(
        &self,
        context: u64,
        port: u64,
        direction: LeaseDirection,
    ) -> Result<usize, HostServiceError> {
        self.admit()?;
        if context == 0 || (context >> 32) != (port >> 32) {
            return Err(LeaseError::InvalidHandle.into());
        }
        let index = (port as u32)
            .checked_sub(1)
            .ok_or(LeaseError::InvalidHandle)? as usize;
        let bound = self.ports.get(index).ok_or(LeaseError::InvalidHandle)?;
        if bound.spec.direction != direction {
            return Err(LeaseError::WrongOwner.into());
        }
        Ok(index)
    }

    fn output_publisher(
        &mut self,
        index: usize,
    ) -> Result<&mut crate::transport::publisher::CerulionPublisher, HostServiceError> {
        let Some(AnyPublisher::Ipc(publisher)) =
            self.native.publisher_mut(&self.ports[index].spec.name)
        else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        Ok(publisher)
    }
}

struct DiagnosticWriter<'a> {
    target: Option<&'a mut [u8]>,
    written: usize,
}

impl Write for DiagnosticWriter<'_> {
    fn write_str(&mut self, text: &str) -> std::fmt::Result {
        let end = self
            .written
            .checked_add(text.len())
            .ok_or(std::fmt::Error)?;
        if let Some(target) = &mut self.target {
            target
                .get_mut(self.written..end)
                .ok_or(std::fmt::Error)?
                .copy_from_slice(text.as_bytes());
        }
        self.written = end;
        Ok(())
    }
}

fn write_diagnostic(
    target: &mut DiagnosticWriter<'_>,
    error: &HostServiceError,
) -> std::fmt::Result {
    match error {
        HostServiceError::Transport(error) => write!(target, "{error}"),
        HostServiceError::State(error) => write!(target, "{error}"),
        other => write!(target, "{other:?}"),
    }
}

fn copy_diagnostic_bytes(
    error: &HostServiceError,
    target: &mut [u8],
) -> Result<usize, HostServiceError> {
    let mut measure = DiagnosticWriter {
        target: None,
        written: 0,
    };
    write_diagnostic(&mut measure, error).map_err(|_| HostServiceError::InvalidDescriptor)?;
    if target.len() < measure.written {
        return Err(HostServiceError::BufferTooSmall(measure.written));
    }
    let mut writer = DiagnosticWriter {
        target: Some(target),
        written: 0,
    };
    write_diagnostic(&mut writer, error).map_err(|_| HostServiceError::InvalidDescriptor)?;
    Ok(writer.written)
}

struct ContextRow {
    generation: u32,
    owner: Option<Arc<Mutex<HostContext>>>,
    raw: Option<Arc<raw::RawServices>>,
}

static CONTEXTS: OnceLock<RwLock<Vec<ContextRow>>> = OnceLock::new();
static NEXT_CONTEXT: AtomicU32 = AtomicU32::new(1);

/// Host-local callback failure. Its token names an owner-local diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostCallFailure {
    /// Frozen boundary status code from DESIGN.md.
    pub status: u32,
    /// Existing typed node error kind; zero when this is a host refusal.
    pub kind: u32,
    /// Context-local diagnostic token, never an address or foreign allocation.
    pub error_token: u64,
}

/// Register a context at graph build; its raw Rust allocation stays host-local.
// hot-path-alloc-ok-fn: graph construction only; callbacks never grow this registry
pub fn register_context(context: HostContext) -> Result<u64, HostServiceError> {
    let generation = crate::transport::plugin_lease::mint_generation(&NEXT_CONTEXT)?;
    let mut rows = CONTEXTS
        .get_or_init(|| RwLock::new(Vec::new()))
        .write()
        .map_err(|_| HostServiceError::Panicked)?;
    let raw = Some(Arc::clone(&context.raw));
    let owner = Some(Arc::new(Mutex::new(context)));
    let index = if let Some(index) = rows.iter().position(|row| row.owner.is_none()) {
        rows[index] = ContextRow {
            generation,
            owner,
            raw,
        };
        index
    } else {
        if rows.len() >= u32::MAX as usize {
            return Err(LeaseError::InvalidBudget.into());
        }
        let index = rows.len();
        rows.push(ContextRow {
            generation,
            owner,
            raw,
        });
        index
    };
    Ok((u64::from(generation) << 32) | (index as u64 + 1))
}

fn context_owner(handle: u64) -> Result<Arc<Mutex<HostContext>>, HostServiceError> {
    let index = (handle as u32)
        .checked_sub(1)
        .ok_or(LeaseError::InvalidHandle)? as usize;
    let rows = CONTEXTS
        .get()
        .ok_or(LeaseError::InvalidHandle)?
        .read()
        .map_err(|_| HostServiceError::Panicked)?;
    let row = rows.get(index).ok_or(LeaseError::InvalidHandle)?;
    if row.generation == 0 || row.generation != (handle >> 32) as u32 {
        return Err(LeaseError::InvalidHandle.into());
    }
    row.owner
        .as_ref()
        .map(Arc::clone)
        .ok_or_else(|| LeaseError::InvalidHandle.into())
}

fn raw_owner(handle: u64) -> Result<Arc<raw::RawServices>, HostServiceError> {
    let index = (handle as u32)
        .checked_sub(1)
        .ok_or(LeaseError::InvalidHandle)? as usize;
    let rows = CONTEXTS
        .get()
        .ok_or(LeaseError::InvalidHandle)?
        .read()
        .map_err(|_| HostServiceError::Panicked)?;
    let row = rows.get(index).ok_or(LeaseError::InvalidHandle)?;
    if row.generation == 0 || row.generation != (handle >> 32) as u32 {
        return Err(LeaseError::InvalidHandle.into());
    }
    row.raw
        .as_ref()
        .map(Arc::clone)
        .ok_or_else(|| LeaseError::InvalidHandle.into())
}

/// Execute a host operation inside its context gate; independent contexts
/// run concurrently. Concurrent/reentrant calls refuse INVALID_STATE without waiting.
/// Unwinds are caught before the mutex guard can poison. Never hold this gate
/// across NodeApi execution, which synchronously calls back into HOST_API.
pub fn with_context<R>(
    handle: u64,
    operation: impl FnOnce(&mut HostContext) -> Result<R, HostServiceError>,
) -> Result<R, HostCallFailure> {
    let owner = context_owner(handle).map_err(|error| classify_failure(&error, 0))?;
    let services = raw_owner(handle).map_err(|error| classify_failure(&error, 0))?;
    let mut context = owner.try_lock().map_err(|error| HostCallFailure {
        status: match error {
            std::sync::TryLockError::WouldBlock => {
                services
                    .violations
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                abi::status_code::INVALID_STATE
            }
            std::sync::TryLockError::Poisoned(_) => abi::status_code::PANICKED,
        },
        kind: 0,
        error_token: 0,
    })?;
    let outcome = catch_unwind(AssertUnwindSafe(|| operation(&mut context)));
    let error = match outcome {
        Ok(Ok(value)) => return Ok(value),
        Ok(Err(error)) => error,
        Err(payload) => {
            // The payload and its destructor remain host-owned. Destruction
            // cannot unwind across C; abort/double-panic profile limits remain.
            context.terminal = true;
            contain_panic_payload(payload);
            // Stop admission but preserve every live borrowed mapping until
            // the node invocation has returned and the loader revokes it.
            HostServiceError::Panicked
        }
    };
    if matches!(error, HostServiceError::InvalidState) {
        services
            .violations
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    let token = match context
        .diagnostic_token
        .checked_add(1)
        .filter(|token| *token < (1 << 63))
    {
        Some(token) => {
            context.diagnostic_token = token;
            token
        }
        None => {
            context.terminal = true;
            0
        }
    };
    let failure = classify_failure(&error, token);
    context.diagnostic = Some(error);
    Err(failure)
}

fn contain_panic_payload(payload: Box<dyn std::any::Any + Send>) {
    if let Err(secondary) = catch_unwind(AssertUnwindSafe(|| drop(payload))) {
        // A fatal host panic already stops admissions. A payload destructor
        // that itself panics must not unwind across C or poison the context.
        // Keep the secondary payload host-local without running another
        // potentially panicking destructor; at most one per terminal context.
        std::mem::forget(secondary);
    }
}

fn classify_failure(error: &HostServiceError, error_token: u64) -> HostCallFailure {
    let status = match error {
        HostServiceError::Transport(_) => 2,
        HostServiceError::Panicked => 3,
        HostServiceError::MissingCapability => 7,
        HostServiceError::BufferTooSmall(_) => 8,
        HostServiceError::State(crate::state::StateError::SinkFull) => 8,
        HostServiceError::State(_) => 2,
        HostServiceError::Lease(LeaseError::InvalidHandle | LeaseError::WrongOwner)
        | HostServiceError::Terminal => 4,
        HostServiceError::InvalidDescriptor
        | HostServiceError::Lease(LeaseError::InvalidBudget | LeaseError::InvalidTransition) => 5,
        HostServiceError::InvalidState => abi::status_code::INVALID_STATE,
        HostServiceError::DestroyBusy => abi::status_code::BUSY,
        HostServiceError::Lease(LeaseError::BudgetExhausted | LeaseError::GenerationExhausted) => {
            abi::status_code::FAILED
        }
    };
    let kind = match error {
        HostServiceError::Transport(error) => error.node_error_kind().map_or(0, |kind| kind.code()),
        _ => 0,
    };
    HostCallFailure {
        status,
        kind,
        error_token,
    }
}

/// Copy a diagnostic without replacing it on size-query or short-buffer errors.
/// This remains available after a terminal panic until context teardown.
pub fn copy_context_diagnostic(
    handle: u64,
    token: u64,
    target: &mut [u8],
) -> Result<usize, HostServiceError> {
    if token & (1 << 63) != 0 {
        return raw_owner(handle)?.copy_error(token, target);
    }
    let owner = context_owner(handle)?;
    let context = owner.try_lock().map_err(|error| match error {
        std::sync::TryLockError::WouldBlock => HostServiceError::InvalidState,
        std::sync::TryLockError::Poisoned(_) => HostServiceError::Panicked,
    })?;
    catch_unwind(AssertUnwindSafe(|| context.copy_diagnostic(token, target))).map_err(
        |payload| {
            contain_panic_payload(payload);
            HostServiceError::Panicked
        },
    )?
}

/// Remove only after node calls/helpers have finished and pollers are revoked.
/// Acquired callbacks either finish before revocation or observe terminal state.
pub fn remove_context(handle: u64) -> Result<(), HostServiceError> {
    let owner = context_owner(handle)?;
    let mut context = owner.lock().map_err(|_| HostServiceError::Panicked)?;
    context.revoke();
    let mut rows = CONTEXTS
        .get()
        .ok_or(LeaseError::InvalidHandle)?
        .write()
        .map_err(|_| HostServiceError::Panicked)?;
    let index = (handle as u32 - 1) as usize;
    let row = rows.get_mut(index).ok_or(LeaseError::InvalidHandle)?;
    if row.generation != (handle >> 32) as u32 || row.owner.is_none() {
        return Err(LeaseError::InvalidHandle.into());
    }
    row.owner = None;
    row.raw = None;
    row.generation = 0;
    Ok(())
}

// CallerMemory exists only within unsafe C entries. Their caller retains live,
// nonoverlapping argument allocations throughout the call. Shape validation
// cannot establish pointer provenance; the unsafe caller contract supplies it.
// None of these temporary Rust borrows is stored in host context state.
struct CallerMemory;
struct Output<T>(*mut T);
impl<T: Copy> Output<T> {
    fn write(&self, value: T) {
        // SAFETY: output() checked null/alignment/range; the unsafe entry's
        // caller grants exclusive writable storage, initialized or not.
        unsafe { self.0.write(value) };
    }
}
impl CallerMemory {
    fn output<T>(&self, ptr: *mut T) -> Result<Output<T>, HostServiceError> {
        if ptr.is_null()
            || !(ptr as usize).is_multiple_of(std::mem::align_of::<T>())
            || (ptr as usize)
                .checked_add(std::mem::size_of::<T>())
                .is_none()
        {
            return Err(HostServiceError::InvalidDescriptor);
        }
        Ok(Output(ptr))
    }
    fn record<T: Copy>(&self, ptr: *const T) -> Result<T, HostServiceError> {
        self.output(ptr.cast_mut())?;
        // SAFETY: shape was checked above; the C caller retains an initialized
        // record for this call. Copy never transfers its allocation owner.
        Ok(unsafe { ptr.read() })
    }
    fn bytes(&self, bytes: abi::Bytes) -> Result<&[u8], HostServiceError> {
        let len = span_len(bytes.ptr, bytes.len)?;
        if len == 0 {
            return Ok(&[]);
        }
        // SAFETY: the unsafe entry's caller provides initialized readable bytes
        // in one live allocation; checked length is at most isize::MAX.
        Ok(unsafe { std::slice::from_raw_parts(bytes.ptr, len) })
    }
    fn mutable(&mut self, bytes: abi::MutableBytes) -> Result<&mut [u8], HostServiceError> {
        let len = span_len(bytes.ptr.cast_const(), bytes.len)?;
        if len == 0 {
            return Ok(&mut []);
        }
        // SAFETY: the unsafe entry's caller provides exclusive writable storage
        // for this borrow, with no overlapping arguments during the call.
        Ok(unsafe { std::slice::from_raw_parts_mut(bytes.ptr, len) })
    }
}
fn span_len(ptr: *const u8, len: u64) -> Result<usize, HostServiceError> {
    let len = usize::try_from(len).map_err(|_| HostServiceError::InvalidDescriptor)?;
    if len > isize::MAX as usize
        || (len != 0 && ptr.is_null())
        || (ptr as usize).checked_add(len).is_none()
    {
        return Err(HostServiceError::InvalidDescriptor);
    }
    Ok(len)
}
fn status(code: u32) -> abi::Status {
    abi::Status {
        code,
        kind: 0,
        error: 0,
    }
}
fn call_status(result: Result<abi::Status, HostCallFailure>) -> abi::Status {
    result.unwrap_or_else(|error| abi::Status {
        code: error.status,
        kind: error.kind,
        error: error.error_token,
    })
}
macro_rules! host_callback {
    ($name:ident($context:ident $(, $arg:ident: $ty:ty)*) |$host:ident, $memory:ident| $body:block) => {
        pub(super) unsafe extern "C" fn $name($context: abi::ContextHandle, $($arg: $ty),*) -> abi::Status {
            match catch_unwind(AssertUnwindSafe(|| {
                #[allow(unused_mut)]
                let mut $memory = CallerMemory;
                call_status(with_context($context.value, |$host| { $host.require_node_call()?; $body }))
            })) {
                Ok(result) => result,
                Err(payload) => { contain_panic_payload(payload); status(abi::status_code::PANICKED) }
            }
        }
    };
}
fn lease_port(
    host: &HostContext,
    token: abi::LeaseHandle,
    direction: LeaseDirection,
) -> Result<u64, HostServiceError> {
    let (port, owner_direction) = host.leases.owner(token.value)?;
    if owner_direction != direction {
        return Err(LeaseError::WrongOwner.into());
    }
    Ok(port)
}
fn read_record(slot: HostReadSlot, layout: PluginWireLayout) -> abi::ReadSlotV1 {
    abi::ReadSlotV1 {
        data: slot.data,
        lease: abi::LeaseHandle { value: slot.lease },
        schema_hash: layout.schema_hash,
        size: slot.size,
        alignment: layout.alignment,
    }
}
host_callback!(callback_bind(context, ordinal: u32, direction: u32, out: *mut abi::PortBindingV1) |host, memory| {
    let out = memory.output(out)?;
    let direction = match direction { 0 => LeaseDirection::Read, 1 => LeaseDirection::Write, _ => return Err(HostServiceError::InvalidDescriptor) };
    out.write(host.binding(context.value, ordinal, direction)?);
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_typed_loan(context, port: abi::PortHandle, request: abi::LoanRequestV1, out: *mut abi::WriteSlotV1) |host, memory| {
    let out = memory.output(out)?;
    let index = host.port_index(context.value, port.value, LeaseDirection::Write)?;
    let layout = host.ports[index].spec.layout;
    let prefix = layout.prefix_len().ok_or(HostServiceError::InvalidDescriptor)?;
    if request.schema_hash != layout.schema_hash || request.wire_fixed_size != layout.fixed_size
        || request.variable_count != layout.variable_fields || request.alignment != layout.alignment
        || request.minimum_capacity != prefix { return Err(HostServiceError::InvalidDescriptor); }
    let slot = host.loan_output(context.value, port.value)?;
    out.write(abi::WriteSlotV1 { data: slot.data, lease: abi::LeaseHandle { value: slot.lease }, schema_hash: layout.schema_hash, capacity: slot.capacity, configured_capacity: slot.max_capacity, alignment: layout.alignment, prefix_size: prefix });
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_finish(context, lease: abi::LeaseHandle, total: u32) |host, _memory| {
    let port = lease_port(host, lease, LeaseDirection::Write)?;
    host.commit_output(context.value, port, lease.value, total)?;
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_spill(context, lease: abi::LeaseHandle, payload: abi::Bytes, total: u32) |host, memory| {
    let port = lease_port(host, lease, LeaseDirection::Write)?;
    match memory.bytes(payload) {
        Ok(payload) => host.submit_spill(context.value, port, lease.value, payload, total)?,
        Err(error) => {
            host.reject_spill_span(context.value, port, lease.value)?;
            return Err(error);
        }
    }
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_typed_discard(context, lease: abi::LeaseHandle, reason: u32) |host, _memory| {
    let port = lease_port(host, lease, LeaseDirection::Write)?;
    let reason = match reason {
        abi::discard_reason::TICK_FAILED | abi::discard_reason::TICK_PANICKED => HostDiscardReason::TickFailed,
        abi::discard_reason::MISSING_VARIABLE | abi::discard_reason::NESTED_FAILED => HostDiscardReason::Incomplete,
        abi::discard_reason::REPLAY_SUPPRESSED => HostDiscardReason::ReplaySuppressed,
        abi::discard_reason::SPILL_RELEASE => HostDiscardReason::SpillRelease,
        abi::discard_reason::INVARIANT => HostDiscardReason::Invariant,
        _ => { host.discard_output(context.value, port, lease.value, HostDiscardReason::Invariant)?; return Err(HostServiceError::InvalidDescriptor); },
    };
    host.discard_output(context.value, port, lease.value, reason)?;
    Ok(status(abi::status_code::OK))
});
unsafe extern "C" fn callback_loan(
    context: abi::ContextHandle,
    port: abi::PortHandle,
    request: abi::LoanRequestV1,
    out: *mut abi::WriteSlotV1,
) -> abi::Status {
    match raw_owner(context.value) {
        Ok(raw) if raw.port_candidate(port.value) => {
            // SAFETY: forwarding the unchanged live caller arguments preserves the C entry contract.
            unsafe { raw::callback_loan_raw(context, port, request, out) }
        }
        _ => {
            // SAFETY: forwarding the unchanged live caller arguments preserves the C entry contract.
            unsafe { callback_typed_loan(context, port, request, out) }
        }
    }
}
unsafe extern "C" fn callback_discard(
    context: abi::ContextHandle,
    lease: abi::LeaseHandle,
    reason: u32,
) -> abi::Status {
    match raw_owner(context.value) {
        Ok(raw) if raw.lease_candidate(lease.value) => {
            // SAFETY: forwarding the unchanged caller handles preserves the C entry contract.
            unsafe { raw::callback_discard_raw(context, lease, reason) }
        }
        _ => {
            // SAFETY: forwarding the unchanged caller handles preserves the C entry contract.
            unsafe { callback_typed_discard(context, lease, reason) }
        }
    }
}

host_callback!(callback_receive(context, port: abi::PortHandle, mode: u32, out: *mut abi::ReadSlotV1) |host, memory| {
    let out = memory.output(out)?;
    let index = host.port_index(context.value, port.value, LeaseDirection::Read)?;
    if mode != host.ports[index].spec.read_mode { return Err(HostServiceError::InvalidDescriptor); }
    let layout = host.ports[index].spec.layout;
    match host.receive_input(context.value, port.value)? {
        Some(slot) => { out.write(read_record(slot, layout)); Ok(status(abi::status_code::OK)) },
        None => Ok(status(abi::status_code::EMPTY)),
    }
});
host_callback!(callback_release(context, lease: abi::LeaseHandle) |host, _memory| {
    let port = lease_port(host, lease, LeaseDirection::Read)?;
    host.release_input(context.value, port, lease.value)?;
    Ok(status(abi::status_code::OK))
});

host_callback!(callback_head(context, port: abi::PortHandle, operation: u32, out: *mut abi::HeadOutcomeV1) |host, memory| {
    use crate::scheduler::SyncAlignSite;
    let out = memory.output(out)?;
    let mut value = abi::HeadOutcomeV1 { popped: 0, timestamp_ns: 0, has_timestamp: 0, kind: abi::head_kind::NOTHING };
    if matches!(operation, abi::head_op::DRAIN | abi::head_op::REFILL) {
        let index = host.port_index(context.value, port.value, LeaseDirection::Read)?;
        host.unborrowed(index)?;
        let Some(super::node::AnySubscriber::Ipc(subscriber)) = host.native.subscriber_mut(&host.ports[index].spec.name) else { return Err(HostServiceError::InvalidDescriptor); };
        let site = if operation == abi::head_op::DRAIN { SyncAlignSite::Boundary } else { SyncAlignSite::Refill };
        let (popped, stamp, withheld, failed) = subscriber.plugin_trigger_drain(site);
        value.popped = popped;
        value.timestamp_ns = stamp.unwrap_or(0);
        value.has_timestamp = u32::from(stamp.is_some());
        value.kind = if failed { abi::head_kind::FAILED } else if withheld { abi::head_kind::WITHHELD } else if stamp.is_some() { abi::head_kind::HEAD } else { abi::head_kind::NOTHING };
    } else {
        let operation = match operation {
            abi::head_op::FILL_BOUNDARY => SyncHeadOp::FillBoundary,
            abi::head_op::FILL_REFILL => SyncHeadOp::FillRefill,
            abi::head_op::PROBE_NEXT => SyncHeadOp::ProbeNext,
            abi::head_op::PEEK_BOUNDARY => SyncHeadOp::PeekNext { site: SyncAlignSite::Boundary },
            abi::head_op::PEEK_REFILL => SyncHeadOp::PeekNext { site: SyncAlignSite::Refill },
            abi::head_op::ADVANCE_BOUNDARY => SyncHeadOp::Advance { site: SyncAlignSite::Boundary },
            abi::head_op::ADVANCE_REFILL => SyncHeadOp::Advance { site: SyncAlignSite::Refill },
            abi::head_op::VOID => SyncHeadOp::Void,
            _ => return Err(HostServiceError::InvalidDescriptor),
        };
        value.kind = match host.input_head(context.value, port.value, operation)? {
            SyncOpAnswer::Nothing => abi::head_kind::NOTHING,
            SyncOpAnswer::Present => abi::head_kind::PRESENT,
            SyncOpAnswer::Failed => abi::head_kind::FAILED,
            SyncOpAnswer::Stamp(stamp) => { value.timestamp_ns = stamp; value.has_timestamp = 1; abi::head_kind::STAMP },
            SyncOpAnswer::Head(stamp) => { value.timestamp_ns = stamp; value.has_timestamp = 1; abi::head_kind::HEAD },
        };
    }
    out.write(value);
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_snapshots_configure(context, names: abi::Bytes) |host, memory| {
    host.configure_snapshots(memory.bytes(names)?)?;
    Ok(status(abi::status_code::OK))
});
host_callback!(
    callback_snapshots(context) | host,
    _memory | {
        host.snapshot_configured()?;
        Ok(status(abi::status_code::OK))
    }
);
host_callback!(
    callback_pump(context) | host,
    _memory | {
        host.pump_history()?;
        Ok(status(abi::status_code::OK))
    }
);
host_callback!(
    callback_shutdown(context) | host,
    _memory | {
        host.request_shutdown()?;
        Ok(status(abi::status_code::OK))
    }
);
host_callback!(callback_environment(context, key: abi::Bytes, target: abi::MutableBytes, required: *mut u64) |host, memory| {
    let required = memory.output(required)?;
    let key = std::str::from_utf8(memory.bytes(key)?).map_err(|_| HostServiceError::InvalidDescriptor)?;
    let value = host.environment_value(key)?;
    let len = value.map_or(0, str::len);
    required.write(len as u64);
    let target = memory.mutable(target)?;
    let Some(value) = value else { return Ok(status(abi::status_code::EMPTY)); };
    if target.len() < len { return Err(HostServiceError::BufferTooSmall(len)); }
    target[..len].copy_from_slice(value.as_bytes());
    Ok(status(abi::status_code::OK))
});
fn event_record(port: abi::PortHandle, event: HostEvent) -> abi::EventV1 {
    use super::node::BackpressurePolicy;
    use crate::scheduler::{LivelinessCause, LivelinessState};
    let mut out = abi::EventV1 {
        port,
        timestamp_ns: 0,
        total: 0,
        in_regime: 0,
        dropped: 0,
        capacity: 0,
        kind: 0,
        detail: 0,
        window_ms: 0,
        elapsed_ms: 0,
        policy_value: 0,
        cause: 0,
        reserved: 0,
    };
    match event {
        HostEvent::Backpressure(event) => {
            out.kind = abi::event_kind::BACKPRESSURE;
            out.detail = match event.policy {
                BackpressurePolicy::DropOldest => abi::event_detail::DROP_OLDEST,
                BackpressurePolicy::Block => abi::event_detail::BLOCK,
                BackpressurePolicy::Sample(interval) => {
                    out.policy_value = interval;
                    abi::event_detail::SAMPLE
                }
            };
            out.timestamp_ns = event.regime_started_at_ns;
            out.total = event.count_total;
            out.in_regime = event.count_in_regime;
            out.dropped = event.dropped;
            out.capacity = event.buffer_capacity as u64;
        }
        HostEvent::ExpectWithin(event) => {
            out.kind = abi::event_kind::EXPECT_WITHIN;
            out.timestamp_ns = event.missed_at_ns;
            out.total = event.count_total;
            out.window_ms = event.expect_within_ms;
            out.elapsed_ms = event.elapsed_ms;
        }
        HostEvent::PromiseWithin(event) => {
            out.kind = abi::event_kind::PROMISE_WITHIN;
            out.timestamp_ns = event.missed_at_ns;
            out.total = event.count_total;
            out.window_ms = event.promise_within_ms;
            out.elapsed_ms = event.elapsed_ms;
        }
        HostEvent::Liveliness(event) => {
            out.kind = abi::event_kind::LIVELINESS;
            out.timestamp_ns = event.changed_at_ns;
            out.total = event.count_total;
            out.capacity = event.publisher_count as u64;
            out.detail = match event.state {
                LivelinessState::Alive => abi::event_detail::ALIVE,
                LivelinessState::Lost => abi::event_detail::LOST,
            };
            out.cause = match event.cause {
                LivelinessCause::PublisherConnected => abi::event_detail::CONNECTED,
                LivelinessCause::PublisherDisconnected => abi::event_detail::DISCONNECTED,
            };
        }
    }
    out
}
host_callback!(callback_event(context, port: abi::PortHandle, kind: u32, out: *mut abi::EventV1) |host, memory| {
    let out = memory.output(out)?;
    let kind = match kind { abi::event_kind::BACKPRESSURE => HostEventKind::Backpressure, abi::event_kind::EXPECT_WITHIN => HostEventKind::ExpectWithin, abi::event_kind::PROMISE_WITHIN => HostEventKind::PromiseWithin, abi::event_kind::LIVELINESS => HostEventKind::Liveliness, _ => return Err(HostServiceError::InvalidDescriptor) };
    match host.poll_event(context.value, port.value, kind)? {
        Some(event) => { out.write(event_record(port, event)); Ok(status(abi::status_code::OK)) },
        None => Ok(status(abi::status_code::EMPTY)),
    }
});
host_callback!(callback_counters(context, port: abi::PortHandle, out: *mut abi::PublisherCountersV1) |host, memory| {
    let out = memory.output(out)?;
    let index = host.port_index(context.value, port.value, LeaseDirection::Write)?;
    let publisher = host.output_publisher(index)?;
    out.write(abi::PublisherCountersV1 { committed: u64::from(publisher.committed_frames()), discarded: publisher.output_discard_count(), send_failures: publisher.frames_dropped_send_fail() });
    Ok(status(abi::status_code::OK))
});
unsafe extern "C" fn callback_copy_error(
    context: abi::ContextHandle,
    token: u64,
    target: abi::MutableBytes,
    required: *mut u64,
) -> abi::Status {
    match catch_unwind(AssertUnwindSafe(|| {
        let mut memory = CallerMemory;
        let required = memory.output(required)?;
        let target = memory.mutable(target)?;
        match copy_context_diagnostic(context.value, token, target) {
            Ok(len) => {
                required.write(len as u64);
                Ok(status(abi::status_code::OK))
            }
            Err(HostServiceError::BufferTooSmall(len)) => {
                required.write(len as u64);
                Ok(status(abi::status_code::BUFFER_TOO_SMALL))
            }
            Err(error) => Err(error),
        }
    })) {
        Ok(Ok(result)) => result,
        Ok(Err(error)) => call_status(Err(classify_failure(&error, 0))),
        Err(payload) => {
            contain_panic_payload(payload);
            status(abi::status_code::PANICKED)
        }
    }
}

host_callback!(callback_diagnostic(context, port: abi::PortHandle, level: u32, text: abi::Bytes) |host, memory| {
    use crate::transport::failure_regime_latch::RegimeDecision;
    host.admit()?;
    let text = std::str::from_utf8(memory.bytes(text)?).map_err(|_| HostServiceError::InvalidDescriptor)?;
    if !(1..=5).contains(&level) { return Err(HostServiceError::InvalidDescriptor); }
    let latch = if port.value == 0 { &mut host.diagnostics } else {
        let index = (port.value as u32).checked_sub(1).ok_or(LeaseError::InvalidHandle)? as usize;
        let bound = host.ports.get(index).ok_or(LeaseError::InvalidHandle)?;
        host.port_index(context.value, port.value, bound.spec.direction)?;
        &mut host.ports[index].diagnostics
    };
    if level <= 2 {
        match latch.on_failure() {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                if level == 1 { tracing::error!(port = port.value, error = text, total_failures = latch.total_failures(), "node diagnostic"); }
                else { tracing::warn!(port = port.value, error = text, total_failures = latch.total_failures(), "node diagnostic"); }
            },
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(port = port.value, error = text, suppressed_count = suppressed, "node diagnostic repeated"),
        }
    } else {
        if let Some(suppressed) = latch.on_success() { tracing::info!(port = port.value, suppressed_count = suppressed, "node diagnostic recovered"); }
        match level {
            3 => tracing::info!(port = port.value, diagnostic = text, "node diagnostic"),
            4 => tracing::debug!(port = port.value, diagnostic = text, "node diagnostic"),
            _ => tracing::trace!(port = port.value, diagnostic = text, "node diagnostic"),
        }
    }
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_accumulate_count(context, port: abi::PortHandle, out: *mut abi::AccumulateInfoV1) |host, memory| {
    let out = memory.output(out)?;
    out.write(host.accumulate_info(context.value, port.value)?);
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_accumulate_frame(context, port: abi::PortHandle, index: u32, out: *mut abi::ReadSlotV1) |host, memory| {
    let out = memory.output(out)?;
    let port_index = host.accumulator_index(context.value, port.value)?;
    let layout = host.ports[port_index].spec.layout;
    out.write(read_record(host.accumulated_frame(context.value, port.value, index)?, layout));
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_mark_served(context, lease: abi::LeaseHandle) |host, _memory| {
    host.mark_accumulated_served(context.value, lease.value)?;
    Ok(status(abi::status_code::OK))
});
struct CallbackSink {
    sink: abi::StateSinkV1,
    failure: Option<abi::Status>,
}
impl crate::state::StateSink for CallbackSink {
    fn write(&mut self, bytes: &[u8]) -> Result<(), crate::state::SinkFull> {
        if self.failure.is_some() {
            return Err(crate::state::SinkFull);
        }
        // SAFETY: caller retains the concrete sink record/context and callback
        // for this synchronous capture. Bytes borrow the host encoder only
        // during this call, and the callback contains its own panics.
        let result = unsafe {
            (self.sink.write)(
                self.sink.context,
                abi::Bytes {
                    ptr: bytes.as_ptr(),
                    len: bytes.len() as u64,
                },
            )
        };
        if result.code == abi::status_code::OK {
            Ok(())
        } else {
            self.failure = Some(result);
            Err(crate::state::SinkFull)
        }
    }
}
host_callback!(callback_capture(context, port: abi::PortHandle, sink: *const abi::StateSinkV1) |host, memory| {
    let mut sink = CallbackSink { sink: memory.record(sink)?, failure: None };
    let result = host.capture_accumulator(context.value, port.value, &mut sink);
    if let Some(failure) = sink.failure { return Ok(abi::Status { code: failure.code, kind: failure.kind, error: 0 }); }
    result?;
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_extension(context, _id: u64, _version: u32, out: *mut abi::ExtensionDescriptorV1) |host, memory| {
    memory.output(out)?;
    host.admit()?;
    Ok(status(abi::status_code::UNSUPPORTED))
});
host_callback!(callback_restore(context, port: abi::PortHandle, bytes: abi::Bytes) |host, memory| {
    host.restore_accumulator(context.value, port.value, memory.bytes(bytes)?)?;
    Ok(status(abi::status_code::OK))
});

mod raw;

/// Process-lifetime frozen host table. The loader supplies this same table to
/// every checked node instance; each instance has its own context handle.
pub static HOST_API: abi::HostApiV1 = abi::HostApiV1 {
    header: abi::TableHeaderV1::for_type::<abi::HostApiV1>(),
    capabilities: abi::capability::STATE | abi::capability::ACCUMULATE,
    bind_port: callback_bind,
    loan_output: callback_loan,
    finish_output: callback_finish,
    submit_spill: callback_spill,
    discard_output: callback_discard,
    receive_input: callback_receive,
    release_input: callback_release,
    input_head: callback_head,
    set_snapshot_inputs: callback_snapshots_configure,
    snapshot_inputs: callback_snapshots,
    pump_history: callback_pump,
    clock: raw::callback_clock,
    request_shutdown: callback_shutdown,
    environment: callback_environment,
    poll_event: callback_event,
    publisher_counters: callback_counters,
    copy_error: callback_copy_error,
    diagnostic: callback_diagnostic,
    accumulate_count: callback_accumulate_count,
    accumulate_frame: callback_accumulate_frame,
    mark_served: callback_mark_served,
    capture_accumulate: callback_capture,
    query_extension: callback_extension,
    restore_accumulate: callback_restore,
    create_raw_publisher: raw::callback_create_raw,
    finish_raw_output: raw::callback_finish_raw,
    register_raw_egress: raw::callback_register_raw,
    destroy_raw_publisher: raw::callback_destroy_raw,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::VirtualClock;
    use crate::graph::node::ShutdownSignal;
    use indexmap::IndexMap;

    fn clock_context(now: u64) -> HostContext {
        let clock = Arc::new(VirtualClock::new());
        clock.set(now);
        let env = Arc::new(HashMap::from([(
            "HOST_ABI_ORACLE".to_string(),
            "frozen".to_string(),
        )]));
        let native = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock,
            ShutdownSignal::new(),
            Arc::clone(&env),
        );
        HostContext::new(native, Vec::new(), env, 0).unwrap()
    }

    #[test]
    fn clock_environment_and_shutdown_use_the_host_context() {
        for _ in 0..2 {
            let context = clock_context(123_456);
            assert_eq!(context.clock_now().unwrap(), 123_456);
            assert_eq!(context.clock_virtual().unwrap(), Some(123_456));
            assert_eq!(context.clock_external().unwrap(), None);
            assert_eq!(
                context.environment_value("HOST_ABI_ORACLE").unwrap(),
                Some("frozen")
            );
            assert_eq!(
                context.environment_value("ABSENT_HOST_ABI_KEY").unwrap(),
                None
            );
            assert!(!context.native.shutdown_signal().is_requested());
            context.request_shutdown().unwrap();
            assert!(context.native.shutdown_signal().is_requested());
        }
    }

    #[test]
    fn stale_context_and_port_handles_refuse_before_effects() {
        let handle = register_context(clock_context(77)).unwrap();
        assert_eq!(with_context(handle, |context| context.clock_now()), Ok(77));
        for invalid in [0, 1, u64::MAX, handle ^ (1 << 32)] {
            assert_eq!(
                with_context(invalid, |context| context.clock_now())
                    .unwrap_err()
                    .status,
                4
            );
        }
        let failure = with_context(handle, |context| {
            context.bind_port(handle, 0, 0, LeaseDirection::Write)
        })
        .unwrap_err();
        assert_eq!(failure.status, 5);
        assert_eq!(
            with_context(handle, |context| Ok(context.outstanding_leases())),
            Ok(0)
        );
        remove_context(handle).unwrap();
        assert_eq!(
            with_context(handle, |context| context.clock_now())
                .unwrap_err()
                .status,
            4
        );
        assert!(matches!(
            remove_context(handle),
            Err(HostServiceError::Lease(LeaseError::InvalidHandle))
        ));
    }

    #[test]
    fn caught_host_panic_keeps_the_gate_unpoisoned_and_diagnostics_available() {
        let handle = register_context(clock_context(99)).unwrap();
        let error =
            with_context::<()>(handle, |_| panic!("host callback panic oracle")).unwrap_err();
        assert_eq!(error.status, 3);
        let mut text = [0; 64];
        let written = copy_context_diagnostic(handle, error.error_token, &mut text).unwrap();
        assert_eq!(&text[..written], b"Panicked");
        assert_eq!(
            with_context(handle, |context| context.clock_now())
                .unwrap_err()
                .status,
            4
        );
        remove_context(handle).unwrap();
    }

    #[test]
    fn short_diagnostic_buffers_retry_without_truncation_or_token_consumption() {
        let handle = register_context(clock_context(0)).unwrap();
        let error =
            with_context::<()>(handle, |_| Err(HostServiceError::InvalidDescriptor)).unwrap_err();
        let mut short = [0xAA; 3];
        assert!(matches!(
            copy_context_diagnostic(handle, error.error_token, &mut short),
            Err(HostServiceError::BufferTooSmall(17))
        ));
        assert_eq!(short, [0xAA; 3]);
        let mut exact = [0; 17];
        assert_eq!(
            copy_context_diagnostic(handle, error.error_token, &mut exact).unwrap(),
            17
        );
        assert_eq!(&exact, b"InvalidDescriptor");
        assert!(matches!(
            copy_context_diagnostic(handle, 0, &mut exact),
            Err(HostServiceError::Lease(LeaseError::InvalidHandle))
        ));
        remove_context(handle).unwrap();
    }

    #[test]
    fn independent_context_gates_allow_concurrent_operations() {
        let first = register_context(clock_context(1)).unwrap();
        let second = register_context(clock_context(2)).unwrap();
        std::thread::scope(|scope| {
            let (entered_tx, entered_rx) = std::sync::mpsc::channel();
            let (release_a, wait_a) = std::sync::mpsc::channel();
            let (release_b, wait_b) = std::sync::mpsc::channel();
            let entered_a = entered_tx.clone();
            scope.spawn(move || {
                with_context(first, |context| {
                    entered_a.send(()).unwrap();
                    let _ = wait_a.recv();
                    assert_eq!(context.clock_now().unwrap(), 1);
                    Ok(())
                })
                .unwrap()
            });
            scope.spawn(move || {
                with_context(second, |context| {
                    entered_tx.send(()).unwrap();
                    let _ = wait_b.recv();
                    assert_eq!(context.clock_now().unwrap(), 2);
                    Ok(())
                })
                .unwrap()
            });
            // A bounded liveness check, not a performance oracle. On failure,
            // these senders drop before scope joins and unblock both workers.
            for _ in 0..2 {
                entered_rx
                    .recv_timeout(std::time::Duration::from_secs(5))
                    .expect("independent gates both entered");
            }
            release_a.send(()).unwrap();
            release_b.send(()).unwrap();
        });
        remove_context(first).unwrap();
        remove_context(second).unwrap();
    }

    #[test]
    fn replay_suppression_returns_storage_without_live_discard_or_sequence_effects() {
        use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
        let transport = crate::testing::TestTransport::new();
        let mut publisher =
            transport.publisher("abi/replay", crate::wire::MaxSliceLen::const_new(64), 0);
        let mut subscriber = transport.subscriber("abi/replay");
        let suppressed = Arc::new(AtomicBool::new(true));
        let discard_signal = Arc::new(AtomicU32::new(0));
        publisher.set_replay_suppress(Arc::clone(&suppressed));
        publisher.set_discard_signal(Arc::clone(&discard_signal));
        let layout = PluginWireLayout {
            schema_hash: 0x1122_3344_5566_7788,
            fixed_size: 24,
            variable_fields: 0,
            alignment: crate::shm_runtime::FRAME_ALIGN as u32,
        };
        let env = Arc::new(HashMap::new());
        let native = NodeContext::with_runtime_env(
            IndexMap::from([("out".to_string(), AnyPublisher::Ipc(publisher))]),
            IndexMap::new(),
            Arc::new(VirtualClock::new()),
            ShutdownSignal::new(),
            Arc::clone(&env),
        );
        let host = HostContext::new(
            native,
            vec![HostPortSpec {
                name: "out".into(),
                direction: LeaseDirection::Write,
                layout,
                read_mode: 0,
                accumulate_cap: None,
            }],
            env,
            1,
        )
        .unwrap();
        let handle = register_context(host).unwrap();
        let port = with_context(handle, |host| {
            host.bind_port(handle, 0, layout.schema_hash, LeaseDirection::Write)
        })
        .unwrap();
        for reason in [
            HostDiscardReason::TickFailed,
            HostDiscardReason::Incomplete,
            HostDiscardReason::Invariant,
            HostDiscardReason::Teardown,
            HostDiscardReason::ReplaySuppressed,
        ] {
            let loan = with_context(handle, |host| host.loan_output(handle, port)).unwrap();
            with_context(handle, |host| {
                host.discard_output(handle, port, loan.lease, reason)
            })
            .unwrap();
        }
        let loan = with_context(handle, |host| host.loan_output(handle, port)).unwrap();
        // The native proxy suppresses before checking a writer or its size.
        with_context(handle, |host| {
            host.commit_output(handle, port, loan.lease, 0)
        })
        .unwrap();
        let loan = with_context(handle, |host| host.loan_output(handle, port)).unwrap();
        with_context(handle, |host| {
            host.submit_spill(handle, port, loan.lease, &[], 0)
        })
        .unwrap();
        assert_eq!(
            with_context(handle, |host| Ok(host.outstanding_leases())),
            Ok(1)
        );
        with_context(handle, |host| {
            host.discard_output(handle, port, loan.lease, HostDiscardReason::SpillRelease)
        })
        .unwrap();
        assert_eq!(discard_signal.load(Ordering::Relaxed), 0);
        assert_eq!(
            with_context(handle, |host| Ok(host.counters())),
            Ok(HostLeaseCounters::default())
        );
        with_context(handle, |host| {
            host.with_native(|native| {
                let Some(AnyPublisher::Ipc(publisher)) = native.publisher("out") else {
                    panic!("output remains bound");
                };
                assert_eq!(
                    (
                        publisher.committed_frames(),
                        publisher.output_discard_count(),
                        publisher.frames_dropped_invariant_violation()
                    ),
                    (0, 0, 0)
                );
            })
        })
        .unwrap();
        assert!(subscriber.view_raw().unwrap().is_none());
        suppressed.store(false, Ordering::Relaxed);
        let loan = with_context(handle, |host| host.loan_output(handle, port)).unwrap();
        with_context(handle, |host| {
            host.commit_output(handle, port, loan.lease, 64)
        })
        .unwrap();
        let delivered = subscriber
            .view_raw_expecting(layout.schema_hash)
            .unwrap()
            .unwrap();
        assert_eq!(
            (
                delivered.header().sequence,
                delivered.header().timestamp_ns,
                delivered.header().total_size
            ),
            (0, 0, 64)
        );
        assert_eq!(&delivered[40..], &[0; 24]);
        drop(delivered);
        remove_context(handle).unwrap();
    }

    #[test]
    fn panicking_payload_destructor_is_contained_before_the_gate_can_poison() {
        struct DropPanic;
        impl Drop for DropPanic {
            fn drop(&mut self) {
                panic!("panic payload destructor oracle");
            }
        }
        let handle = register_context(clock_context(900)).unwrap();
        let failure = with_context::<()>(handle, |_| std::panic::panic_any(DropPanic)).unwrap_err();
        assert_eq!(failure.status, abi::status_code::PANICKED);
        let mut text = [0; 32];
        assert_eq!(
            copy_context_diagnostic(handle, failure.error_token, &mut text).unwrap(),
            8
        );
        assert_eq!(&text[..8], b"Panicked");
        remove_context(handle).unwrap();
    }

    #[test]
    fn reentrant_callback_refuses_invalid_state_without_waiting_on_its_own_gate() {
        let handle = register_context(clock_context(321)).unwrap();
        with_context(handle, |context| {
            // SAFETY: the context is live; reentrant ordinary admission must
            // refuse before accessing state or requesting shutdown.
            let result =
                unsafe { (HOST_API.request_shutdown)(abi::ContextHandle { value: handle }) };
            assert_eq!(result.code, abi::status_code::INVALID_STATE);
            assert_eq!(context.clock_now().unwrap(), 321);
            Ok(())
        })
        .unwrap();
        assert_eq!(with_context(handle, |context| context.clock_now()), Ok(321));
        remove_context(handle).unwrap();
    }

    #[test]
    fn status_classification_preserves_every_refusal_family() {
        for (error, status) in [
            (HostServiceError::Lease(LeaseError::InvalidHandle), 4),
            (HostServiceError::Lease(LeaseError::WrongOwner), 4),
            (HostServiceError::Lease(LeaseError::InvalidTransition), 5),
            (HostServiceError::Lease(LeaseError::InvalidBudget), 5),
            (HostServiceError::Lease(LeaseError::BudgetExhausted), 2),
            (HostServiceError::Lease(LeaseError::GenerationExhausted), 2),
            (HostServiceError::InvalidState, 9),
            (HostServiceError::DestroyBusy, 6),
            (HostServiceError::InvalidDescriptor, 5),
            (HostServiceError::BufferTooSmall(17), 8),
            (
                HostServiceError::State(crate::state::StateError::SinkFull),
                8,
            ),
            (
                HostServiceError::State(crate::state::StateError::TrailingBytes { remaining: 1 }),
                2,
            ),
            (HostServiceError::Panicked, 3),
            (HostServiceError::MissingCapability, 7),
            (HostServiceError::Terminal, 4),
        ] {
            assert_eq!(
                classify_failure(&error, 13),
                HostCallFailure {
                    status,
                    kind: 0,
                    error_token: 13
                }
            );
        }
        // Native error classification uses the real crate error constructor.
        let error = TransportError::Internal {
            reason: "host error oracle".to_string(),
        };
        assert_eq!(
            classify_failure(&HostServiceError::Transport(error), 13).status,
            2
        );
    }
}

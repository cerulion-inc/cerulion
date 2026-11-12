// SPDX-License-Identifier: AGPL-3.0-only
//! Raw ingress is independent of the native node-call mutex. Each publisher
//! owns its native port and fixed sample arena behind its own execution gate.
use super::*;
use crate::transport::publisher::CerulionPublisher;
use crate::transport::FullQueueSend;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

struct RawPublisher {
    topic: String,
    schema_hash: u64,
    state: Mutex<PublisherState>,
}
struct PublisherState {
    publisher: Option<CerulionPublisher>,
    leases: LeaseArena<RawShmLoan>,
    panicked: bool,
    // Loans and port retire before the producer's exclusion.
    local_scope: Option<crate::transport::egress_scope::LocalOnlyTopic>,
}
struct RawDiagnostic {
    token: u64,
    error: Option<HostServiceError>,
}
pub(super) struct RawServices {
    rows: Box<[OnceLock<RawPublisher>]>,
    next: Mutex<usize>,
    // Keep the process's existing manager through the last publisher owner.
    manager: Option<Arc<crate::transport::TransportManager>>,
    clock: Arc<dyn crate::clock::Clock>,
    normal_ports: usize,
    lease_budget: usize,
    diagnostic: Mutex<RawDiagnostic>,
    queue_diagnostics: [Mutex<crate::transport::failure_regime_latch::FailureRegimeLatch>; 2],
    pub(super) active: AtomicUsize,
    pub(super) committed: AtomicU64,
    pub(super) discarded: AtomicU64,
    pub(super) violations: AtomicU64,
    pub(super) invariants: AtomicU64,
    retired: AtomicBool,
}
impl RawServices {
    // hot-path-alloc-ok-fn: immutable bounded publisher index rows are cold provisioned
    pub(super) fn new(
        manager: Option<Arc<crate::transport::TransportManager>>,
        clock: Arc<dyn crate::clock::Clock>,
        normal_ports: usize,
        lease_budget: usize,
        budget: usize,
    ) -> Result<Self, HostServiceError> {
        if normal_ports
            .checked_add(budget)
            .is_none_or(|n| n >= u32::MAX as usize)
            || budget
                .checked_add(1)
                .and_then(|n| n.checked_mul(lease_budget))
                .is_none_or(|n| n > u32::MAX as usize)
        {
            return Err(LeaseError::InvalidBudget.into());
        }
        Ok(Self {
            rows: (0..budget)
                .map(|_| OnceLock::new())
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            next: Mutex::new(0),
            queue_diagnostics: std::array::from_fn(|_| {
                Mutex::new(crate::transport::failure_regime_latch::FailureRegimeLatch::new())
            }),
            manager,
            clock,
            normal_ports,
            lease_budget,
            diagnostic: Mutex::new(RawDiagnostic {
                token: 0,
                error: None,
            }),
            active: AtomicUsize::new(0),
            committed: AtomicU64::new(0),
            discarded: AtomicU64::new(0),
            invariants: AtomicU64::new(0),
            violations: AtomicU64::new(0),
            retired: AtomicBool::new(false),
        })
    }
    pub(super) fn lease_budget(&self) -> usize {
        self.lease_budget
    }
    pub(super) fn port_candidate(&self, port: u64) -> bool {
        port as u32 as usize > self.normal_ports
    }
    pub(super) fn lease_candidate(&self, token: u64) -> bool {
        token as u32 as usize > self.lease_budget
    }
    fn port_index(&self, context: u64, port: u64) -> Result<usize, HostServiceError> {
        if context == 0 || context >> 32 != port >> 32 {
            return Err(LeaseError::InvalidHandle.into());
        }
        (port as u32 as usize)
            .checked_sub(self.normal_ports + 1)
            .ok_or_else(|| LeaseError::InvalidHandle.into())
    }
    fn lease_index(&self, token: u64) -> Result<usize, HostServiceError> {
        if self.lease_budget == 0 {
            return Err(LeaseError::InvalidHandle.into());
        }
        (token as u32 as usize)
            .checked_sub(1)
            .map(|n| n / self.lease_budget)
            .and_then(|n| n.checked_sub(1))
            .ok_or_else(|| LeaseError::InvalidHandle.into())
    }
    fn row(&self, index: usize) -> Result<&RawPublisher, HostServiceError> {
        self.rows
            .get(index)
            .and_then(OnceLock::get)
            .ok_or_else(|| LeaseError::InvalidHandle.into())
    }
    fn port_handle(&self, context: u64, index: usize) -> u64 {
        context & 0xffff_ffff_0000_0000 | (self.normal_ports + index + 1) as u64
    }
    fn with_publisher<R>(
        &self,
        index: usize,
        operation: impl FnOnce(&RawPublisher, &mut PublisherState) -> Result<R, HostServiceError>,
    ) -> Result<R, HostServiceError> {
        let row = self.row(index)?;
        let mut state = row.state.lock().map_err(|_| HostServiceError::Panicked)?;
        if state.publisher.is_none() {
            return Err(LeaseError::InvalidHandle.into());
        }
        match catch_unwind(AssertUnwindSafe(|| operation(row, &mut state))) {
            Ok(result) => result,
            Err(payload) => {
                state.panicked = true;
                contain_panic_payload(payload);
                Err(HostServiceError::Panicked)
            }
        }
    }
    fn admit(&self) -> Result<(), HostServiceError> {
        if self.retired.load(Ordering::Acquire) {
            Err(HostServiceError::Terminal)
        } else {
            Ok(())
        }
    }
    // hot-path-alloc-ok-fn: topic, native ingress service and fixed lease rows are created cold
    fn create(
        &self,
        context: u64,
        topic: &str,
        request: abi::RawPublisherRequestV1,
    ) -> Result<abi::PortBindingV1, HostServiceError> {
        self.admit()?;
        let strategy = match request.full_queue {
            abi::full_queue::CONFIG_DEFAULT => FullQueueSend::ConfigDefault,
            abi::full_queue::DISCARD => FullQueueSend::Discard,
            tag => {
                use crate::transport::failure_regime_latch::RegimeDecision;
                let reserved = tag == abi::full_queue::RESERVED_BLOCK;
                let mut latch = crate::transport::failure_regime_latch::lock_regime_latch(
                    &self.queue_diagnostics[usize::from(!reserved)],
                );
                match latch.on_failure() {
                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(
                        topic,
                        full_queue = tag,
                        total_failures = latch.total_failures(),
                        "raw ingress full-queue strategy refused"
                    ),
                    RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                        topic,
                        full_queue = tag,
                        suppressed_count = suppressed,
                        "raw ingress full-queue strategy refusal repeated"
                    ),
                }
                return Err(if reserved {
                    HostServiceError::UnsupportedRawStrategy(tag)
                } else {
                    HostServiceError::UnknownRawStrategy(tag)
                });
            }
        };
        if topic.is_empty() || request.configured_capacity < WireHeader::SIZE as u32 {
            return Err(HostServiceError::InvalidDescriptor);
        }
        let capacity = crate::wire::MaxSliceLen::try_new(request.configured_capacity)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let mut next = self.next.lock().map_err(|_| HostServiceError::Panicked)?;
        if *next == self.rows.len() {
            return Err(LeaseError::BudgetExhausted.into());
        }
        let manager = self
            .manager
            .as_ref()
            .ok_or(HostServiceError::MissingCapability)?;
        let local_scope = manager.retain_local_topic_scope(topic)?;
        let publisher = manager.create_ingress_publisher_with(topic, capacity, strategy)?;
        self.admit()?;
        let index = *next;
        let topic = publisher.topic().to_owned();
        let leases = LeaseArena::with_index_base(
            self.lease_budget,
            ((index + 1) * self.lease_budget) as u32,
        )?;
        let row = RawPublisher {
            topic,
            schema_hash: request.schema_hash,
            state: Mutex::new(PublisherState {
                publisher: Some(publisher),
                leases,
                panicked: false,
                local_scope,
            }),
        };
        self.rows[index]
            .set(row)
            .map_err(|_| HostServiceError::InvalidDescriptor)?;
        *next += 1;
        let row = self.row(index)?;
        for latch in &self.queue_diagnostics {
            if let Some(suppressed) =
                crate::transport::failure_regime_latch::lock_regime_latch(latch).on_success()
            {
                tracing::info!(
                    topic = %row.topic,
                    suppressed_count = suppressed,
                    "raw ingress full-queue strategy recovered"
                );
            }
        }
        Ok(abi::PortBindingV1 {
            port: abi::PortHandle {
                value: self.port_handle(context, index),
            },
            topic: abi::Bytes {
                ptr: row.topic.as_ptr(),
                len: row.topic.len() as u64,
            },
            configured_capacity: capacity.get(),
            read_mode: 0,
        })
    }
    pub(super) fn loan(
        &self,
        context: u64,
        port: u64,
        request: abi::LoanRequestV1,
    ) -> Result<abi::WriteSlotV1, HostServiceError> {
        let index = self.port_index(context, port)?;
        self.with_publisher(index, |row, state| {
            self.admit()?;
            if state.panicked {
                return Err(HostServiceError::Panicked);
            }
            let publisher = state.publisher.as_mut().ok_or(LeaseError::InvalidHandle)?;
            let capacity = publisher.max_slice_len().get();
            if request.schema_hash != row.schema_hash
                || request.wire_fixed_size != 0
                || request.variable_count != 0
                || request.alignment != crate::shm_runtime::FRAME_ALIGN as u32
                || request.minimum_capacity < WireHeader::SIZE as u32
                || request.minimum_capacity > capacity
            {
                return Err(HostServiceError::InvalidDescriptor);
            }
            let token = state.leases.claim(port, LeaseDirection::Write)?;
            let mut loan = match publisher.loan_plugin_raw_frame(request.minimum_capacity) {
                Ok(loan) => loan,
                Err(error) => {
                    state.leases.cancel(token)?;
                    return Err(error.into());
                }
            };
            let slot = abi::WriteSlotV1 {
                data: loan.bytes_mut().as_mut_ptr(),
                lease: abi::LeaseHandle { value: token },
                schema_hash: row.schema_hash,
                capacity: request.minimum_capacity,
                configured_capacity: capacity,
                alignment: crate::shm_runtime::FRAME_ALIGN as u32,
                prefix_size: WireHeader::SIZE as u32,
            };
            state.leases.insert(token, loan)?;
            self.active.fetch_add(1, Ordering::Release);
            Ok(slot)
        })
    }
    fn finish(&self, context: u64, token: u64, total: u32) -> Result<u64, HostServiceError> {
        let index = self.lease_index(token)?;
        let port = self.port_handle(context, index);
        self.with_publisher(index, |row, state| {
            self.admit()?;
            let mut loan = state.leases.take(token, port, LeaseDirection::Write)?;
            self.active.fetch_sub(1, Ordering::Release);
            let publisher = state.publisher.as_mut().ok_or(LeaseError::InvalidHandle)?;
            if state.panicked {
                publisher.record_invariant_violation_drop();
                self.invariants.fetch_add(1, Ordering::Relaxed);
                self.discarded.fetch_add(1, Ordering::Relaxed);
                return Err(HostServiceError::Panicked);
            }
            let frame = loan.bytes_mut();
            let header =
                WireHeader::read_from_buf(frame).ok_or(HostServiceError::InvalidDescriptor)?;
            if total as usize != frame.len()
                || header.total_size != total
                || header.schema_hash != row.schema_hash
                || !raw_frame_valid(&header, frame)
            {
                publisher.record_invariant_violation_drop();
                self.invariants.fetch_add(1, Ordering::Relaxed);
                self.discarded.fetch_add(1, Ordering::Relaxed);
                return Err(HostServiceError::InvalidDescriptor);
            }
            let recipients = publisher.commit_plugin_raw_frame(loan)?;
            self.committed.fetch_add(1, Ordering::Relaxed);
            Ok(recipients as u64)
        })
    }
    pub(super) fn discard(
        &self,
        context: u64,
        token: u64,
        reason: u32,
    ) -> Result<(), HostServiceError> {
        let index = self.lease_index(token)?;
        let port = self.port_handle(context, index);
        self.with_publisher(index, |_row, state| {
            self.admit()?;
            let loan = state.leases.take(token, port, LeaseDirection::Write)?;
            self.active.fetch_sub(1, Ordering::Release);
            drop(loan);
            let publisher = state.publisher.as_mut().ok_or(LeaseError::InvalidHandle)?;
            match reason {
                abi::discard_reason::REPLAY_SUPPRESSED => return Ok(()),
                abi::discard_reason::TICK_FAILED | abi::discard_reason::TICK_PANICKED => {
                    publisher.bump_discard_signal()
                }
                abi::discard_reason::MISSING_VARIABLE | abi::discard_reason::NESTED_FAILED => {
                    publisher.record_output_discard();
                }
                abi::discard_reason::INVARIANT => {
                    publisher.record_invariant_violation_drop();
                    self.invariants.fetch_add(1, Ordering::Relaxed);
                }
                abi::discard_reason::SPILL_RELEASE => {
                    publisher.record_invariant_violation_drop();
                    self.invariants.fetch_add(1, Ordering::Relaxed);
                    self.discarded.fetch_add(1, Ordering::Relaxed);
                    return Err(HostServiceError::InvalidDescriptor);
                }
                _ => {
                    publisher.record_invariant_violation_drop();
                    self.invariants.fetch_add(1, Ordering::Relaxed);
                    self.discarded.fetch_add(1, Ordering::Relaxed);
                    return Err(HostServiceError::InvalidDescriptor);
                }
            }
            self.discarded.fetch_add(1, Ordering::Relaxed);
            Ok(())
        })
    }
    fn register(&self, context: u64, port: u64) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port)?;
        self.with_publisher(index, |row, state| {
            self.admit()?;
            if state.local_scope.is_some() {
                return Ok(());
            }
            self.manager
                .as_ref()
                .ok_or(HostServiceError::MissingCapability)?
                .register_dynamic_egress_topic(&row.topic, row.schema_hash)?;
            Ok(())
        })
    }
    fn destroy(&self, context: u64, port: u64) -> Result<(), HostServiceError> {
        let index = self.port_index(context, port)?;
        self.with_publisher(index, |_row, state| {
            self.admit()?;
            if state.leases.active() != 0 {
                return Err(HostServiceError::DestroyBusy);
            }
            state.publisher.take();
            state.local_scope.take();
            Ok(())
        })
    }
    pub(super) fn revoke(&self) {
        self.retired.store(true, Ordering::Release);
        for row in self.rows.iter().filter_map(OnceLock::get) {
            let mut state = row
                .state
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let PublisherState {
                publisher,
                leases,
                local_scope,
                ..
            } = &mut *state;
            leases.revoke_with(|_, _, owner| {
                if let Some(publisher) = publisher {
                    publisher.record_invariant_violation_drop();
                    self.invariants.fetch_add(1, Ordering::Relaxed);
                }
                self.discarded.fetch_add(1, Ordering::Relaxed);
                self.active.fetch_sub(1, Ordering::Release);
                drop(owner);
            });
            publisher.take();
            local_scope.take();
        }
    }
    pub(super) fn record_error(&self, error: HostServiceError) -> HostCallFailure {
        let mut diagnostic = self
            .diagnostic
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let token = match diagnostic.token.checked_add(1).filter(|n| *n < (1 << 63)) {
            Some(next) => {
                diagnostic.token = next;
                next | (1 << 63)
            }
            None => {
                self.retired.store(true, Ordering::Release);
                0
            }
        };
        let result = classify_failure(&error, token);
        diagnostic.error = Some(error);
        result
    }
    pub(super) fn copy_error(
        &self,
        token: u64,
        target: &mut [u8],
    ) -> Result<usize, HostServiceError> {
        let diagnostic = self
            .diagnostic
            .lock()
            .map_err(|_| HostServiceError::Panicked)?;
        if token == 0 || token != diagnostic.token | (1 << 63) {
            return Err(LeaseError::InvalidHandle.into());
        }
        copy_diagnostic_bytes(
            diagnostic.error.as_ref().ok_or(LeaseError::InvalidHandle)?,
            target,
        )
    }
}
fn raw_frame_valid(header: &WireHeader, frame: &[u8]) -> bool {
    let Some(table_size) = header.offset_table_count.checked_mul(8) else {
        return false;
    };
    let Some(prefix) = header.offset_table_offset.checked_add(table_size) else {
        return false;
    };
    if header.offset_table_offset < WireHeader::SIZE as u32 || prefix > header.total_size {
        return false;
    }
    PluginWireLayout {
        schema_hash: header.schema_hash,
        fixed_size: header.offset_table_offset - WireHeader::SIZE as u32,
        variable_fields: header.offset_table_count,
        alignment: crate::shm_runtime::FRAME_ALIGN as u32,
    }
    .accepts_payload(&frame[WireHeader::SIZE..], header.total_size)
}
fn raw_status(
    context: u64,
    operation: impl FnOnce(&RawServices) -> Result<abi::Status, HostServiceError>,
) -> abi::Status {
    let services = match raw_owner(context) {
        Ok(services) => services,
        Err(error) => return call_status(Err(classify_failure(&error, 0))),
    };
    match catch_unwind(AssertUnwindSafe(|| operation(&services))) {
        Ok(Ok(value)) => value,
        Ok(Err(error)) => call_status(Err(services.record_error(error))),
        Err(payload) => {
            contain_panic_payload(payload);
            call_status(Err(services.record_error(HostServiceError::Panicked)))
        }
    }
}
macro_rules! raw_callback {
    ($name:ident($context:ident $(, $arg:ident: $ty:ty)*) |$services:ident, $memory:ident| $body:block) => {
        pub(super) unsafe extern "C" fn $name($context: abi::ContextHandle, $($arg: $ty),*) -> abi::Status {
            match catch_unwind(AssertUnwindSafe(|| {
                raw_status($context.value, |$services| { let $memory = CallerMemory; $body })
            })) { Ok(result) => result, Err(payload) => { contain_panic_payload(payload); status(abi::status_code::PANICKED) } }
        }
    };
}
raw_callback!(callback_clock(context, source: u32, out: *mut abi::ClockValueV1) |services, memory| {
    services.admit()?;
    let out = memory.output(out)?;
    let time = match source {
        abi::clock_source::ACTIVE => Some(services.clock.now_ns()),
        abi::clock_source::VIRTUAL => services.clock.virt_ns(),
        abi::clock_source::EXTERNAL => services.clock.ext_ns(),
        abi::clock_source::REAL => Some(crate::clock::real_ns()),
        _ => return Err(HostServiceError::InvalidDescriptor),
    };
    out.write(abi::ClockValueV1 { timestamp_ns: time.unwrap_or(0), present: u32::from(time.is_some()), reserved: 0 });
    Ok(status(abi::status_code::OK))
});
raw_callback!(callback_create_raw(context, request: abi::RawPublisherRequestV1, out: *mut abi::PortBindingV1) |services, memory| {
    let out = memory.output(out)?;
    let topic = std::str::from_utf8(memory.bytes(request.topic)?).map_err(|_| HostServiceError::InvalidDescriptor)?;
    out.write(services.create(context.value, topic, request)?); Ok(status(abi::status_code::OK))
});
raw_callback!(callback_register_raw(context, port: abi::PortHandle) |services, _memory| {
    services.register(context.value, port.value)?; Ok(status(abi::status_code::OK))
});
raw_callback!(callback_destroy_raw(context, port: abi::PortHandle) |services, _memory| {
    services.destroy(context.value, port.value)?; Ok(status(abi::status_code::OK))
});
raw_callback!(callback_loan_raw(context, port: abi::PortHandle, request: abi::LoanRequestV1, out: *mut abi::WriteSlotV1) |services, memory| {
    let out = memory.output(out)?; out.write(services.loan(context.value, port.value, request)?); Ok(status(abi::status_code::OK))
});
raw_callback!(callback_discard_raw(context, token: abi::LeaseHandle, reason: u32) |services, _memory| {
    services.discard(context.value, token.value, reason)?; Ok(status(abi::status_code::OK))
});
pub(super) unsafe extern "C" fn callback_finish_raw(
    context: abi::ContextHandle,
    lease: abi::LeaseHandle,
    total: u32,
    recipients: *mut u64,
) -> abi::Status {
    match catch_unwind(AssertUnwindSafe(|| {
        let memory = CallerMemory;
        let out = match memory.output(recipients) {
            Ok(out) => out,
            Err(error) => {
                return raw_status(context.value, |services| {
                    services.discard(context.value, lease.value, abi::discard_reason::INVARIANT)?;
                    Err(error)
                });
            }
        };
        out.write(0);
        raw_status(context.value, |services| {
            out.write(services.finish(context.value, lease.value, total)?);
            Ok(status(abi::status_code::OK))
        })
    })) {
        Ok(result) => result,
        Err(payload) => {
            contain_panic_payload(payload);
            status(abi::status_code::PANICKED)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::VirtualClock;
    use crate::graph::node::ShutdownSignal;
    use indexmap::IndexMap;

    #[test]
    fn exhausted_raw_diagnostics_never_restart_the_token_generation() {
        let raw = RawServices::new(None, Arc::new(VirtualClock::new()), 0, 0, 0).unwrap();
        raw.diagnostic.lock().unwrap().token = (1 << 63) - 1;
        for _ in 0..2 {
            assert_eq!(
                raw.record_error(HostServiceError::InvalidDescriptor)
                    .error_token,
                0
            );
            assert_eq!(raw.diagnostic.lock().unwrap().token, (1 << 63) - 1);
            assert!(raw.retired.load(Ordering::Acquire));
            assert!(matches!(
                raw.copy_error((1 << 63) | 1, &mut []),
                Err(HostServiceError::Lease(LeaseError::InvalidHandle))
            ));
        }
    }

    #[test]
    fn native_raw_send_failure_retires_the_lease_and_next_frame_is_delivered() {
        let clock = Arc::new(VirtualClock::new());
        let manager = crate::transport::TransportManager::init_for_test(
            crate::transport::TransportConfig {
                node_name: "raw_failure_oracle".into(),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            crate::testing::iceoryx_test_config(),
        )
        .unwrap();
        let env = Arc::new(HashMap::new());
        let mut native = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock,
            ShutdownSignal::new(),
            env.clone(),
        );
        native.set_transport(manager.clone());
        let host = HostContext::new(native, Vec::new(), env, 1)
            .unwrap()
            .with_raw_publisher_budget(1)
            .unwrap();
        let context = register_context(host).unwrap();
        let topic = b"abi/raw/fault";
        let schema_hash = 0x1122_3344_5566_7788;
        let raw = raw_owner(context).unwrap();
        let binding = raw
            .create(
                context,
                "abi/raw/fault",
                abi::RawPublisherRequestV1 {
                    topic: abi::Bytes {
                        ptr: topic.as_ptr(),
                        len: topic.len() as u64,
                    },
                    schema_hash,
                    configured_capacity: 64,
                    full_queue: abi::full_queue::DISCARD,
                },
            )
            .unwrap();
        let mut subscriber = manager.create_subscriber("abi/raw/fault").unwrap();
        raw.with_publisher(0, |_, state| {
            state
                .publisher
                .as_mut()
                .unwrap()
                .fault_inject_send_raw_loan();
            Ok(())
        })
        .unwrap();
        for (sequence, fail) in [(100, true), (101, false)] {
            let mut expected = [0; 64];
            WireHeader {
                schema_hash,
                total_size: 64,
                offset_table_offset: 64,
                offset_table_count: 0,
                sequence,
                timestamp_ns: 777,
                step_tag: crate::wire::StepTag::UNTAGGED,
            }
            .write_to_buf(&mut expected);
            expected[40..].copy_from_slice(&[1; 24]);
            let slot = raw
                .loan(
                    context,
                    binding.port.value,
                    abi::LoanRequestV1 {
                        schema_hash,
                        minimum_capacity: 64,
                        wire_fixed_size: 0,
                        variable_count: 0,
                        alignment: crate::shm_runtime::FRAME_ALIGN as u32,
                    },
                )
                .unwrap();
            raw.with_publisher(0, |_, state| {
                state
                    .leases
                    .get_mut(slot.lease.value, binding.port.value, LeaseDirection::Write)?
                    .bytes_mut()
                    .copy_from_slice(&expected);
                Ok(())
            })
            .unwrap();
            let result = raw.finish(context, slot.lease.value, 64);
            if fail {
                assert_eq!(
                    classify_failure(&result.unwrap_err(), 0).status,
                    abi::status_code::FAILED
                );
                assert!(subscriber.view_raw().unwrap().is_none());
            } else {
                assert_eq!(result.unwrap(), 1);
                let frame = subscriber.view_raw_expecting(schema_hash).unwrap().unwrap();
                assert_eq!(&frame[..], &expected);
            }
            assert_eq!(
                with_context(context, |host| Ok(host.outstanding_leases())),
                Ok(0)
            );
        }
        remove_context(context).unwrap();
    }
}

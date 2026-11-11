// SPDX-License-Identifier: AGPL-3.0-only
//! Host-local raw ingress owners. Creation is cold; frame leases share the
//! fixed context arena and use the existing native ingress send/notify path.
use super::*;
use crate::transport::publisher::CerulionPublisher;
use crate::transport::FullQueueSend;

pub(super) struct RawPort {
    // Retain topic storage even after destroy: PortBinding promises context lifetime.
    topic: String,
    schema_hash: u64,
    pub(super) publisher: Option<CerulionPublisher>,
    pub(super) busy: bool,
}

impl HostContext {
    /// Provision a different bounded raw publisher budget before registration.
    /// No raw handle has been published while this owned builder runs.
    // hot-path-alloc-ok-fn: fixed publisher rows are provisioned only at context construction
    pub fn with_raw_publisher_budget(mut self, budget: usize) -> Result<Self, HostServiceError> {
        if self.raw_created != 0
            || self
                .ports
                .len()
                .checked_add(budget)
                .is_none_or(|n| n >= u32::MAX as usize)
        {
            return Err(HostServiceError::InvalidDescriptor);
        }
        self.raw_ports = (0..budget)
            .map(|_| None)
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Ok(self)
    }

    pub(super) fn raw_candidate(&self, context: u64, port: u64) -> bool {
        context != 0
            && port & 0xffff_ffff_0000_0000 == context & 0xffff_ffff_0000_0000
            && port as u32 as usize > self.ports.len()
    }

    fn raw_index(&self, context: u64, port: u64) -> Result<usize, HostServiceError> {
        self.admit()?;
        if !self.raw_candidate(context, port) {
            return Err(LeaseError::WrongOwner.into());
        }
        let index = (port as u32 as usize)
            .checked_sub(self.ports.len() + 1)
            .ok_or(LeaseError::InvalidHandle)?;
        if self
            .raw_ports
            .get(index)
            .and_then(Option::as_ref)
            .and_then(|row| row.publisher.as_ref())
            .is_none()
        {
            return Err(LeaseError::InvalidHandle.into());
        }
        Ok(index)
    }

    // hot-path-alloc-ok-fn: cold topic copy and native ingress service creation
    fn create_raw(
        &mut self,
        context: u64,
        topic: &str,
        request: abi::RawPublisherRequestV1,
    ) -> Result<abi::PortBindingV1, HostServiceError> {
        self.admit()?;
        let strategy = match request.full_queue {
            abi::full_queue::CONFIG_DEFAULT => FullQueueSend::ConfigDefault,
            abi::full_queue::DISCARD => FullQueueSend::Discard,
            // BLOCK is reserved: the native ingress API has no such override.
            _ => return Err(HostServiceError::InvalidDescriptor),
        };
        if topic.is_empty() || request.configured_capacity < WireHeader::SIZE as u32 {
            return Err(HostServiceError::InvalidDescriptor);
        }
        let capacity = crate::wire::MaxSliceLen::try_new(request.configured_capacity)
            .ok_or(HostServiceError::InvalidDescriptor)?;
        if self.raw_created == self.raw_ports.len() {
            return Err(HostServiceError::Busy);
        }
        let manager = self
            .native
            .transport()
            .ok_or(HostServiceError::MissingCapability)?;
        let publisher = manager.create_ingress_publisher_with(topic, capacity, strategy)?;
        let index = self.raw_created;
        let row = self.raw_ports[index].insert(RawPort {
            topic: publisher.topic().to_owned(),
            schema_hash: request.schema_hash,
            publisher: Some(publisher),
            busy: false,
        });
        self.raw_created += 1;
        Ok(abi::PortBindingV1 {
            port: abi::PortHandle {
                value: context & 0xffff_ffff_0000_0000 | (self.ports.len() + index + 1) as u64,
            },
            topic: abi::Bytes {
                ptr: row.topic.as_ptr(),
                len: row.topic.len() as u64,
            },
            configured_capacity: capacity.get(),
            read_mode: 0,
        })
    }

    pub(super) fn loan_raw(
        &mut self,
        context: u64,
        port: u64,
        request: abi::LoanRequestV1,
    ) -> Result<abi::WriteSlotV1, HostServiceError> {
        let index = self.raw_index(context, port)?;
        let row = self.raw_ports[index]
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let publisher = row
            .publisher
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let capacity = publisher.max_slice_len().get();
        // Raw framing has no typed layout. The caller requests the exact frame
        // length and copies/initializes the entire frame, including its header.
        if request.schema_hash != row.schema_hash
            || request.wire_fixed_size != 0
            || request.variable_count != 0
            || request.alignment != crate::shm_runtime::FRAME_ALIGN as u32
            || request.minimum_capacity < WireHeader::SIZE as u32
            || request.minimum_capacity > capacity
        {
            return Err(HostServiceError::InvalidDescriptor);
        }
        if row.busy {
            return Err(HostServiceError::Busy);
        }
        let token = self.leases.claim(port, LeaseDirection::Write)?;
        let loan = match publisher.loan_plugin_raw_frame(request.minimum_capacity) {
            Ok(loan) => loan,
            Err(error) => {
                self.leases.cancel(token)?;
                return Err(error.into());
            }
        };
        let slot = abi::WriteSlotV1 {
            data: loan.as_ptr().cast_mut(),
            lease: abi::LeaseHandle { value: token },
            schema_hash: row.schema_hash,
            capacity: request.minimum_capacity,
            configured_capacity: capacity,
            alignment: crate::shm_runtime::FRAME_ALIGN as u32,
            prefix_size: WireHeader::SIZE as u32,
        };
        self.leases.insert(token, NativeLease::RawWrite { loan })?;
        row.busy = true;
        Ok(slot)
    }

    fn finish_raw(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
        total: u32,
    ) -> Result<u64, HostServiceError> {
        let index = self.raw_index(context, port)?;
        // Consume before validation/send so every terminal refusal returns storage.
        let NativeLease::RawWrite { mut loan } =
            self.leases.take(token, port, LeaseDirection::Write)?
        else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        let row = self.raw_ports[index]
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        row.busy = false;
        let publisher = row
            .publisher
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        let frame = loan.bytes_mut();
        let Some(header) = WireHeader::read_from_buf(frame) else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        if total as usize != frame.len()
            || header.total_size != total
            || header.schema_hash != row.schema_hash
            || !raw_frame_valid(&header, frame)
        {
            publisher.record_invariant_violation_drop();
            return Err(HostServiceError::InvalidDescriptor);
        }
        let next = self
            .counters
            .committed
            .checked_add(1)
            .ok_or(LeaseError::GenerationExhausted)?;
        let recipients = publisher.commit_plugin_raw_frame(loan)?;
        self.counters.committed = next;
        Ok(recipients as u64)
    }

    pub(super) fn discard_raw(
        &mut self,
        context: u64,
        port: u64,
        token: u64,
        reason: HostDiscardReason,
    ) -> Result<(), HostServiceError> {
        let index = self.raw_index(context, port)?;
        if reason == HostDiscardReason::SpillRelease {
            return Err(LeaseError::InvalidTransition.into());
        }
        // Raw ingress never performs adaptive spill or consumes host wire sequences.
        let NativeLease::RawWrite { loan } =
            self.leases.take(token, port, LeaseDirection::Write)?
        else {
            return Err(HostServiceError::InvalidDescriptor);
        };
        self.raw_ports[index]
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?
            .busy = false;
        drop(loan);
        if reason != HostDiscardReason::ReplaySuppressed {
            self.counters.discarded = self
                .counters
                .discarded
                .checked_add(1)
                .ok_or(LeaseError::GenerationExhausted)?;
        }
        Ok(())
    }

    fn register_raw_egress(&self, context: u64, port: u64) -> Result<(), HostServiceError> {
        let index = self.raw_index(context, port)?;
        let row = self.raw_ports[index]
            .as_ref()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        self.native
            .transport()
            .ok_or(HostServiceError::MissingCapability)?
            .register_dynamic_egress_topic(&row.topic, row.schema_hash)?;
        Ok(())
    }

    fn destroy_raw(&mut self, context: u64, port: u64) -> Result<(), HostServiceError> {
        let index = self.raw_index(context, port)?;
        let row = self.raw_ports[index]
            .as_mut()
            .ok_or(HostServiceError::InvalidDescriptor)?;
        if row.busy {
            return Err(HostServiceError::Busy);
        }
        row.publisher.take();
        Ok(())
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
    let layout = PluginWireLayout {
        schema_hash: header.schema_hash,
        fixed_size: header.offset_table_offset - WireHeader::SIZE as u32,
        variable_fields: header.offset_table_count,
        alignment: crate::shm_runtime::FRAME_ALIGN as u32,
    };
    layout.accepts_payload(&frame[WireHeader::SIZE..], header.total_size)
}

host_callback!(callback_create_raw(context, request: abi::RawPublisherRequestV1, out: *mut abi::PortBindingV1) |host, memory| {
    let out = memory.output(out)?;
    let topic = std::str::from_utf8(memory.bytes(request.topic)?).map_err(|_| HostServiceError::InvalidDescriptor)?;
    out.write(host.create_raw(context.value, topic, request)?);
    Ok(status(abi::status_code::OK))
});
// Initialize the recipient output before context admission, so stale/foreign,
// terminal, concurrent and panicking calls also return zero recipients.
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
            Err(error) => return call_status(Err(classify_failure(&error, 0))),
        };
        out.write(0);
        call_status(with_context(context.value, |host| {
            let port = lease_port(host, lease, LeaseDirection::Write)?;
            let count = host.finish_raw(context.value, port, lease.value, total)?;
            out.write(count);
            Ok(status(abi::status_code::OK))
        }))
    })) {
        Ok(result) => result,
        Err(payload) => {
            contain_panic_payload(payload);
            status(abi::status_code::PANICKED)
        }
    }
}

host_callback!(callback_register_raw(context, port: abi::PortHandle) |host, _memory| {
    host.register_raw_egress(context.value, port.value)?;
    Ok(status(abi::status_code::OK))
});
host_callback!(callback_destroy_raw(context, port: abi::PortHandle) |host, _memory| {
    host.destroy_raw(context.value, port.value)?;
    Ok(status(abi::status_code::OK))
});

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::VirtualClock;
    use crate::graph::node::ShutdownSignal;
    use indexmap::IndexMap;

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
        let binding = with_context(context, |host| {
            host.create_raw(
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
        })
        .unwrap();
        let mut subscriber = manager.create_subscriber("abi/raw/fault").unwrap();
        with_context(context, |host| {
            let index = host.raw_index(context, binding.port.value)?;
            host.raw_ports[index]
                .as_mut()
                .unwrap()
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
            let result = with_context(context, |host| {
                let slot = host.loan_raw(
                    context,
                    binding.port.value,
                    abi::LoanRequestV1 {
                        schema_hash,
                        minimum_capacity: 64,
                        wire_fixed_size: 0,
                        variable_count: 0,
                        alignment: crate::shm_runtime::FRAME_ALIGN as u32,
                    },
                )?;
                let NativeLease::RawWrite { loan } = host.leases.get_mut(
                    slot.lease.value,
                    binding.port.value,
                    LeaseDirection::Write,
                )?
                else {
                    panic!("native raw owner");
                };
                loan.bytes_mut().copy_from_slice(&expected);
                host.finish_raw(context, binding.port.value, slot.lease.value, 64)
            });
            if fail {
                assert_eq!(result.unwrap_err().status, abi::status_code::FAILED);
                assert!(subscriber.view_raw().unwrap().is_none());
            } else {
                assert_eq!(result, Ok(1));
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

// SPDX-License-Identifier: AGPL-3.0-only
//! Shared author API for raw ingress; plugin transport ownership stays opaque.
use super::{bytes, HostClient};
use crate::error::TransportResult;
use crate::node_abi::{
    discard_reason, full_queue, LoanRequestV1, PortHandle, RawPublisherRequestV1, WriteSlotV1,
};
use crate::transport::publisher::CerulionPublisher;
use crate::transport::{FullQueueSend, TransportManager};
use crate::wire::{MaxSliceLen, WireHeader};
use std::sync::Arc;

/// Cloneable raw-ingress factory. Construction creates no transport session.
#[derive(Clone)]
pub struct RawIngress {
    backend: Backend,
}
#[derive(Clone)]
enum Backend {
    Native(Arc<TransportManager>),
    Plugin(HostClient),
}
impl RawIngress {
    /// Wrap the existing process transport for an in-process node.
    pub fn native(transport: Arc<TransportManager>) -> Self {
        Self {
            backend: Backend::Native(transport),
        }
    }
    pub(super) fn plugin(client: HostClient) -> Self {
        Self {
            backend: Backend::Plugin(client),
        }
    }
    /// Create a publisher using the existing ingress configuration and QoS.
    // hot-path-alloc-ok-fn: cold topic creation copies names and configures one bounded host port.
    pub fn create_publisher(
        &self,
        topic: &str,
        schema_hash: u64,
        maximum: MaxSliceLen,
        full_queue_send: FullQueueSend,
    ) -> TransportResult<RawPublisher> {
        match &self.backend {
            Backend::Native(manager) => Ok(RawPublisher {
                backend: PublisherBackend::Native {
                    publisher: Box::new(manager.create_ingress_publisher_with(
                        topic,
                        maximum,
                        full_queue_send,
                    )?),
                    manager: Arc::clone(manager),
                    topic: Arc::from(topic),
                    schema_hash,
                },
            }),
            Backend::Plugin(client) => {
                let request = RawPublisherRequestV1 {
                    topic: super::client::bytes_span(topic.as_bytes()),
                    schema_hash,
                    configured_capacity: maximum.get(),
                    full_queue: match full_queue_send {
                        FullQueueSend::ConfigDefault => full_queue::CONFIG_DEFAULT,
                        FullQueueSend::Discard => full_queue::DISCARD,
                    },
                };
                let binding = client.create_raw(request)?;
                let mut publisher = PluginPublisher {
                    client: HostClient::share(client),
                    handle: binding.port,
                    topic: Arc::from(topic),
                    hash: schema_hash,
                    maximum,
                };
                if binding.configured_capacity != maximum.get() || binding.read_mode != 0 {
                    return Err(super::argument_error(
                        "host raw publisher binding has invalid capacity or direction",
                    ));
                }
                // SAFETY (U12): successful binding borrows an immutable topic
                // for the context lifetime; validate/copy it only at creation.
                let resolved = unsafe { bytes::read(binding.topic.ptr, binding.topic.len, 1) }
                    .map_err(|error| super::span_error(topic, error))?;
                publisher.topic = Arc::from(
                    std::str::from_utf8(resolved)
                        .map_err(|_| super::argument_error("raw topic is not UTF-8"))?,
                );
                Ok(RawPublisher {
                    backend: PublisherBackend::Plugin(publisher),
                })
            }
        }
    }
}

/// Local publisher owner; plugin samples and native ports stay host-owned.
pub struct RawPublisher {
    backend: PublisherBackend,
}
enum PublisherBackend {
    Native {
        publisher: Box<CerulionPublisher>,
        manager: Arc<TransportManager>,
        topic: Arc<str>,
        schema_hash: u64,
    },
    Plugin(PluginPublisher),
}
struct PluginPublisher {
    client: HostClient,
    handle: PortHandle,
    topic: Arc<str>,
    hash: u64,
    maximum: MaxSliceLen,
}
impl Drop for PluginPublisher {
    fn drop(&mut self) {
        self.client.destroy_raw(self.handle);
    }
}
struct RawLease {
    client: HostClient,
    slot: WriteSlotV1,
    active: bool,
    reason: u32,
}
impl Drop for RawLease {
    fn drop(&mut self) {
        if self.active {
            self.client.discard_output(
                self.slot.lease,
                if std::thread::panicking() {
                    discard_reason::TICK_PANICKED
                } else {
                    self.reason
                },
            );
        }
    }
}
impl RawPublisher {
    /// Copy one complete caller-stamped frame into a loan and return recipients.
    /// The stable host preserves its sequence and timestamp without restamping.
    pub fn publish_raw(&mut self, frame: &[u8]) -> TransportResult<usize> {
        match &mut self.backend {
            PublisherBackend::Native {
                publisher,
                topic,
                schema_hash,
                ..
            } => {
                crate::transport::input_view::validate_wire_frame_raw(
                    topic,
                    frame,
                    Some(*schema_hash),
                )?;
                publisher.publish_raw(frame)
            }
            PublisherBackend::Plugin(publisher) => {
                crate::transport::input_view::validate_wire_frame_raw(
                    &publisher.topic,
                    frame,
                    Some(publisher.hash),
                )?;
                let size = u32::try_from(frame.len())
                    .map_err(|_| super::argument_error("raw frame exceeds u32"))?;
                if size > publisher.maximum.get() {
                    return Err(super::argument_error(
                        "raw frame exceeds configured capacity",
                    ));
                }
                let request = LoanRequestV1 {
                    schema_hash: publisher.hash,
                    minimum_capacity: size,
                    wire_fixed_size: 0,
                    variable_count: 0,
                    alignment: crate::shm_runtime::FRAME_ALIGN as u32,
                };
                let slot = publisher.client.loan_frame(publisher.handle, request)?;
                let mut lease = RawLease {
                    client: HostClient::share(&publisher.client),
                    slot,
                    active: true,
                    reason: discard_reason::INVARIANT,
                };
                if slot.capacity < size
                    || slot.capacity > slot.configured_capacity
                    || slot.configured_capacity != publisher.maximum.get()
                    || slot.prefix_size != WireHeader::SIZE as u32
                    || slot.schema_hash != publisher.hash
                    || slot.alignment < crate::shm_runtime::FRAME_ALIGN as u32
                {
                    return Err(super::argument_error(
                        "raw ingress loan descriptor is invalid",
                    ));
                }
                // SAFETY (U09): validated exclusive lease pins the initialized
                // writable slot through copy; the borrow ends before terminal
                // publish or discard, including every validation failure.
                let output = unsafe {
                    bytes::write(slot.data, size.into(), crate::shm_runtime::FRAME_ALIGN)
                }
                .map_err(|error| super::span_error(&publisher.topic, error))?;
                output.copy_from_slice(frame);
                lease.reason = discard_reason::TICK_FAILED;
                lease.active = false;
                publisher.client.finish_raw(slot.lease, size)
            }
        }
    }
    /// Announce dynamic egress; a failure leaves local delivery usable.
    pub fn register_egress(&self) -> TransportResult<()> {
        match &self.backend {
            PublisherBackend::Native {
                manager,
                topic,
                schema_hash,
                ..
            } => manager
                .register_dynamic_egress_topic(topic, *schema_hash)
                .map(|_| ()),
            PublisherBackend::Plugin(publisher) => publisher.client.register_raw(publisher.handle),
        }
    }
}

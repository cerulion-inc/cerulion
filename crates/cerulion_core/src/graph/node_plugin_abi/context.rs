// SPDX-License-Identifier: AGPL-3.0-only
//! Node-local context, port names and event reconstruction.

use std::str::FromStr;
use std::sync::Arc;

use indexmap::IndexMap;

use crate::clock::Clock;
use crate::error::TransportResult;
use crate::graph::node::{BackpressurePolicy, NodeInfo, ShutdownSignal};
use crate::node_abi::{direction, event_detail, event_kind, PortHandle};
use crate::scheduler::{
    BackpressureEvent, ExpectWithinEvent, LivelinessCause, LivelinessEvent, LivelinessState,
    PromiseWithinEvent,
};
use crate::wire::MaxSliceLen;

use super::HostClient;

/// Publisher identity and immutable configuration held by this node.
/// No native publisher, transport manager or host Rust object is present.
pub struct AnyPublisher {
    pub(super) client: HostClient,
    pub(super) handle: PortHandle,
    pub(super) topic: Arc<str>,
    pub(super) name: Arc<str>,
    pub(super) maximum: MaxSliceLen,
}

impl AnyPublisher {
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// Subscriber identity; all queue and held-frame state stays on the host.
pub struct AnySubscriber {
    pub(super) client: HostClient,
    pub(super) handle: PortHandle,
    pub(super) topic: Arc<str>,
    pub(super) name: Arc<str>,
    pub(super) read_mode: u32,
}

impl AnySubscriber {
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

struct HostClock(HostClient);

impl Clock for HostClock {
    fn now_ns(&self) -> u64 {
        self.0.clock_now()
    }
    fn virt_ns(&self) -> Option<u64> {
        self.0.clock_virtual()
    }
    fn ext_ns(&self) -> Option<u64> {
        self.0.clock_external()
    }
}

/// Context built locally from checked host callbacks and integer port handles.
/// Used only by the generated plugin init/tick adapters.
pub struct NodeContext {
    pub(super) client: HostClient,
    publishers: IndexMap<String, AnyPublisher>,
    subscribers: IndexMap<String, AnySubscriber>,
    clock: Arc<dyn Clock>,
    shutdown_signal: ShutdownSignal,
}

impl NodeContext {
    /// Bind declaration-order ports before calling the user's init.
    // hot-path-alloc-ok-fn: create copies names/configuration into node-owned maps once.
    pub fn new(client: HostClient, info: &NodeInfo) -> TransportResult<Self> {
        let mut publishers = IndexMap::new();
        let mut subscribers = IndexMap::new();
        for (index, name) in info.input_names().iter().enumerate() {
            let binding = client.bind_port(
                u32::try_from(index)
                    .map_err(|_| super::argument_error("input index exceeds u32"))?,
                direction::INPUT,
            )?;
            let topic = topic_from_binding(&binding)?;
            if !matches!(
                binding.read_mode,
                crate::node_abi::read_mode::ONE
                    | crate::node_abi::read_mode::HELD
                    | crate::node_abi::read_mode::LATEST
            ) {
                return Err(super::argument_error("host input read mode is invalid"));
            }
            subscribers.insert(
                name.clone(),
                AnySubscriber {
                    client: client.clone(),
                    handle: binding.port,
                    topic,
                    name: Arc::from(name.as_str()),
                    read_mode: binding.read_mode,
                },
            );
        }
        for (index, name) in info.output_names().iter().enumerate() {
            let binding = client.bind_port(
                u32::try_from(index)
                    .map_err(|_| super::argument_error("output index exceeds u32"))?,
                direction::OUTPUT,
            )?;
            let topic = topic_from_binding(&binding)?;
            let maximum = MaxSliceLen::try_new(binding.configured_capacity)
                .ok_or_else(|| super::argument_error("host output capacity is invalid"))?;
            publishers.insert(
                name.clone(),
                AnyPublisher {
                    client: client.clone(),
                    handle: binding.port,
                    topic,
                    name: Arc::from(name.as_str()),
                    maximum,
                },
            );
        }
        let clock = Arc::new(HostClock(client.clone()));
        Ok(Self {
            client,
            publishers,
            subscribers,
            clock,
            shutdown_signal: ShutdownSignal::new(),
        })
    }

    pub fn split_publishers_subscribers_mut(
        &mut self,
    ) -> (
        &mut IndexMap<String, AnyPublisher>,
        &mut IndexMap<String, AnySubscriber>,
    ) {
        (&mut self.publishers, &mut self.subscribers)
    }

    pub fn publisher(&self, name: &str) -> Option<&AnyPublisher> {
        self.publishers.get(name)
    }
    pub fn publisher_mut(&mut self, name: &str) -> Option<&mut AnyPublisher> {
        self.publishers.get_mut(name)
    }
    pub fn subscriber(&self, name: &str) -> Option<&AnySubscriber> {
        self.subscribers.get(name)
    }
    pub fn subscriber_mut(&mut self, name: &str) -> Option<&mut AnySubscriber> {
        self.subscribers.get_mut(name)
    }
    pub fn publisher_names(&self) -> impl Iterator<Item = &str> {
        self.publishers.keys().map(String::as_str)
    }
    pub fn subscriber_names(&self) -> impl Iterator<Item = &str> {
        self.subscribers.keys().map(String::as_str)
    }
    /// Access host-owned raw-ingress publishers without a native transport owner.
    pub fn raw_ingress(&self) -> TransportResult<super::RawIngress> {
        Ok(super::RawIngress::plugin(HostClient::share(&self.client)))
    }

    pub fn clock(&self) -> &Arc<dyn Clock> {
        &self.clock
    }
    pub fn request_shutdown(&self) {
        self.client.request_shutdown();
    }
    pub fn shutdown_signal(&self) -> &ShutdownSignal {
        &self.shutdown_signal
    }

    pub fn env<T: FromStr>(&self, key: &str, default: T) -> T {
        match self.env_opt(key) {
            Some(raw) => match raw.parse() {
                Ok(value) => value,
                Err(_) => {
                    tracing::warn!(env_var = %key, raw = %raw, target_type = std::any::type_name::<T>(),
                        "env var present but failed to parse; using default");
                    default
                }
            },
            None => default,
        }
    }

    // hot-path-alloc-ok-fn: author init materializes a value from the frozen run environment.
    pub fn env_str(&self, key: &str, default: &str) -> String {
        self.env_opt(key).unwrap_or_else(|| default.into())
    }

    // hot-path-alloc-ok-fn: returns the existing author-owned String environment contract.
    pub fn env_opt(&self, key: &str) -> Option<String> {
        self.client.environment(key)
    }

    pub fn take_backpressure_event(&mut self, name: &str) -> Option<BackpressureEvent> {
        let port = self.subscribers.get(name)?;
        let event = self.event(port.handle, event_kind::BACKPRESSURE)?;
        let policy = match event.detail {
            event_detail::DROP_OLDEST => BackpressurePolicy::DropOldest,
            event_detail::BLOCK => BackpressurePolicy::Block,
            event_detail::SAMPLE if event.policy_value > 0 => {
                BackpressurePolicy::Sample(event.policy_value)
            }
            _ => {
                self.client
                    .record_failure(super::argument_error("host backpressure policy is invalid"));
                return None;
            }
        };
        let Ok(capacity) = usize::try_from(event.capacity) else {
            self.client
                .record_failure(super::argument_error("host event capacity exceeds usize"));
            return None;
        };
        Some(BackpressureEvent {
            policy,
            input_name: Arc::clone(&port.name),
            count_total: event.total,
            count_in_regime: event.in_regime,
            dropped: event.dropped,
            regime_started_at_ns: event.timestamp_ns,
            buffer_capacity: capacity,
        })
    }

    pub fn take_expect_within_event(&mut self, name: &str) -> Option<ExpectWithinEvent> {
        let port = self.subscribers.get(name)?;
        let event = self.event(port.handle, event_kind::EXPECT_WITHIN)?;
        Some(ExpectWithinEvent {
            input_name: Arc::clone(&port.name),
            expect_within_ms: event.window_ms,
            elapsed_ms: event.elapsed_ms,
            count_total: event.total,
            missed_at_ns: event.timestamp_ns,
        })
    }

    pub fn take_promise_within_event(&mut self, name: &str) -> Option<PromiseWithinEvent> {
        let port = self.publishers.get(name)?;
        let event = self.event(port.handle, event_kind::PROMISE_WITHIN)?;
        Some(PromiseWithinEvent {
            output_name: Arc::clone(&port.name),
            promise_within_ms: event.window_ms,
            elapsed_ms: event.elapsed_ms,
            count_total: event.total,
            missed_at_ns: event.timestamp_ns,
        })
    }

    pub fn take_liveliness_event(&mut self, name: &str) -> Option<LivelinessEvent> {
        let port = self.subscribers.get(name)?;
        let event = self.event(port.handle, event_kind::LIVELINESS)?;
        let state = match event.detail {
            event_detail::ALIVE => LivelinessState::Alive,
            event_detail::LOST => LivelinessState::Lost,
            _ => {
                self.client
                    .record_failure(super::argument_error("host liveliness state is invalid"));
                return None;
            }
        };
        let cause = match event.cause {
            event_detail::CONNECTED => LivelinessCause::PublisherConnected,
            event_detail::DISCONNECTED => LivelinessCause::PublisherDisconnected,
            _ => {
                self.client
                    .record_failure(super::argument_error("host liveliness cause is invalid"));
                return None;
            }
        };
        Some(LivelinessEvent {
            input_name: Arc::clone(&port.name),
            state,
            cause,
            count_total: event.total,
            changed_at_ns: event.timestamp_ns,
            publisher_count: usize::try_from(event.capacity).ok()?,
        })
    }

    fn event(&self, port: PortHandle, kind: u32) -> Option<crate::node_abi::EventV1> {
        match self.client.poll_event(port, kind) {
            Ok(event) => event,
            Err(error) => {
                self.client.record_failure(error);
                None
            }
        }
    }
}

// hot-path-alloc-ok-fn: create copies the immutable context-owned topic once.
fn topic_from_binding(binding: &crate::node_abi::PortBindingV1) -> TransportResult<Arc<str>> {
    // SAFETY (U12): host binding promises immutable context-lifetime topic
    // storage. Create holds that context and copies the bytes before returning.
    let bytes = unsafe { super::bytes::read(binding.topic.ptr, binding.topic.len, 1) }
        .map_err(|error| super::span_error("port topic", error))?;
    let topic =
        std::str::from_utf8(bytes).map_err(|_| super::argument_error("host topic is not UTF-8"))?;
    Ok(Arc::from(topic))
}

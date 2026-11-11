// SPDX-License-Identifier: AGPL-3.0-only
//! Process-independent exclusion of local-only topics from every remote plane.
//!
//! An excluded producer holds a notifier before creating its data service.
//! A remote reader or writer holds a listener until after its data port drops.
//! The two admissions fence before scanning the opposite registry, exactly like
//! mirror-origin source leases. Neither side can admit across the other, even
//! when a remote tap retains a data service across a producer replacement.
//! These ports never send events and add no work to publish or receive.

use iceoryx2::node::Node;
use iceoryx2::port::{listener::Listener, notifier::Notifier};
use iceoryx2::prelude::*;
use iceoryx2::service::attribute::AttributeVerifier;

use super::CerService;
use crate::error::{TransportError, TransportResult};

pub(crate) static REMOTE_ACCESS_REFUSALS: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

const PREFIX: &str = "/__cerulion/egress_scope/";
type Marker = iceoryx2::service::port_factory::event::PortFactory<CerService>;

// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
fn error(topic: &str, reason: impl std::fmt::Display) -> TransportError {
    TransportError::EgressScope {
        topic: super::mirror_origin::terminal_safe(topic),
        reason: super::mirror_origin::terminal_safe(&reason.to_string()),
    }
}

/// Reserved marker name; its topic attribute also verifies hash collisions.
// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
pub fn marker_service_name(topic: &str) -> String {
    format!("{PREFIX}{:016x}", crate::wire::fnv1a_hash(topic.as_bytes()))
}

// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
fn verifier(topic: &str) -> TransportResult<AttributeVerifier> {
    super::validate_topic_name(topic).map_err(|e| error(topic, e))?;
    let key = "topic"
        .try_into()
        .map_err(|e| error(topic, format!("{e}")))?;
    let value = topic.try_into().map_err(|e| error(topic, format!("{e}")))?;
    AttributeVerifier::new()
        .require(&key, &value)
        .map_err(|e| error(topic, e))
}

// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
fn name(topic: &str) -> TransportResult<ServiceName> {
    marker_service_name(topic)
        .as_str()
        .try_into()
        .map_err(|e| error(topic, format!("{e}")))
}

// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
fn open(node: &Node<CerService>, topic: &str) -> TransportResult<Marker> {
    node.service_builder(&name(topic)?)
        .event()
        .event_id_max_value(super::CERULION_MAX_EVENT_ID)
        .max_notifiers(64)
        .max_listeners(64)
        .open_or_create_with_attributes(&verifier(topic)?)
        .map_err(|e| error(topic, e))
}

/// Mandatory producer exclusion. Keep it until every run data port has dropped.
#[must_use = "hold the exclusion until all run data ports have dropped"]
pub struct LocalOnlyTopic {
    _notifier: Notifier<CerService>,
    _service: Marker,
}

impl LocalOnlyTopic {
    // hot-path-alloc-ok-fn: cold admission only, never frame delivery.
    pub(crate) fn acquire(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open(node, topic)?;
        let notifier = service
            .notifier_builder()
            .create()
            .map_err(|e| error(topic, e))?;
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_listeners() > 0 {
            return Err(error(topic, "a remote reader or writer still holds this topic; close its demand before starting a local-only producer"));
        }
        Ok(Self {
            _notifier: notifier,
            _service: service,
        })
    }
}

/// Remote admission lease. Its data port must drop before this lease.
#[must_use = "hold the remote lease until its data port drops"]
pub struct RemoteTopicLease {
    _listener: Listener<CerService>,
    _service: Marker,
}

impl RemoteTopicLease {
    // hot-path-alloc-ok-fn: cold admission only, never frame delivery.
    pub(crate) fn acquire(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open(node, topic)?;
        let listener = service
            .listener_builder()
            .create()
            .map_err(|e| error(topic, e))?;
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_notifiers() > 0 {
            return Err(error(topic, "the producing run excludes this topic from network egress; remote access is refused"));
        }
        Ok(Self {
            _listener: listener,
            _service: service,
        })
    }
}

/// Open-only policy check. Missing means no exclusion; every other error refuses.
// hot-path-alloc-ok-fn: cold control-plane check only, never frame delivery.
pub(crate) fn allowed(node: &Node<CerService>, topic: &str) -> TransportResult<bool> {
    use iceoryx2::service::builder::event::EventOpenError;
    match node
        .service_builder(&name(topic)?)
        .event()
        .event_id_max_value(super::CERULION_MAX_EVENT_ID)
        .open_with_attributes(&verifier(topic)?)
    {
        Ok(service) => Ok(service.dynamic_config().number_of_notifiers() == 0),
        Err(EventOpenError::DoesNotExist) => Ok(true),
        Err(e) => Err(error(topic, e)),
    }
}

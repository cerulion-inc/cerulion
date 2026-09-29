// SPDX-License-Identifier: AGPL-3.0-only
//! Required, network-free identity of a network re-injection publisher.
//! Origin attribution remains in `mirror_registry`; its failure cannot turn a
//! marked network publisher into a local source. The guard precedes publisher
//! creation and is retained until after the injection publisher is dropped.

use iceoryx2::node::Node;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::prelude::*;
use iceoryx2::service::attribute::AttributeVerifier;

use super::CerService;
use crate::error::{TransportError, TransportResult};

const PREFIX: &str = "/__cerulion/mirror_origin/";
const TOPIC_ATTRIBUTE: &str = "topic";

fn error(topic: &str, reason: impl std::fmt::Display) -> TransportError {
    TransportError::Internal {
        reason: format!("network mirror identity for '{topic}' could not be established: {reason}"),
    }
}

/// Marker name; the full topic attribute also verifies hash collisions.
// hot-path-alloc-ok-fn: mirror construction and source selection, never frame delivery.
pub fn marker_service_name(topic: &str) -> String {
    format!("{PREFIX}{:016x}", crate::wire::fnv1a_hash(topic.as_bytes()))
}

fn verifier(topic: &str) -> TransportResult<AttributeVerifier> {
    super::validate_topic_name(topic).map_err(|e| error(topic, e))?;
    let key = TOPIC_ATTRIBUTE.try_into().map_err(|e| error(topic, e))?;
    let value = topic.try_into().map_err(|e| error(topic, e))?;
    AttributeVerifier::new()
        .require(&key, &value)
        .map_err(|e| error(topic, e))
}

/// Held only by a remote injector, after that injector's data publisher field.
pub(crate) struct MirrorOrigin {
    _notifier: Notifier<CerService>,
    _service: iceoryx2::service::port_factory::event::PortFactory<CerService>,
}

/// An explicitly local observer's source lease. Drop its subscriber first.
#[must_use = "keep this lease until after the observing subscriber drops"]
pub struct LocalObservationLease {
    _listener: Listener<CerService>,
    _service: iceoryx2::service::port_factory::event::PortFactory<CerService>,
}

impl LocalObservationLease {
    pub(crate) fn open(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open_marker(node, topic)?;
        let listener = service
            .listener_builder()
            .create()
            .map_err(|e| error(topic, e))?;
        // Healthy pinned event registries publish the owner cell before create
        // returns. Both admission sides fence before scanning opposite owners;
        // two EMPTY scans with both ports held would contradict the SC order.
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_notifiers() > 0 {
            return Err(error(
                topic,
                "local observation refused a live network mirror",
            ));
        }
        Ok(Self {
            _listener: listener,
            _service: service,
        })
    }
}

fn open_marker(
    node: &Node<CerService>,
    topic: &str,
) -> TransportResult<iceoryx2::service::port_factory::event::PortFactory<CerService>> {
    let name = marker_service_name(topic)
        .as_str()
        .try_into()
        .map_err(|e| error(topic, e))?;
    node.service_builder(&name)
        .event()
        .max_notifiers(64)
        // Omit max_listeners: retain configured creation capacity (default 16)
        // and the existing open behavior, which imposes no minimum quota.
        .open_or_create_with_attributes(&verifier(topic)?)
        .map_err(|e| error(topic, e))
}

impl MirrorOrigin {
    pub(crate) fn open(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open_marker(node, topic)?;
        let notifier = service
            .notifier_builder()
            .create()
            .map_err(|e| error(topic, e))?;
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_listeners() > 0 {
            return Err(error(
                topic,
                "network mirror refused while a local observer holds its source lease",
            ));
        }
        Ok(Self {
            _notifier: notifier,
            _service: service,
        })
    }
}

/// Open-only identity check. A malformed/incompatible marker fails closed.
pub(crate) fn is_network_mirror(node: &Node<CerService>, topic: &str) -> TransportResult<bool> {
    use iceoryx2::service::builder::event::EventOpenError;
    let name = marker_service_name(topic)
        .as_str()
        .try_into()
        .map_err(|e| error(topic, e))?;
    match node
        .service_builder(&name)
        .event()
        .open_with_attributes(&verifier(topic)?)
    {
        Ok(service) => Ok(service.dynamic_config().number_of_notifiers() > 0),
        Err(EventOpenError::DoesNotExist) => Ok(false),
        Err(e) => Err(error(topic, e)),
    }
}

/// Enumerate marker identities on this manager's namespace, without a gather wait.
// hot-path-alloc-ok-fn: one-shot topic listing, never frame delivery.
pub(crate) fn topics(node: &Node<CerService>) -> TransportResult<Vec<String>> {
    let mut candidates: Vec<String> = Vec::new();
    let mut malformed = None;
    <CerService as iceoryx2::service::Service>::list(
        node.config(),
        |service: iceoryx2::service::ServiceDetails<CerService>| {
            if service.static_details.name().as_str().starts_with(PREFIX) {
                let mut found = false;
                for attribute in service.static_details.attributes().iter() {
                    if attribute.key().as_bytes() == TOPIC_ATTRIBUTE.as_bytes() {
                        let Ok(topic) = std::str::from_utf8(attribute.value().as_bytes()) else {
                            malformed = Some(service.static_details.name().as_str().to_string());
                            continue;
                        };
                        if !found
                            && marker_service_name(topic) == service.static_details.name().as_str()
                        {
                            candidates.push(topic.to_string());
                        } else {
                            malformed = Some(service.static_details.name().as_str().to_string());
                        }
                        found = true;
                    }
                }
                if !found {
                    malformed = Some(service.static_details.name().as_str().to_string());
                }
            }
            CallbackProgression::Continue
        },
    )
    .map_err(|e| error("<enumeration>", e))?;
    if let Some(name) = malformed {
        return Err(error(
            "<enumeration>",
            format!("malformed reserved mirror marker '{name}'"),
        ));
    }
    let mut topics = Vec::new();
    for topic in candidates {
        if is_network_mirror(node, &topic)? {
            topics.push(topic);
        }
    }
    topics.sort();
    topics.dedup();
    Ok(topics)
}

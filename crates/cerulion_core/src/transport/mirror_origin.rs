// SPDX-License-Identifier: AGPL-3.0-only
//! Required, network-free identity of a network re-injection publisher.
//! Origin attribution remains in `mirror_registry`; its failure cannot turn a
//! marked network publisher into a local source. The guard precedes publisher
//! creation and is retained until after the injection publisher is dropped.
//!
//! Each role reports its own refusal: a re-injector that cannot establish its
//! identity gets [`TransportError::MirrorIdentity`], a local observer that
//! cannot hold its source gets [`TransportError::LocalObservationLease`], and a
//! reserved marker another process left in an unreadable shape gets
//! [`TransportError::MalformedMirrorMarker`], which names the remedy.

use iceoryx2::node::Node;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::prelude::*;
use iceoryx2::service::attribute::AttributeVerifier;

use super::CerService;
use crate::error::{TransportError, TransportResult};

const PREFIX: &str = "/__cerulion/mirror_origin/";
const TOPIC_ATTRIBUTE: &str = "topic";

/// Native event listener quota per marker, the ceiling on simultaneous
/// explicitly local observers of one topic when the configuration is untouched.
const DEFAULT_LOCAL_OBSERVER_QUOTA: usize = 16;

type MarkerService = iceoryx2::service::port_factory::event::PortFactory<CerService>;

/// Marker service names and topic attributes are written by OTHER processes
/// and the CLI prints these refusals verbatim, so a control character in them
/// would reach the operator's terminal as an escape sequence (screen clear,
/// title or clipboard OSC, BEL). Every control character (C0, DEL, C1) is
/// rendered as its visible `\u{..}` escape at construction; printable text,
/// non-ASCII included, passes through unchanged so the name stays recognisable.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn terminal_safe(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        if c.is_control() {
            out.extend(c.escape_debug());
        } else {
            out.push(c);
        }
    }
    out
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn identity_error(topic: &str, reason: impl std::fmt::Display) -> TransportError {
    TransportError::MirrorIdentity {
        topic: terminal_safe(topic),
        reason: terminal_safe(&reason.to_string()),
    }
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn lease_error(topic: &str, reason: impl std::fmt::Display) -> TransportError {
    TransportError::LocalObservationLease {
        topic: terminal_safe(topic),
        reason: terminal_safe(&reason.to_string()),
    }
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn malformed_error(service: &str, reason: impl std::fmt::Display) -> TransportError {
    TransportError::MalformedMirrorMarker {
        service: terminal_safe(service),
        reason: terminal_safe(&reason.to_string()),
    }
}

/// Marker name; the full topic attribute also verifies hash collisions.
// hot-path-alloc-ok-fn: mirror construction and source selection, never frame delivery.
pub fn marker_service_name(topic: &str) -> String {
    format!("{PREFIX}{:016x}", crate::wire::fnv1a_hash(topic.as_bytes()))
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn verifier(
    topic: &str,
    err: &impl Fn(&str, String) -> TransportError,
) -> TransportResult<AttributeVerifier> {
    super::validate_topic_name(topic).map_err(|e| err(topic, e.to_string()))?;
    let key = TOPIC_ATTRIBUTE
        .try_into()
        .map_err(|e| err(topic, format!("{e}")))?;
    let value = topic.try_into().map_err(|e| err(topic, format!("{e}")))?;
    AttributeVerifier::new()
        .require(&key, &value)
        .map_err(|e| err(topic, e.to_string()))
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn service_name(
    topic: &str,
    err: &impl Fn(&str, String) -> TransportError,
) -> TransportResult<ServiceName> {
    marker_service_name(topic)
        .as_str()
        .try_into()
        .map_err(|e| err(topic, format!("{e}")))
}

/// Held only by a remote injector, after that injector's data publisher field.
pub(crate) struct MirrorOrigin {
    _notifier: Notifier<CerService>,
    _service: MarkerService,
}

/// An explicitly local observer's source lease. Drop its subscriber first.
#[must_use = "keep this lease until after the observing subscriber drops"]
pub struct LocalObservationLease {
    _listener: Listener<CerService>,
    _service: MarkerService,
}

impl LocalObservationLease {
    // hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
    pub(crate) fn open(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open_marker(node, topic, &lease_error)?;
        let listener = service.listener_builder().create().map_err(|e| {
            lease_error(
                topic,
                format!(
                    "{e}; the marker's listener quota (default \
                     {DEFAULT_LOCAL_OBSERVER_QUOTA} local observers per topic) may be \
                     exhausted: release other local observers of this topic and retry"
                ),
            )
        })?;
        // Healthy pinned event registries publish the owner cell before create
        // returns. Both admission sides fence before scanning opposite owners;
        // two EMPTY scans with both ports held would contradict the SC order.
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_notifiers() > 0 {
            return Err(lease_error(
                topic,
                "a live network mirror owns this topic; observe it without local scope so \
                 cerulion-netd can serve it, or read it on its source robot",
            ));
        }
        Ok(Self {
            _listener: listener,
            _service: service,
        })
    }
}

// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn open_marker(
    node: &Node<CerService>,
    topic: &str,
    err: &impl Fn(&str, String) -> TransportError,
) -> TransportResult<MarkerService> {
    let name = service_name(topic, err)?;
    node.service_builder(&name)
        .event()
        // Same event-id ceiling as every other Cerulion event service; see
        // `CERULION_MAX_EVENT_ID`. The marker's ports only count presence, so
        // the ceiling costs nothing here, but one uncapped creator would widen
        // every later opener's bitset walk.
        .event_id_max_value(super::CERULION_MAX_EVENT_ID)
        .max_notifiers(64)
        // Omit max_listeners: retain configured creation capacity (default 16)
        // and the existing open behavior, which imposes no minimum quota.
        .open_or_create_with_attributes(&verifier(topic, err)?)
        .map_err(|e| err(topic, e.to_string()))
}

impl MirrorOrigin {
    // hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
    pub(crate) fn open(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service = open_marker(node, topic, &identity_error)?;
        let notifier = service
            .notifier_builder()
            .create()
            .map_err(|e| identity_error(topic, e.to_string()))?;
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
        if service.dynamic_config().number_of_listeners() > 0 {
            return Err(identity_error(
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

/// Open-only identity check. A malformed/incompatible marker fails closed and
/// names the marker service plus the remedy.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
pub(crate) fn is_network_mirror(node: &Node<CerService>, topic: &str) -> TransportResult<bool> {
    use iceoryx2::service::builder::event::EventOpenError;
    let name = service_name(topic, &identity_error)?;
    match node
        .service_builder(&name)
        .event()
        // On an open the ceiling is a requirement the existing service must
        // meet; see `CERULION_MAX_EVENT_ID`.
        .event_id_max_value(super::CERULION_MAX_EVENT_ID)
        .open_with_attributes(&verifier(topic, &identity_error)?)
    {
        Ok(service) => Ok(service.dynamic_config().number_of_notifiers() > 0),
        Err(EventOpenError::DoesNotExist) => Ok(false),
        Err(e) => Err(malformed_error(
            name.as_str(),
            format!("opening it as the marker of topic '{topic}' failed with {e}"),
        )),
    }
}

/// Enumerate marker identities on this manager's namespace, without a gather wait.
// hot-path-alloc-ok-fn: one-shot topic listing, never frame delivery.
pub(crate) fn topics(node: &Node<CerService>) -> TransportResult<Vec<String>> {
    let mut candidates: Vec<String> = Vec::new();
    let mut malformed: Option<(String, &'static str)> = None;
    <CerService as iceoryx2::service::Service>::list(
        node.config(),
        |service: iceoryx2::service::ServiceDetails<CerService>| {
            let name = service.static_details.name().as_str();
            if name.starts_with(PREFIX) {
                let mut found = false;
                for attribute in service.static_details.attributes().iter() {
                    if attribute.key().as_bytes() == TOPIC_ATTRIBUTE.as_bytes() {
                        let Ok(topic) = std::str::from_utf8(attribute.value().as_bytes()) else {
                            malformed =
                                Some((name.to_string(), "its topic attribute is not UTF-8"));
                            continue;
                        };
                        if !found && marker_service_name(topic) == name {
                            candidates.push(topic.to_string());
                        } else {
                            malformed = Some((
                                name.to_string(),
                                "its topic attribute does not hash to the marker name",
                            ));
                        }
                        found = true;
                    }
                }
                if !found {
                    malformed = Some((name.to_string(), "it carries no topic attribute"));
                }
            }
            CallbackProgression::Continue
        },
    )
    .map_err(|e| TransportError::Internal {
        reason: format!("network mirror marker enumeration failed: {e}"),
    })?;
    if let Some((name, reason)) = malformed {
        return Err(malformed_error(&name, reason));
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

#[cfg(test)]
mod tests {
    use super::*;

    fn has_raw_control(text: &str) -> bool {
        text.chars().any(char::is_control)
    }

    /// A marker name or topic attribute written by another process can carry
    /// ESC, BEL or CR; each refusal renders them as visible escapes, keeps the
    /// printable part of the name intact and leaves ordinary UTF-8 untouched.
    #[test]
    fn refusals_render_foreign_control_characters_as_visible_escapes() {
        let hostile = "/t\u{1b}[2J\u{07}\r/go2-α";
        for error in [
            identity_error(hostile, format!("opening '{hostile}' failed")),
            lease_error(hostile, format!("opening '{hostile}' failed")),
            malformed_error(hostile, format!("opening '{hostile}' failed")),
        ] {
            let text = error.to_string();
            assert!(!has_raw_control(&text), "{text:?}");
            assert_eq!(
                text.matches("/t\\u{1b}[2J\\u{7}\\r/go2-α").count(),
                2,
                "{text}"
            );
        }
        assert_eq!(terminal_safe("/camera/front"), "/camera/front");
        assert_eq!(terminal_safe("a\u{7f}b\u{85}c"), "a\\u{7f}b\\u{85}c");
    }
}

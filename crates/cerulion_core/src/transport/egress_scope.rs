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

/// Internal scope passed by the CLI to opaque ROS children before startup.
#[doc(hidden)]
pub const RUN_SCOPE_ENV: &str = "CERULION_RUN_TOPIC_SCOPE";

/// Producing process policy, resolved before any data endpoint opens.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum RunTopicScope {
    Unrestricted,
    LocalOnly,
    Declared {
        topics: Vec<String>,
        #[serde(default)]
        ingress: Vec<String>,
    },
}

impl RunTopicScope {
    // hot-path-alloc-ok-fn: cold process configuration, never frame delivery.
    pub fn from_values(network: Option<&str>, scope: Option<&str>) -> TransportResult<Self> {
        let network = network.map(str::trim);
        if network.is_some_and(|value| !value.trim().is_empty()) {
            if network != Some("off") {
                tracing::warn!("unrecognized network switch: treating this process as local-only");
            }
            return Ok(Self::LocalOnly);
        }
        match scope {
            None => Ok(Self::Unrestricted),
            Some(json) => serde_json::from_str(json).map_err(|e| error(RUN_SCOPE_ENV, e)),
        }
    }

    pub fn excludes(&self, topic: &str) -> bool {
        match self {
            Self::Unrestricted => false,
            Self::LocalOnly => true,
            Self::Declared { topics, .. } => !topics.iter().any(|allowed| allowed == topic),
        }
    }

    /// Declared ingress admits consumers without permitting those consumers to publish.
    pub fn excludes_input(&self, topic: &str) -> bool {
        self.excludes(topic)
            && !matches!(self, Self::Declared { ingress, .. } if ingress.iter().any(|allowed| allowed == topic))
    }
}

/// One service family scope protects its request and every per-client reply.
// hot-path-alloc-ok-fn: cold endpoint admission, never frame delivery.
pub fn service_scope_topic(service: &str) -> String {
    format!("__cerulion_service_scope/{}", service.trim_matches('/'))
}

// hot-path-alloc-ok-fn: cold endpoint admission, never frame delivery.
fn service_parent_scope(topic: &str) -> Option<String> {
    let service = topic.trim_start_matches('/').strip_prefix("svc/")?;
    let name = service
        .strip_suffix("/request")
        .or_else(|| service.rsplit_once("/reply/").map(|(name, _)| name))?;
    Some(service_scope_topic(name))
}

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
    // Registration and announcement names can exceed native data-port names.
    // Verify their complete identity without putting it in one bounded attribute.
    if topic.is_empty() || topic.len() > super::reg_channel::MAX_REG_TOPIC_LEN {
        return Err(error(
            topic,
            "topic is empty or exceeds the registration name limit",
        ));
    }
    let length_key = "topic_len"
        .try_into()
        .map_err(|e| error(topic, format!("{e}")))?;
    let length = topic
        .len()
        .to_string()
        .as_str()
        .try_into()
        .map_err(|e| error(topic, e))?;
    let mut verifier = AttributeVerifier::new()
        .require(&length_key, &length)
        .map_err(|e| error(topic, e))?;
    let mut remaining = topic;
    let mut index = 0;
    while !remaining.is_empty() {
        let mut end = remaining
            .len()
            .min(iceoryx2::constants::MAX_ATTRIBUTE_VALUE_LENGTH);
        while !remaining.is_char_boundary(end) {
            end -= 1;
        }
        let key = if index == 0 {
            "topic".to_string()
        } else {
            format!("topic_{index}")
        };
        let key = key.as_str().try_into().map_err(|e| error(topic, e))?;
        let value = remaining[..end].try_into().map_err(|e| error(topic, e))?;
        verifier = verifier
            .require(&key, &value)
            .map_err(|e| error(topic, e))?;
        remaining = &remaining[end..];
        index += 1;
    }
    Ok(verifier)
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
            return Err(error(topic, "remote serving still holds this topic; close its demand and withdraw its announcement or restart the serving daemon before starting a local-only producer"));
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
    _service_scope: Option<Box<RemoteTopicLease>>,
    _listener: Listener<CerService>,
    _service: Marker,
}

impl RemoteTopicLease {
    // hot-path-alloc-ok-fn: cold admission only, never frame delivery.
    pub(crate) fn acquire(node: &Node<CerService>, topic: &str) -> TransportResult<Self> {
        let service_scope = service_parent_scope(topic)
            .map(|parent| Self::acquire(node, &parent).map(Box::new))
            .transpose()?;
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
            _service_scope: service_scope,
            _listener: listener,
            _service: service,
        })
    }
}

/// Open-only policy check. Missing means no exclusion; every other error refuses.
// hot-path-alloc-ok-fn: cold control-plane check only, never frame delivery.
pub(crate) fn allowed(node: &Node<CerService>, topic: &str) -> TransportResult<bool> {
    use iceoryx2::service::builder::event::EventOpenError;
    if let Some(parent) = service_parent_scope(topic) {
        if !allowed(node, &parent)? {
            return Ok(false);
        }
    }
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

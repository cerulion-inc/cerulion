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
use iceoryx2::service::builder::event::{EventOpenError, EventOpenOrCreateError};

use super::marker_teardown::{MarkerOpenError, MarkerTeardown, TeardownWait};
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
type MarkerBuilder = iceoryx2::service::builder::event::Builder<CerService>;

/// The marker a refusal names while it is still being torn down.
const POLICY_MARKER: &str = "its policy marker";

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
    // Native attributes accept ASCII only, even though registration names are UTF-8.
    // Preserve ASCII marker compatibility and encode every byte of other identities.
    // A kind attribute distinguishes encoded bytes from a literal ASCII identity.
    let encoded;
    let mut remaining = if topic.is_ascii() {
        topic
    } else {
        let key = "topic_kind".try_into().map_err(|e| error(topic, e))?;
        let kind = "hex".try_into().map_err(|e| error(topic, e))?;
        verifier = verifier.require(&key, &kind).map_err(|e| error(topic, e))?;
        use std::fmt::Write;
        encoded = topic.as_bytes().iter().fold(
            String::with_capacity(topic.len() * 2),
            |mut value, byte| {
                write!(value, "{byte:02x}").expect("writing to a String cannot fail");
                value
            },
        );
        &encoded
    };
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
        .map_err(|e| open_error(topic, e, node.config().global.creation_timeout))
}

/// The refusal for a failed open-or-create of `topic`'s marker. iceoryx2's
/// open-or-create waits out a marker marked for destruction itself, until the
/// node's creation window (`window`) ends, and then returns that state: it is
/// named, and every other error is quoted as iceoryx2 returned it.
// hot-path-alloc-ok-fn: cold admission only, never frame delivery.
fn open_error(
    topic: &str,
    e: EventOpenOrCreateError,
    window: std::time::Duration,
) -> TransportError {
    match e {
        EventOpenOrCreateError::EventOpenError(EventOpenError::IsMarkedForDestruction) => error(
            topic,
            super::marker_teardown::outlasted_creation_window(POLICY_MARKER, window),
        ),
        e => error(topic, e),
    }
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

/// Open-only policy check. Missing means no exclusion. A marker still being
/// torn down is opened again as `wait` allows and refuses, naming that wait,
/// only once it outlasts it; every other error refuses.
// hot-path-alloc-ok-fn: cold control-plane check only, never frame delivery.
pub(crate) fn allowed(
    node: &Node<CerService>,
    topic: &str,
    wait: TeardownWait,
) -> TransportResult<bool> {
    allowed_with(node, topic, wait.budget(), &mut |builder, verifier| {
        builder.open_with_attributes(verifier)
    })
}

/// [`allowed`] with its teardown budget and its open of an existing marker
/// supplied, the open applied to every marker the check reads (a service
/// family's scope first).
// hot-path-alloc-ok-fn: cold control-plane check only, never frame delivery.
fn allowed_with(
    node: &Node<CerService>,
    topic: &str,
    teardown: MarkerTeardown,
    open: &mut dyn FnMut(MarkerBuilder, &AttributeVerifier) -> Result<Marker, EventOpenError>,
) -> TransportResult<bool> {
    if let Some(parent) = service_parent_scope(topic) {
        if !allowed_with(node, &parent, teardown, open)? {
            return Ok(false);
        }
    }
    let name = name(topic)?;
    let verifier = verifier(topic)?;
    let opened = teardown.open(
        || {
            open(
                node.service_builder(&name)
                    .event()
                    .event_id_max_value(super::CERULION_MAX_EVENT_ID),
                &verifier,
            )
        },
        |e| matches!(e, EventOpenError::IsMarkedForDestruction),
    );
    match opened {
        Ok(service) => Ok(service.dynamic_config().number_of_notifiers() == 0),
        Err(MarkerOpenError::Open(EventOpenError::DoesNotExist)) => Ok(true),
        Err(MarkerOpenError::Open(e)) => Err(error(topic, e)),
        Err(MarkerOpenError::TornDown(budget)) => {
            Err(error(topic, budget.outlasted(POLICY_MARKER)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::num::NonZeroU32;
    use std::time::Duration;

    fn node() -> Node<CerService> {
        NodeBuilder::new()
            .config(&crate::testing::iceoryx_test_config())
            .create::<CerService>()
            .unwrap()
    }

    /// A budget of `opens` with no pause between them.
    fn opens(opens: u32) -> MarkerTeardown {
        MarkerTeardown::new(NonZeroU32::new(opens).unwrap(), Duration::ZERO)
    }

    /// [`allowed_with`] whose first `torn` opens report the marker as marked
    /// for destruction and whose later opens are the real open. Returns the
    /// answer and the number of opens.
    fn allowed_after(
        node: &Node<CerService>,
        topic: &str,
        torn: usize,
        teardown: MarkerTeardown,
    ) -> (TransportResult<bool>, usize) {
        let mut calls = 0usize;
        let answer = allowed_with(node, topic, teardown, &mut |builder, verifier| {
            calls += 1;
            if calls <= torn {
                Err(EventOpenError::IsMarkedForDestruction)
            } else {
                builder.open_with_attributes(verifier)
            }
        });
        (answer, calls)
    }

    #[test]
    fn a_policy_marker_being_torn_down_is_waited_out_and_then_read() {
        let node = node();
        // No marker once the teardown ends: the topic is allowed.
        let (answer, calls) = allowed_after(&node, "/teardown/absent", 3, opens(20));
        assert_eq!((answer.unwrap(), calls), (true, 4));
        // A producer's exclusion is read on the budget's last open.
        let _exclusion = LocalOnlyTopic::acquire(&node, "/teardown/excluded").unwrap();
        let (answer, calls) = allowed_after(&node, "/teardown/excluded", 19, opens(20));
        assert_eq!((answer.unwrap(), calls), (false, 20));
        // A remote lease holds no exclusion: still allowed.
        let _lease = RemoteTopicLease::acquire(&node, "/teardown/leased").unwrap();
        let (answer, calls) = allowed_after(&node, "/teardown/leased", 1, opens(20));
        assert_eq!((answer.unwrap(), calls), (true, 2));
        // A service request reads its family's scope marker first, through the
        // same wait: two teardowns, then the scope and the request markers.
        let (answer, calls) = allowed_after(&node, "/svc/teardown/request", 2, opens(20));
        assert_eq!((answer.unwrap(), calls), (true, 4));
    }

    #[test]
    fn a_policy_marker_that_outlasts_the_budget_refuses_naming_the_budget() {
        let node = node();
        let (answer, calls) =
            allowed_after(&node, "/teardown/stuck", usize::MAX, MarkerTeardown::BUDGET);
        assert_eq!(calls, 20);
        assert_eq!(
            answer.unwrap_err().to_string(),
            "Remote access policy for topic '/teardown/stuck' refused: its policy marker was \
             still being torn down after 20 opens 25 ms apart; retry the request"
        );
        // One open short of the budget's end is still waited out.
        let (answer, calls) = allowed_after(&node, "/teardown/stuck", 19, opens(20));
        assert_eq!((answer.unwrap(), calls), (true, 20));
        let (answer, calls) = allowed_after(&node, "/teardown/stuck", 20, opens(20));
        assert_eq!(calls, 20);
        assert!(answer.is_err());
    }

    #[test]
    fn a_check_that_never_waits_opens_once_and_refuses_at_once() {
        let node = node();
        let (answer, calls) = allowed_after(
            &node,
            "/teardown/callback",
            usize::MAX,
            TeardownWait::Never.budget(),
        );
        assert_eq!(calls, 1);
        assert_eq!(
            answer.unwrap_err().to_string(),
            "Remote access policy for topic '/teardown/callback' refused: its policy marker was \
             being torn down, and this check does not wait for it; retry the request"
        );
        // A marker that is not being torn down is read on that one open.
        let _exclusion = LocalOnlyTopic::acquire(&node, "/teardown/callback_excluded").unwrap();
        let (answer, calls) = allowed_after(
            &node,
            "/teardown/callback_excluded",
            0,
            TeardownWait::Never.budget(),
        );
        assert_eq!((answer.unwrap(), calls), (false, 1));
    }

    #[test]
    fn any_other_policy_open_error_refuses_at_once_with_its_text() {
        let node = node();
        let mut calls = 0usize;
        let answer = allowed_with(&node, "/teardown/denied", opens(20), &mut |_, _| {
            calls += 1;
            Err(EventOpenError::InsufficientPermissions)
        });
        assert_eq!(calls, 1);
        assert_eq!(
            answer.unwrap_err().to_string(),
            "Remote access policy for topic '/teardown/denied' refused: \
             EventOpenError::InsufficientPermissions"
        );
    }

    #[test]
    fn an_open_or_create_that_ends_in_a_teardown_names_the_creation_window() {
        let window = Duration::from_millis(500);
        assert_eq!(
            open_error(
                "/x",
                EventOpenOrCreateError::EventOpenError(EventOpenError::IsMarkedForDestruction),
                window,
            )
            .to_string(),
            "Remote access policy for topic '/x' refused: its policy marker was still being torn \
             down when the 500 ms shared-memory creation window ended; retry the request"
        );
        for (e, quoted) in [
            (
                EventOpenOrCreateError::SystemInFlux,
                "EventOpenOrCreateError::SystemInFlux",
            ),
            (
                EventOpenOrCreateError::EventOpenError(EventOpenError::HangsInCreation),
                "EventOpenOrCreateError::EventOpenError(HangsInCreation)",
            ),
        ] {
            assert_eq!(
                open_error("/x", e, window).to_string(),
                format!("Remote access policy for topic '/x' refused: {quoted}")
            );
        }
    }
}

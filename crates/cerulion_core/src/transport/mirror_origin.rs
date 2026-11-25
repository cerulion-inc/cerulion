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
//! [`TransportError::MalformedMirrorMarker`], which names the remedy. A marker
//! whose last holder is dropping it is not malformed: the identity check opens
//! it again within the `marker_teardown` budget and, if it is still being torn
//! down, refuses with [`TransportError::MirrorIdentity`] naming that budget.

use iceoryx2::node::Node;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::prelude::*;
use iceoryx2::service::attribute::AttributeVerifier;
use iceoryx2::service::builder::event::{EventOpenError, EventOpenOrCreateError};

use super::marker_teardown::{MarkerOpenError, MarkerTeardown, TeardownWait};
use super::CerService;
use crate::error::{TransportError, TransportResult};

const PREFIX: &str = "/__cerulion/mirror_origin/";
const TOPIC_ATTRIBUTE: &str = "topic";

/// The marker a refusal names while it is still being torn down.
const MIRROR_MARKER: &str = "its network mirror marker";

/// Native event listener quota per marker, the ceiling on simultaneous
/// explicitly local observers of one topic when the configuration is untouched.
const DEFAULT_LOCAL_OBSERVER_QUOTA: usize = 16;

type MarkerService = iceoryx2::service::port_factory::event::PortFactory<CerService>;
type MarkerBuilder = iceoryx2::service::builder::event::Builder<CerService>;

/// Characters that are not `char::is_control` yet still steer a terminal
/// invisibly: the whole Unicode `Format` (Cf) general category (soft hyphen,
/// Arabic and Syriac marks, Mongolian vowel separator, zero-width and joiner
/// characters, bidirectional embeddings, overrides and isolates, the
/// deprecated U+206A..U+206F shaping controls, the byte-order mark, the
/// interlinear annotation set, Kaithi and Egyptian format marks, musical
/// symbol controls and the tag characters) plus the line and paragraph
/// separators (Zl, Zp). The ranges follow `UnicodeData.txt` for Unicode 15.1.
const fn is_format_char(c: char) -> bool {
    matches!(
        c,
        '\u{00ad}'
            | '\u{0600}'..='\u{0605}'
            | '\u{061c}'
            | '\u{06dd}'
            | '\u{070f}'
            | '\u{0890}'..='\u{0891}'
            | '\u{08e2}'
            | '\u{180e}'
            | '\u{200b}'..='\u{200f}'
            | '\u{2028}'..='\u{202e}'
            | '\u{2060}'..='\u{2064}'
            | '\u{2066}'..='\u{206f}'
            | '\u{feff}'
            | '\u{fff9}'..='\u{fffb}'
            | '\u{110bd}'
            | '\u{110cd}'
            | '\u{13430}'..='\u{1343f}'
            | '\u{1bca0}'..='\u{1bca3}'
            | '\u{1d173}'..='\u{1d17a}'
            | '\u{e0001}'
            | '\u{e0020}'..='\u{e007f}'
    )
}

/// Marker service names and topic attributes are written by OTHER processes
/// and the CLI prints these refusals verbatim, inside single quotes, so a
/// hostile name must not reach the operator's terminal raw. Three hazards are
/// neutralised at construction: every control character (C0, DEL, C1) and the
/// Unicode format set ([`is_format_char`]) render as a visible `\u{..}` escape
/// so no escape sequence, bidi override or zero-width character is emitted;
/// a single quote renders as `\'` so the name cannot close the quoted span and
/// forge the rest of the refusal (a fake remedy); and a backslash renders as
/// `\\` so the rendering is injective: a name that spells out the six
/// characters `\u{1b}` stays distinguishable from one that carries ESC.
/// Printable text, non-ASCII included, passes through unchanged so the name
/// stays recognisable.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
pub(crate) fn terminal_safe(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        if c.is_control() || matches!(c, '\\' | '\'') {
            out.extend(c.escape_debug());
        } else if is_format_char(c) {
            out.extend(c.escape_unicode());
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
        .map_err(|e| {
            err(
                topic,
                open_marker_reason(e, node.config().global.creation_timeout),
            )
        })
}

/// The reason a failed open-or-create of a marker refuses with. iceoryx2's
/// open-or-create waits out a marker marked for destruction itself, until the
/// node's creation window (`window`) ends, and then returns that state: it is
/// named, and every other error is quoted as iceoryx2 returned it.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn open_marker_reason(e: EventOpenOrCreateError, window: std::time::Duration) -> String {
    match e {
        EventOpenOrCreateError::EventOpenError(EventOpenError::IsMarkedForDestruction) => {
            super::marker_teardown::outlasted_creation_window(MIRROR_MARKER, window)
        }
        e => e.to_string(),
    }
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
/// names the marker service plus the remedy. A marker still being torn down is
/// opened again as `wait` allows and refuses, naming that wait, only once it
/// outlasts it.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
pub(crate) fn is_network_mirror(
    node: &Node<CerService>,
    topic: &str,
    wait: TeardownWait,
) -> TransportResult<bool> {
    is_network_mirror_with(node, topic, wait.budget(), &mut |builder, verifier| {
        builder.open_with_attributes(verifier)
    })
}

/// [`is_network_mirror`] with its teardown budget and its open of an existing
/// marker supplied.
// hot-path-alloc-ok-fn: cold admission and refusal text, never frame delivery.
fn is_network_mirror_with(
    node: &Node<CerService>,
    topic: &str,
    teardown: MarkerTeardown,
    open: &mut dyn FnMut(
        MarkerBuilder,
        &AttributeVerifier,
    ) -> Result<MarkerService, EventOpenError>,
) -> TransportResult<bool> {
    let name = service_name(topic, &identity_error)?;
    let verifier = verifier(topic, &identity_error)?;
    let opened = teardown.open(
        || {
            open(
                node.service_builder(&name)
                    .event()
                    // On an open the ceiling is a requirement the existing
                    // service must meet; see `CERULION_MAX_EVENT_ID`.
                    .event_id_max_value(super::CERULION_MAX_EVENT_ID),
                &verifier,
            )
        },
        |e| matches!(e, EventOpenError::IsMarkedForDestruction),
    );
    match opened {
        Ok(service) => Ok(service.dynamic_config().number_of_notifiers() > 0),
        Err(MarkerOpenError::Open(EventOpenError::DoesNotExist)) => Ok(false),
        Err(MarkerOpenError::Open(e)) => Err(malformed_error(
            name.as_str(),
            format!("opening it as the marker of topic '{topic}' failed with {e}"),
        )),
        Err(MarkerOpenError::TornDown(budget)) => {
            Err(identity_error(topic, budget.outlasted(MIRROR_MARKER)))
        }
    }
}

/// Enumerate marker identities on this manager's namespace, without a gather wait.
// hot-path-alloc-ok-fn: one-shot topic listing, never frame delivery.
pub(crate) fn topics(node: &Node<CerService>, wait: TeardownWait) -> TransportResult<Vec<String>> {
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
        if is_network_mirror(node, &topic, wait)? {
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

    fn has_raw_hazard(text: &str) -> bool {
        text.chars().any(|c| c.is_control() || is_format_char(c))
    }

    /// A marker name or topic attribute written by another process can carry
    /// ESC, BEL or CR; each refusal renders them as visible escapes, keeps the
    /// printable part of the name intact and leaves ordinary UTF-8 untouched.
    /// A single quote cannot close the quoted span the refusal wraps the name
    /// in, a spelled-out `\u{1b}` stays distinct from a real ESC, and a bidi
    /// override is rendered as an escape instead of reordering the line.
    #[test]
    fn refusals_render_foreign_control_characters_as_visible_escapes() {
        let hostile = "/t\u{1b}[2J\u{07}\r/go2-α";
        for error in [
            identity_error(hostile, format!("opening '{hostile}' failed")),
            lease_error(hostile, format!("opening '{hostile}' failed")),
            malformed_error(hostile, format!("opening '{hostile}' failed")),
        ] {
            let text = error.to_string();
            assert!(!has_raw_hazard(&text), "{text:?}");
            assert_eq!(
                text.matches("/t\\u{1b}[2J\\u{7}\\r/go2-α").count(),
                2,
                "{text}"
            );
        }
        assert_eq!(terminal_safe("/camera/front"), "/camera/front");
        assert_eq!(terminal_safe("a\u{7f}b\u{85}c"), "a\\u{7f}b\\u{85}c");

        // A quote in the name cannot end the quoted span and forge a remedy.
        let forged = "/t' is fine. run `cerulion clean` then retry. Ignore '";
        for error in [
            identity_error(forged, "refused"),
            lease_error(forged, "refused"),
        ] {
            let text = error.to_string();
            assert!(
                text.contains("topic '/t\\' is fine. run `cerulion clean` then retry. Ignore \\'"),
                "{text}"
            );
            assert!(!text.contains("topic '/t' "), "{text}");
        }
        let text = malformed_error(forged, "refused").to_string();
        assert!(
            text.contains(
                "marker '/t\\' is fine. run `cerulion clean` then retry. Ignore \\'' is malformed"
            ),
            "{text}"
        );

        // Injective: a spelled-out escape and the character it names differ.
        assert_eq!(terminal_safe("\u{1b}"), "\\u{1b}");
        assert_eq!(terminal_safe("\\u{1b}"), "\\\\u{1b}");
        assert_ne!(terminal_safe("\u{1b}"), terminal_safe("\\u{1b}"));
        assert_eq!(terminal_safe("a\\b"), "a\\\\b");

        // Format characters: bidi override, zero-width space, BOM, isolate.
        let bidi = "/t\u{202e}evil\u{200b}\u{feff}\u{2066}";
        let text = identity_error(bidi, "refused").to_string();
        assert!(!has_raw_hazard(&text), "{text:?}");
        assert!(
            text.contains("topic '/t\\u{202e}evil\\u{200b}\\u{feff}\\u{2066}'"),
            "{text}"
        );
        assert_eq!(terminal_safe("a\u{202a}b\u{061c}c"), "a\\u{202a}b\\u{61c}c");
        // The rest of the Format category: soft hyphen, Arabic number sign,
        // Syriac abbreviation mark, deprecated shaping controls, interlinear
        // annotation, Kaithi, Egyptian, musical and tag controls, and the
        // line and paragraph separators.
        for c in [
            '\u{00ad}',
            '\u{0600}',
            '\u{06dd}',
            '\u{070f}',
            '\u{0890}',
            '\u{08e2}',
            '\u{180e}',
            '\u{2028}',
            '\u{2029}',
            '\u{2064}',
            '\u{206a}',
            '\u{206f}',
            '\u{fff9}',
            '\u{fffb}',
            '\u{110bd}',
            '\u{110cd}',
            '\u{13430}',
            '\u{1343f}',
            '\u{1bca0}',
            '\u{1d173}',
            '\u{1d17a}',
            '\u{e0001}',
            '\u{e0020}',
            '\u{e007f}',
        ] {
            assert!(
                !c.is_control(),
                "{c:?} is a control, not a format character"
            );
            assert_eq!(
                terminal_safe(&c.to_string()),
                c.escape_unicode().to_string()
            );
        }
        // Printable non-ASCII letters and symbols still pass through.
        assert_eq!(terminal_safe("go2-α/日本/€"), "go2-α/日本/€");
    }

    fn node() -> Node<CerService> {
        NodeBuilder::new()
            .config(&crate::testing::iceoryx_test_config())
            .create::<CerService>()
            .unwrap()
    }

    /// A budget of `opens` with no pause between them.
    fn opens(opens: u32) -> MarkerTeardown {
        MarkerTeardown::new(
            std::num::NonZeroU32::new(opens).unwrap(),
            std::time::Duration::ZERO,
        )
    }

    /// [`is_network_mirror_with`] whose first `torn` opens report the marker
    /// as marked for destruction and whose later opens are the real open.
    /// Returns the answer and the number of opens.
    fn mirror_after(
        node: &Node<CerService>,
        topic: &str,
        torn: usize,
        teardown: MarkerTeardown,
    ) -> (TransportResult<bool>, usize) {
        let mut calls = 0usize;
        let answer = is_network_mirror_with(node, topic, teardown, &mut |builder, verifier| {
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
    fn a_mirror_marker_being_torn_down_is_waited_out_and_then_read() {
        let node = node();
        // No marker once the teardown ends: a local source.
        let (answer, calls) = mirror_after(&node, "/teardown/local", 3, opens(20));
        assert_eq!((answer.unwrap(), calls), (false, 4));
        // A live network mirror is read on the budget's last open.
        let _mirror = MirrorOrigin::open(&node, "/teardown/mirror").unwrap();
        let (answer, calls) = mirror_after(&node, "/teardown/mirror", 19, opens(20));
        assert_eq!((answer.unwrap(), calls), (true, 20));
        // A local observer's lease holds no mirror notifier: a local source.
        let _lease = LocalObservationLease::open(&node, "/teardown/observed").unwrap();
        let (answer, calls) = mirror_after(&node, "/teardown/observed", 1, opens(20));
        assert_eq!((answer.unwrap(), calls), (false, 2));
    }

    #[test]
    fn a_mirror_marker_that_outlasts_the_budget_is_not_called_malformed() {
        let node = node();
        let (answer, calls) =
            mirror_after(&node, "/teardown/stuck", usize::MAX, MarkerTeardown::BUDGET);
        assert_eq!(calls, 20);
        assert_eq!(
            answer.unwrap_err().to_string(),
            "Network mirror identity for topic '/teardown/stuck' could not be established: its \
             network mirror marker was still being torn down after 20 opens 25 ms apart; retry \
             the request"
        );
        let (answer, calls) = mirror_after(&node, "/teardown/stuck", 20, opens(20));
        assert_eq!(calls, 20);
        assert!(answer.is_err());
    }

    #[test]
    fn a_mirror_check_that_never_waits_opens_once_and_refuses_at_once() {
        let node = node();
        let (answer, calls) = mirror_after(
            &node,
            "/teardown/callback",
            usize::MAX,
            TeardownWait::Never.budget(),
        );
        assert_eq!(calls, 1);
        assert_eq!(
            answer.unwrap_err().to_string(),
            "Network mirror identity for topic '/teardown/callback' could not be established: \
             its network mirror marker was being torn down, and this check does not wait for \
             it; retry the request"
        );
    }

    #[test]
    fn any_other_mirror_open_error_still_names_the_malformed_marker() {
        let node = node();
        let topic = "/teardown/foreign";
        let mut calls = 0usize;
        let answer = is_network_mirror_with(&node, topic, opens(20), &mut |_, _| {
            calls += 1;
            Err(EventOpenError::IncompatibleAttributes)
        });
        assert_eq!(calls, 1);
        // `terminal_safe` escapes the reason's quotes, so they cannot close the
        // span the refusal quotes the marker name in.
        assert_eq!(
            answer.unwrap_err().to_string(),
            format!(
                "Reserved network mirror marker '{}' is malformed: opening it as the marker of \
                 topic \\'{topic}\\' failed with EventOpenError::IncompatibleAttributes. A process \
                 on this machine created it with an incompatible marker scheme: stop or upgrade \
                 that process, or run `cerulion clean` once it has exited, then retry",
                marker_service_name(topic)
            )
        );
    }

    #[test]
    fn an_open_or_create_that_ends_in_a_teardown_names_the_creation_window() {
        let window = std::time::Duration::from_millis(500);
        assert_eq!(
            open_marker_reason(
                EventOpenOrCreateError::EventOpenError(EventOpenError::IsMarkedForDestruction),
                window,
            ),
            "its network mirror marker was still being torn down when the 500 ms shared-memory \
             creation window ended; retry the request"
        );
        assert_eq!(
            open_marker_reason(EventOpenOrCreateError::SystemInFlux, window),
            "EventOpenOrCreateError::SystemInFlux"
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The iceoryx2 service attribute that states which [`WireHeader`] layout a
//! topic's data service carries.
//!
//! Every Cerulion data service (`{topic}/data`) this build creates carries the
//! attribute [`WIRE_HEADER_ATTRIBUTE_KEY`] set to its [`WIRE_HEADER_VERSION`].
//! Every Cerulion opener reads the attribute of the service it opened
//! ([`check_opened`]):
//!
//! - the key states this build's version: the service is used;
//! - the key states ANOTHER version: the open is refused, loudly (once per
//!   topic in the log, every time in the error), because the two sides would
//!   read each other's frames at the wrong offsets;
//! - the key is absent: the service was created by a raw iceoryx2 program, or
//!   by a Cerulion release older than the attribute. It is accepted, and a
//!   warning (once per topic) says its frames are assumed to carry this
//!   build's header layout.
//!
//! What this cannot do: a process built BEFORE the attribute existed reads no
//! attribute, so an older process opening a service this build created is
//! not refused by anything; and an older release's service (no attribute)
//! is accepted here under the assumption above. One Cerulion release per
//! machine is the supported configuration; the attribute turns the
//! newer-meets-newer mismatch loud.
//!
//! A raw iceoryx2 program that publishes Cerulion frames creates its data
//! service with [`data_service_specifier`] to state its version.
//!
//! [`WireHeader`]: crate::wire::WireHeader

use std::collections::BTreeSet;
use std::sync::Mutex;

use iceoryx2::service::attribute::{AttributeSet, AttributeSpecifier, AttributeVerifier};

use crate::error::{TransportError, TransportResult};
use crate::wire::WIRE_HEADER_VERSION;

/// The attribute key on every Cerulion data service. Its value is the decimal
/// [`WIRE_HEADER_VERSION`] of the frames the service carries.
pub const WIRE_HEADER_ATTRIBUTE_KEY: &str = "cerulion.wire_header_version";

// hot-path-alloc-ok-fn: cold, one attribute per service open or create, never per frame.
fn key_and_value(
    version: u16,
) -> TransportResult<(
    iceoryx2::service::attribute::AttributeKey,
    iceoryx2::service::attribute::AttributeValue,
)> {
    let key = WIRE_HEADER_ATTRIBUTE_KEY
        .try_into()
        .map_err(|e| TransportError::Internal {
            reason: format!("wire header attribute key '{WIRE_HEADER_ATTRIBUTE_KEY}': {e:?}"),
        })?;
    let text = version.to_string();
    let value = text
        .as_str()
        .try_into()
        .map_err(|e| TransportError::Internal {
            reason: format!("wire header attribute value '{text}': {e:?}"),
        })?;
    Ok((key, value))
}

/// The attribute set a data service is CREATED with, stating `version`.
// hot-path-alloc-ok-fn: cold, one attribute per service create, never per frame.
pub fn specifier_for(version: u16) -> TransportResult<AttributeSpecifier> {
    let (key, value) = key_and_value(version)?;
    AttributeSpecifier::new()
        .define(&key, &value)
        .map_err(|e| TransportError::Internal {
            reason: format!("wire header attribute: {e:?}"),
        })
}

/// A requirement that the attribute states `version`, for a raw iceoryx2
/// opener that wants iceoryx2 itself to refuse any other service, including
/// one with no attribute (`open_with_attributes`; passed to
/// `open_or_create_with_attributes` it also defines the attribute when the
/// call creates the service). Cerulion's own openers do not use it: they open
/// plainly and apply [`check_opened`], which accepts an absent attribute.
// hot-path-alloc-ok-fn: cold, one attribute per service open, never per frame.
pub fn verifier_for(version: u16) -> TransportResult<AttributeVerifier> {
    let (key, value) = key_and_value(version)?;
    AttributeVerifier::new()
        .require(&key, &value)
        .map_err(|e| TransportError::Internal {
            reason: format!("wire header attribute: {e:?}"),
        })
}

/// [`specifier_for`] at this build's [`WIRE_HEADER_VERSION`].
pub fn data_service_specifier() -> TransportResult<AttributeSpecifier> {
    specifier_for(WIRE_HEADER_VERSION)
}

/// [`verifier_for`] at this build's [`WIRE_HEADER_VERSION`], for a raw
/// iceoryx2 opener (see [`verifier_for`]).
pub fn data_service_verifier() -> TransportResult<AttributeVerifier> {
    verifier_for(WIRE_HEADER_VERSION)
}

/// What an opened data service states about its wire header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StatedWireHeader {
    /// The attribute states this build's version.
    Matches,
    /// The service carries no such attribute.
    Absent,
    /// The attribute states something else (another version, or a value that
    /// is not one), verbatim.
    Other(String),
}

/// Read the wire header attribute off an opened service's attribute set.
// hot-path-alloc-ok-fn: cold, one read per service open, never per frame.
pub fn stated(attributes: &AttributeSet) -> StatedWireHeader {
    let Ok(key) = WIRE_HEADER_ATTRIBUTE_KEY.try_into() else {
        return StatedWireHeader::Absent;
    };
    let ours = WIRE_HEADER_VERSION.to_string();
    let n = attributes.number_of_key_values(&key);
    if n == 0 {
        return StatedWireHeader::Absent;
    }
    let mut other = None;
    for i in 0..n {
        if let Some(v) = attributes.key_value(&key, i) {
            let v = v.to_string();
            if v == ours {
                return StatedWireHeader::Matches;
            }
            other.get_or_insert(v);
        }
    }
    StatedWireHeader::Other(other.unwrap_or_default())
}

/// The most topics each once-per-topic log set remembers, so a machine with
/// unbounded topic churn cannot grow it. Past it, a topic not yet in the set
/// logs its absent attribute or its mismatch at `debug!` instead (a refusal
/// still returns its error every time).
const LOGGED_TOPICS_CAP: usize = 1024;

static WARNED_ABSENT: Mutex<BTreeSet<String>> = Mutex::new(BTreeSet::new()); // hot-path-alloc-ok: const empty set, filled only at service open
static LOGGED_MISMATCH: Mutex<BTreeSet<String>> = Mutex::new(BTreeSet::new()); // hot-path-alloc-ok: const empty set, filled only at service open

// hot-path-alloc-ok-fn: cold, one insert per topic, bounded.
fn first_time(set: &Mutex<BTreeSet<String>>, topic: &str) -> bool {
    let mut set = set
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    set.len() < LOGGED_TOPICS_CAP && set.insert(topic.to_string())
}

/// The rule every Cerulion opener applies to a data service it opened (see
/// the module docs): `Ok` to use it, `Err(reason)` to refuse. An absent
/// attribute is accepted with a once-per-topic warning; another stated
/// version is refused, logged once per topic, the reason naming both
/// versions and the remedy.
// hot-path-alloc-ok-fn: cold, one check per service open, never per frame.
pub fn check_opened(topic: &str, attributes: &AttributeSet) -> Result<(), String> {
    match stated(attributes) {
        StatedWireHeader::Matches => Ok(()),
        StatedWireHeader::Absent => {
            if first_time(&WARNED_ABSENT, topic) {
                tracing::warn!(
                    topic = %topic,
                    local_wire_header = WIRE_HEADER_VERSION,
                    "this topic's data service states no wire header version (it was created \
                     by a raw iceoryx2 program or an older Cerulion release); its frames are \
                     assumed to carry this build's header layout. If a process from an older \
                     Cerulion release created it, its frames will be read at the wrong offsets: \
                     stop that release's processes and run one Cerulion release on this machine"
                );
            } else {
                tracing::debug!(topic = %topic, "data service states no wire header version (repeat)");
            }
            Ok(())
        }
        StatedWireHeader::Other(theirs) => {
            let reason = format!(
                "the data service states wire header version {theirs} and this build reads \
                 version {WIRE_HEADER_VERSION}, so its frames would be read at the wrong offsets; \
                 it was created by a process from another Cerulion release. Stop the processes \
                 from that release (and rebuild node libraries with `cerulion node build`), then \
                 retry"
            );
            if first_time(&LOGGED_MISMATCH, topic) {
                tracing::error!(
                    topic = %topic,
                    stated_wire_header = %theirs,
                    local_wire_header = WIRE_HEADER_VERSION,
                    "refused a data service created by another Cerulion release: it states \
                     another wire header version, so its frames would be read at the wrong \
                     offsets"
                );
            } else {
                tracing::debug!(
                    topic = %topic,
                    stated_wire_header = %theirs,
                    "refused a data service stating another wire header version (repeat)"
                );
            }
            Err(reason)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The attribute states the version in decimal under one fixed key, on
    /// both the create and the open side.
    #[test]
    fn the_attribute_states_the_version_in_decimal_under_one_key() {
        let spec = specifier_for(2).expect("specifier");
        let attrs = spec.attributes();
        assert_eq!(attrs.number_of_attributes(), 1);
        let key = WIRE_HEADER_ATTRIBUTE_KEY.try_into().expect("key");
        assert_eq!(attrs.number_of_key_values(&key), 1);
        assert_eq!(
            attrs.key_value(&key, 0).map(|v| v.to_string()),
            Some("2".to_string())
        );

        let verifier = verifier_for(513).expect("verifier");
        let required = verifier.required_attributes();
        assert_eq!(
            required.key_value(&key, 0).map(|v| v.to_string()),
            Some("513".to_string())
        );
        assert!(verifier.required_keys().is_empty());

        // A service stating version 1 does not satisfy a version 2 verifier,
        // and one stating nothing satisfies no verifier. (Cerulion's own
        // openers do not use a verifier; `check_opened` accepts an absent
        // attribute.)
        assert!(verifier_for(2)
            .unwrap()
            .verify_requirements(specifier_for(1).unwrap().attributes())
            .is_err());
        assert!(verifier_for(2)
            .unwrap()
            .verify_requirements(AttributeSpecifier::new().attributes())
            .is_err());
        assert!(verifier_for(2)
            .unwrap()
            .verify_requirements(specifier_for(2).unwrap().attributes())
            .is_ok());
    }

    /// `stated` reads the three cases off an attribute set.
    #[test]
    fn an_attribute_set_states_matching_absent_or_another_version() {
        assert_eq!(
            stated(specifier_for(WIRE_HEADER_VERSION).unwrap().attributes()),
            StatedWireHeader::Matches
        );
        assert_eq!(
            stated(AttributeSpecifier::new().attributes()),
            StatedWireHeader::Absent
        );
        assert_eq!(
            stated(specifier_for(WIRE_HEADER_VERSION + 1).unwrap().attributes()),
            StatedWireHeader::Other((WIRE_HEADER_VERSION + 1).to_string())
        );
        assert!(check_opened("/t/absent", AttributeSpecifier::new().attributes()).is_ok());
        let err = check_opened(
            "/t/other",
            specifier_for(WIRE_HEADER_VERSION + 1).unwrap().attributes(),
        )
        .unwrap_err();
        assert!(err.contains(&format!("wire header version {}", WIRE_HEADER_VERSION + 1)));
    }
}

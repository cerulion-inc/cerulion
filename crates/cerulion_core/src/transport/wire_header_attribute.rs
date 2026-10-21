// SPDX-License-Identifier: AGPL-3.0-only
//! The iceoryx2 service attribute that states which [`WireHeader`] layout a
//! topic's data service carries.
//!
//! Every Cerulion data service (`{topic}/data`) is created with the attribute
//! [`WIRE_HEADER_ATTRIBUTE_KEY`] set to this build's [`WIRE_HEADER_VERSION`],
//! and every Cerulion opener REQUIRES it. A process from a release with a
//! different header version (or a foreign iceoryx2 process that created the
//! service without the attribute) therefore cannot be attached to: the open
//! fails with `IncompatibleAttributes` and the error names the remedy, instead
//! of the two sides reading each other's frames at the wrong offsets.
//!
//! What this cannot do: a process built BEFORE the attribute existed requires
//! nothing when it opens, so an older process opening a service this build
//! created is not refused by iceoryx2. Run one Cerulion release per machine.
//!
//! A raw iceoryx2 program that publishes Cerulion frames creates its data
//! service with [`data_service_specifier`] (or opens with
//! [`data_service_verifier`]) to interoperate.
//!
//! [`WireHeader`]: crate::wire::WireHeader

use iceoryx2::service::attribute::{AttributeSpecifier, AttributeVerifier};

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

/// The requirement a data service is OPENED with: the attribute must state
/// `version`. Passed to `open_or_create_with_attributes`, it also defines the
/// attribute when the call creates the service.
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

/// [`verifier_for`] at this build's [`WIRE_HEADER_VERSION`].
pub fn data_service_verifier() -> TransportResult<AttributeVerifier> {
    verifier_for(WIRE_HEADER_VERSION)
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

        // A service stating version 1 does not satisfy a version 2 opener, and
        // one stating nothing satisfies no opener.
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
}

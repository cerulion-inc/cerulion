// SPDX-License-Identifier: AGPL-3.0-only
//! Python exception types and the `TransportError` → `PyErr` mapping.
//!
//! One exception hierarchy (`cerulion.CerulionError` root) shared by the
//! native module and the facade. `TransportError` is `#[non_exhaustive]`,
//! so the match lists every CURRENT variant explicitly and keeps a `_`
//! arm for variants added later - new variants degrade to `TransportError`
//! (the catch-all bucket) rather than failing to compile silently elsewhere.

use cerulion_core::TransportError as CerTransportError;
use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::PyErr;

create_exception!(
    cerulion,
    CerulionError,
    PyException,
    "Base error for all cerulion client failures."
);
create_exception!(
    cerulion,
    SchemaMismatch,
    CerulionError,
    "A frame's schema_hash differs from what the receiver expected."
);
create_exception!(
    cerulion,
    BorrowLimitExceeded,
    CerulionError,
    "The subscriber holds more frames than the service's borrow budget allows."
);
create_exception!(
    cerulion,
    ReleasedFrame,
    CerulionError,
    "A frame was accessed after release()."
);
create_exception!(
    cerulion,
    TransportError,
    CerulionError,
    "A transport-layer operation failed."
);
create_exception!(
    cerulion,
    EncodeError,
    CerulionError,
    "A publish/loan request cannot be encoded into the publisher's slot."
);

/// Map a core [`CerTransportError`] onto the Python exception hierarchy.
///
/// `Receive` whose reason carries iceoryx2's `ExceedsMaxBorrows` is the
/// borrow-budget overflow a Python user CAN fix (release held frames, or
/// create the subscriber with a bigger depth), so it gets its own type
/// and the remedy rides inside the message.
pub(crate) fn map_transport_err(e: CerTransportError) -> PyErr {
    if let CerTransportError::Receive { reason, .. } = &e {
        if reason.contains("ExceedsMaxBorrows") {
            return BorrowLimitExceeded::new_err(format!(
                "{e}: release frames (and any live memoryviews over them) to return borrowed slots"
            ));
        }
    }
    let msg = e.to_string();
    match e {
        CerTransportError::SchemaMismatch { .. } | CerTransportError::SchemaHashMismatch { .. } => {
            SchemaMismatch::new_err(msg)
        }
        CerTransportError::LoanCapacity { .. } => TransportError::new_err(msg),
        CerTransportError::NodeCreation { .. } => TransportError::new_err(msg),
        CerTransportError::PublisherCreation { .. } => TransportError::new_err(msg),
        CerTransportError::SubscriberCreation { .. } => TransportError::new_err(msg),
        CerTransportError::Loan { .. } => TransportError::new_err(msg),
        CerTransportError::Publish { .. } => TransportError::new_err(msg),
        CerTransportError::Receive { .. } => TransportError::new_err(msg),
        CerTransportError::TopicNotFound { .. } => TransportError::new_err(msg),
        CerTransportError::NotInitialized => TransportError::new_err(msg),
        CerTransportError::DuplicateNode { .. } => TransportError::new_err(msg),
        CerTransportError::NodeNotFound { .. } => TransportError::new_err(msg),
        CerTransportError::SchedulerError { .. } => TransportError::new_err(msg),
        CerTransportError::GraphError { .. } => TransportError::new_err(msg),
        CerTransportError::ExternalNodesInertAtLaunch { .. } => TransportError::new_err(msg),
        CerTransportError::LiveReactorBuild { .. } => TransportError::new_err(msg),
        CerTransportError::InvalidTransportConfig { .. } => TransportError::new_err(msg),
        CerTransportError::UnrepresentableNodeName { .. } => TransportError::new_err(msg),
        CerTransportError::GraphParseError { .. } => TransportError::new_err(msg),
        CerTransportError::NodeError { .. } => TransportError::new_err(msg),
        CerTransportError::NodeTickPanicked { .. } => TransportError::new_err(msg),
        CerTransportError::NodeInfoParse { .. } => TransportError::new_err(msg),
        CerTransportError::SessionCreation { .. } => TransportError::new_err(msg),
        CerTransportError::Deserialization { .. } => TransportError::new_err(msg),
        CerTransportError::BufferTooSmall { .. } => TransportError::new_err(msg),
        CerTransportError::ProxyBufferTooSmall { .. } => TransportError::new_err(msg),
        CerTransportError::MissingVariableField { .. } => TransportError::new_err(msg),
        CerTransportError::MaxSliceLenRequired { .. } => TransportError::new_err(msg),
        CerTransportError::PushAfterNonTail { .. } => TransportError::new_err(msg),
        CerTransportError::NestedWriteConflict { .. } => TransportError::new_err(msg),
        CerTransportError::NestedChildIncomplete { .. } => TransportError::new_err(msg),
        CerTransportError::PayloadTooLarge { .. } => TransportError::new_err(msg),
        CerTransportError::AllocationFailed { .. } => TransportError::new_err(msg),
        CerTransportError::Internal { .. } => TransportError::new_err(msg),
        // `#[non_exhaustive]`: any variant added upstream degrades to the
        // generic TransportError bucket.
        _ => TransportError::new_err(msg),
    }
}

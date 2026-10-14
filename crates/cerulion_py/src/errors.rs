// SPDX-License-Identifier: AGPL-3.0-only
//! Python exception types and the `TransportError` → `PyErr` mapping.
//!
//! One exception hierarchy (`cerulion.CerulionError` root) shared by the
//! native module and the facade. `TransportError` is `#[non_exhaustive]`,
//! so the match lists every CURRENT variant explicitly and keeps a `_`
//! arm for variants added later - new variants degrade to `TransportError`
//! (the catch-all bucket) rather than failing to compile silently elsewhere.

use cerulion_core::dynamic::{DynamicError, WalkError};
use cerulion_core::TransportError as CerTransportError;
use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::types::PyAnyMethods;
use pyo3::{PyErr, Python};

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
create_exception!(
    cerulion,
    DecodeError,
    CerulionError,
    "A wire frame could not be decoded according to its schema."
);
create_exception!(
    cerulion,
    SchemaError,
    CerulionError,
    "A schema could not be loaded or resolved."
);
create_exception!(
    cerulion,
    BagError,
    CerulionError,
    "A recording bag could not be opened or read."
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
            Python::attach(|py| schema_mismatch(py, msg))
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

/// Map every [`cerulion_bag::BagError`] onto Python `BagError`.
pub(crate) fn map_bag_err(e: cerulion_bag::BagError) -> PyErr {
    BagError::new_err(e.to_string())
}

fn dynamic_exception(py: Python<'_>, decode: bool, kind: &str, message: String) -> PyErr {
    let ty = if decode {
        py.get_type::<DecodeError>()
    } else {
        py.get_type::<SchemaError>()
    };
    let err = ty
        .call1((message,))
        .expect("constructing a native exception cannot fail");
    err.setattr("kind", kind)
        .expect("setting a native exception kind cannot fail");
    PyErr::from_value(err)
}

fn walk_kind(error: &WalkError) -> &'static str {
    match error {
        WalkError::UnknownSchema(_) => "UnknownSchema",
        WalkError::UnknownSchemaHash(_) => "UnknownSchemaHash",
        WalkError::FrameTooShort { .. } => "FrameTooShort",
        WalkError::FixedFieldOutOfBounds { .. } => "FixedFieldOutOfBounds",
        WalkError::OffsetTableTruncated { .. } => "OffsetTableTruncated",
        WalkError::VariableFieldOutOfBounds { .. } => "VariableFieldOutOfBounds",
        WalkError::NestedResolutionFailed { .. } => "NestedResolutionFailed",
        WalkError::ElementBodyNotExactlyConsumed { .. } => "ElementBodyNotExactlyConsumed",
    }
}

pub(crate) fn map_dynamic_err(py: Python<'_>, error: DynamicError) -> PyErr {
    let message = error.to_string();
    match &error {
        DynamicError::Yaml(_)
        | DynamicError::MissingSchemasKey
        | DynamicError::InvalidSchemaName(_)
        | DynamicError::SchemaNotMapping { .. }
        | DynamicError::FieldsNotMapping { .. }
        | DynamicError::InvalidFieldKey { .. }
        | DynamicError::FixedLengthTooLarge { .. }
        | DynamicError::Rosmsg { .. }
        | DynamicError::Io { .. }
        | DynamicError::SchemaNotWireRepresentable { .. }
        | DynamicError::UnknownSchema(_)
        | DynamicError::InvalidLayout { .. } => {
            dynamic_exception(py, false, dynamic_kind(&error), message)
        }
        DynamicError::UnknownSchemaHash(_) | DynamicError::SchemaHashMismatch { .. } => {
            schema_mismatch(py, message)
        }
        DynamicError::VariableCountMismatch { .. }
        | DynamicError::LengthNotElementMultiple { .. }
        | DynamicError::FrameTooLarge { .. }
        | DynamicError::BufferTooSmall { .. } => EncodeError::new_err(message),
        DynamicError::FrameTooShort { .. }
        | DynamicError::TotalSizeExceedsBuffer { .. }
        | DynamicError::TotalSizeBelowPrefix { .. }
        | DynamicError::OffsetTableMismatch { .. }
        | DynamicError::OffsetBelowDataFloor { .. }
        | DynamicError::VariableFieldOutOfBounds { .. }
        | DynamicError::OverlappingEntries { .. }
        | DynamicError::MisalignedElements { .. }
        | DynamicError::MisalignedBuffer { .. }
        | DynamicError::InvalidUtf8 { .. }
        | DynamicError::UnknownFixedField(_)
        | DynamicError::UnknownVariableField(_)
        | DynamicError::NotAStringField(_)
        | DynamicError::NotAPrimitiveArrayField(_) => {
            dynamic_exception(py, true, dynamic_kind(&error), message)
        }
        DynamicError::Walk(walk) => dynamic_exception(py, true, walk_kind(walk), message),
        _ => dynamic_exception(py, true, "DynamicError", message),
    }
}

pub(crate) fn schema_mismatch(py: Python<'_>, message: String) -> PyErr {
    let err = py
        .get_type::<SchemaMismatch>()
        .call1((message,))
        .expect("constructing a native schema mismatch cannot fail");
    err.setattr("kind", "SchemaMismatch")
        .expect("setting a native schema mismatch kind cannot fail");
    PyErr::from_value(err)
}

fn dynamic_kind(error: &DynamicError) -> &'static str {
    match error {
        DynamicError::Yaml(_) => "Yaml",
        DynamicError::MissingSchemasKey => "MissingSchemasKey",
        DynamicError::InvalidSchemaName(_) => "InvalidSchemaName",
        DynamicError::SchemaNotMapping { .. } => "SchemaNotMapping",
        DynamicError::FieldsNotMapping { .. } => "FieldsNotMapping",
        DynamicError::InvalidFieldKey { .. } => "InvalidFieldKey",
        DynamicError::FixedLengthTooLarge { .. } => "FixedLengthTooLarge",
        DynamicError::Rosmsg { .. } => "Rosmsg",
        DynamicError::Io { .. } => "Io",
        DynamicError::SchemaNotWireRepresentable { .. } => "SchemaNotWireRepresentable",
        DynamicError::UnknownSchema(_) => "UnknownSchema",
        DynamicError::UnknownSchemaHash(_) => "UnknownSchemaHash",
        DynamicError::InvalidLayout { .. } => "InvalidLayout",
        DynamicError::SchemaHashMismatch { .. } => "SchemaHashMismatch",
        DynamicError::VariableCountMismatch { .. } => "VariableCountMismatch",
        DynamicError::LengthNotElementMultiple { .. } => "LengthNotElementMultiple",
        DynamicError::FrameTooLarge { .. } => "FrameTooLarge",
        DynamicError::BufferTooSmall { .. } => "BufferTooSmall",
        DynamicError::FrameTooShort { .. } => "FrameTooShort",
        DynamicError::TotalSizeExceedsBuffer { .. } => "TotalSizeExceedsBuffer",
        DynamicError::TotalSizeBelowPrefix { .. } => "TotalSizeBelowPrefix",
        DynamicError::OffsetTableMismatch { .. } => "OffsetTableMismatch",
        DynamicError::OffsetBelowDataFloor { .. } => "OffsetBelowDataFloor",
        DynamicError::VariableFieldOutOfBounds { .. } => "VariableFieldOutOfBounds",
        DynamicError::OverlappingEntries { .. } => "OverlappingEntries",
        DynamicError::MisalignedElements { .. } => "MisalignedElements",
        DynamicError::MisalignedBuffer { .. } => "MisalignedBuffer",
        DynamicError::InvalidUtf8 { .. } => "InvalidUtf8",
        DynamicError::UnknownFixedField(_) => "UnknownFixedField",
        DynamicError::UnknownVariableField(_) => "UnknownVariableField",
        DynamicError::NotAStringField(_) => "NotAStringField",
        DynamicError::NotAPrimitiveArrayField(_) => "NotAPrimitiveArrayField",
        DynamicError::Walk(walk) => walk_kind(walk),
        _ => "DynamicError",
    }
}

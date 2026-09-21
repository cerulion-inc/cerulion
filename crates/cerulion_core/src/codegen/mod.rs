// SPDX-License-Identifier: AGPL-3.0-only
//! Code generation for message schemas.
//!
//! This module parses ROS 2 `.msg` text into a [`MessageSchema`] and
//! generates Rust source from it. Per message it emits three items: a unit
//! marker `<Name>` that implements `ShmMessage` (the type a node writes on a
//! port field), the accessor `<Name>Shm` that reads and writes the message
//! where it lives in shared memory, and an owned `<Name>Snapshot`.
//!
//! The `native_ros2_messages` build script is its main caller. Node code does
//! not call it.

pub mod big_array;
pub mod cdr_codec;
pub mod element_codec;
pub mod frame_walker;
mod generator;
pub mod layout;
mod resolve;
mod rosmsg_parser;
pub mod route_budget;
mod schema;
pub mod slice_ceiling;
pub mod unknown_hash;

pub use cdr_codec::{
    resolve_nested_qname_ladder, split_encapsulation, CdrCodec, CdrCodecError, CdrEndianness,
    MAX_CDR_FRAME_BYTES, MAX_CDR_TRAILING_PAD,
};
pub use element_codec::{
    variable_payload_align, CanonicalBodyBuilder, CanonicalBodyReader, ElementBodyError,
};
pub use frame_walker::{
    FrameValue, FrameValueKind, FrameWalker, NamedValue, PrimArray, PrimType, WalkError,
};
pub use generator::{generate_schema, variable_schema_max_slice_len, IN_REPO_PACKAGES};
pub use resolve::{composed_overflow_indices, resolve_fixed_nested};
pub use rosmsg_parser::parse_rosmsg;
pub use route_budget::{route_slice_budget, route_slice_budget_capped};
pub use schema::{DefaultLiteral, FieldDef, FieldType, MessageSchema, NestedFixedInfo};
pub use slice_ceiling::{
    resolve_rmw_slice_ceiling, slice_ceiling_for_type, SliceCeilingOverrides, RMW_SLICE_CEILING_ENV,
};
pub use unknown_hash::{
    diagnose_unknown_hash, diagnose_unknown_hash_with_walker, report_schema_hash_resolved,
    report_unknown_schema_hash, DiagnosisVantage, UnknownHashCandidate, UnknownHashDiagnosis,
};

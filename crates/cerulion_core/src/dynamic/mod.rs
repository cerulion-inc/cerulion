// SPDX-License-Identifier: AGPL-3.0-only
//! Dynamic (schema-driven) frame API - **the supported surface for language
//! bindings**.
//!
//! A binding (Python, C, …) has no generated Rust message types. This module
//! lets it work with Cerulion wire frames from schemas alone:
//!
//! 1. Load schemas into a [`SchemaSet`] (workspace YAML, ROS 2 `.msg` text,
//!    or pre-parsed [`MessageSchema`]s). Resolution warnings come back AND
//!    are logged.
//! 2. Read a [`WireLayout`] (or its JSON via [`WireLayout::to_json`]) to
//!    learn field names, fixed offsets/sizes, and variable-field order.
//! 3. Encode with [`FrameEncoder`] into any `&mut [u8]` - typically a
//!    transport loan - byte-identical to the generated writer path.
//! 4. Decode with [`FrameView`] (validated, zero-copy slices by field name)
//!    or the full typed [`FrameWalker`] tree via [`FrameView::decode`].
//!
//! Every failure is one variant of [`DynamicError`].
//!
//! **Stability**: the names, signatures and error variants exported here are
//! stable-intent: additions are fine, renames/removals are breaking and go
//! through a deprecation cycle. The JSON descriptor shape produced by
//! [`WireLayout::to_json`] is likewise pinned by an oracle test. The wire
//! format itself is governed by [`crate::wire`] and unchanged by this module.
//! Types re-exported from [`crate::codegen`] keep that module's semantics;
//! the re-export is the commitment that they remain reachable from here.
//!
//! Contract details and a worked byte example: `docs/internals/core-dynamic.md`.

mod encoder;
mod error;
mod schema_set;
mod schema_yaml;
mod view;

pub use encoder::{FrameCursor, FrameEncoder};
pub use error::DynamicError;
pub use schema_set::{parse_rosmsg, SchemaSet};
pub use schema_yaml::{
    parse_field_key, parse_yaml_schemas, validate_fixed_lengths, MAX_FIXED_ARRAY_LEN,
};
pub use view::FrameView;

pub use crate::codegen::element_codec::variable_payload_align;
pub use crate::codegen::layout::{FieldLayout, LayoutResolver, VariableFieldLayout, WireLayout};
/// These lower-level constructors take trusted IR. In particular,
/// [`FrameWalker::new`] and [`LayoutResolver`] may panic on hostile fixed
/// sizes; [`SchemaSet`] is the checked construction path.
pub use crate::codegen::{
    FieldDef, FieldType, FrameValue, FrameValueKind, FrameWalker, MessageSchema, NamedValue,
    NestedFixedInfo, PrimArray, PrimType, WalkError,
};
pub use crate::wire::{OffsetEntry, WireHeader};

#[cfg(test)]
mod tests;

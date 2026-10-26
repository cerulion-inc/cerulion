// SPDX-License-Identifier: AGPL-3.0-only
//! Messages as text on local shared memory, for the verbs that send or show
//! one message: `cerulion topic pub`, `service call`, `topic snapshot` on a
//! desk, and the same operations as `cerulion-remoted` ops verbs on a robot.
//!
//! - [`text`]: JSON or YAML text to a wire frame and a message back to JSON,
//!   and the exit-code classes every verb reports ([`text::VerbError`]).
//! - [`schema_store`]: the workspace `.msg` store and its `.srv` files.
//! - [`service`]: call a service the way `rmw_cerulion` serves one.
//! - [`publish`]: publish frames of one message on a topic.
//! - [`snapshot`]: a camera frame as JPEG, and the newest frame of a topic.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod publish;
pub mod schema_store;
pub mod service;
pub mod snapshot;
pub mod text;

use std::path::Path;

use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};

/// Every built-in ROS 2 message (`native_ros2_messages::BUILTIN_MSGS`)
/// parsed to its schema, in registry order.
///
/// The text parsed at build time to emit the types this crate links
/// against, so a parse failure is a broken build invariant and panics with
/// the type's name.
pub fn builtin_schemas() -> Vec<MessageSchema> {
    let entries = native_ros2_messages::BUILTIN_MSGS;
    let mut schemas = Vec::with_capacity(entries.len());
    for &(package, name, text) in entries {
        let schema = parse_rosmsg(text, name, Some(package))
            .unwrap_or_else(|e| panic!("built-in registry: failed to parse {package}/{name}: {e}"));
        schemas.push(schema);
    }
    schemas
}

/// The built-ins plus every `.msg` of the store under `schemas_dir` (its
/// `schemas/*.yaml` are not read here).
pub fn store_schemas(schemas_dir: Option<&Path>) -> Vec<MessageSchema> {
    let mut schemas = builtin_schemas();
    if let Some(dir) = schemas_dir {
        schemas.extend(schema_store::SchemaStore::load(dir).schemas());
    }
    schemas
}

/// A walker over `schemas`, built through the checked
/// [`cerulion_core::dynamic::SchemaSet`] construction.
pub fn checked_walker(schemas: Vec<MessageSchema>) -> Result<FrameWalker, text::VerbError> {
    cerulion_core::dynamic::SchemaSet::from_schemas(schemas)
        .map(|(set, _warnings)| set.walker().clone())
        .map_err(|e| text::VerbError::Usage(format!("the schemas do not lay out: {e}")))
}

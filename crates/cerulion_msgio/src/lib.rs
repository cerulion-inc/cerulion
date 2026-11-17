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

/// Schemas built the way `rmw_cerulion` builds a type's schema from its ROS 2
/// introspection members (`type_bridge::collect_schemas`): each message in
/// the package of its introspection namespace, each nested member named
/// with its package, and the hash read off a `LayoutResolver` over the type
/// and every type it nests (`BridgedMessage::new`).
#[cfg(test)]
pub(crate) mod introspection {
    use cerulion_core::codegen::layout::LayoutResolver;
    use cerulion_core::codegen::{FieldDef, FieldType, MessageSchema};

    /// The message `package/name` with `members` in declaration order.
    pub(crate) fn message(
        package: &str,
        name: &str,
        members: Vec<(&str, FieldType)>,
    ) -> MessageSchema {
        let mut schema = MessageSchema::new_in_package(name, package);
        for (field, field_type) in members {
            schema.add_field(FieldDef::new(field, field_type));
        }
        schema
    }

    /// A member whose type is the message `package/name`.
    pub(crate) fn nested(package: &str, name: &str) -> FieldType {
        FieldType::Nested {
            schema_name: name.to_string(),
            package: Some(package.to_string()),
            fixed: None,
        }
    }

    /// `std_msgs/Header` and the `builtin_interfaces/Time` it nests.
    pub(crate) fn header() -> Vec<MessageSchema> {
        vec![
            message(
                "builtin_interfaces",
                "Time",
                vec![("sec", FieldType::I32), ("nanosec", FieldType::U32)],
            ),
            message(
                "std_msgs",
                "Header",
                vec![
                    ("stamp", nested("builtin_interfaces", "Time")),
                    ("frame_id", FieldType::String),
                ],
            ),
        ]
    }

    /// The schema hash `rmw_cerulion` stamps for `root`, laid out over `set`
    /// (the root and every type it nests). The bridge refuses a set with
    /// resolution warnings, so this does too.
    pub(crate) fn hash(set: Vec<MessageSchema>, root: &str) -> u64 {
        let (mut resolver, warnings) = LayoutResolver::new(set);
        assert!(warnings.is_empty(), "{warnings:?}");
        resolver
            .layout_of(root)
            .unwrap_or_else(|| panic!("{root} does not lay out"))
            .schema_hash
    }
}

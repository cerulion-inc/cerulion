// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE workspace schema-YAML parser: `schemas/*.yaml` → [`MessageSchema`].
//!
//! The `cerulion` CLI (`schema info`/`list`/`serve`, `graph run`, `topic`)
//! and [`SchemaSet`](super::SchemaSet) both route through
//! [`parse_yaml_schemas`], so a binding and the CLI can never disagree on
//! which files parse or on a schema's hash.

use super::DynamicError;
use crate::codegen::{FieldDef, FieldType, MessageSchema};

/// Largest inline fixed length accepted for `T[n]` and `string_fixed[n]`
/// declarations (2^20). Keeps every parse-time fixed-section size far below
/// `usize` overflow, so the panicking size recipe never sees a hostile
/// length - one such field is 8 MiB of `float64`, already past any frame a
/// real graph carries.
pub const MAX_FIXED_ARRAY_LEN: usize = 1_048_576;

/// Parse a workspace schema-YAML document into package-less schemas, in
/// declaration order, with each schema's `description` populated.
///
/// Strict and loud:
/// - the document must have a top-level `schemas:` mapping;
/// - every entry key is a non-empty string, kept in its DECLARED spelling
///   (the wire-hash identity - a workspace node's build script hashes the
///   same key);
/// - a schema definition must be a mapping; absent or explicit-null `fields:`
///   means zero fields, while a present non-null `fields:` must be a mapping;
/// - a non-string `description:` is ignored for CLI compatibility, and field
///   VALUES are ignored because each field key carries the declaration;
/// - every `fields:` key is exactly `<type> <name>`, the type accepted by
///   [`FieldType::parse`], inline lengths at most [`MAX_FIXED_ARRAY_LEN`].
///
/// Nested references are left unresolved (that is the
/// [`LayoutResolver`](crate::codegen::layout::LayoutResolver)'s job over the
/// FULL set), so the returned schemas are hash-stable only once resolved.
pub fn parse_yaml_schemas(yaml: &str) -> Result<Vec<MessageSchema>, DynamicError> {
    let doc: serde_yaml::Value =
        serde_yaml::from_str(yaml).map_err(|e| DynamicError::Yaml(e.to_string()))?;
    let schemas = doc
        .get("schemas")
        .and_then(|s| s.as_mapping())
        .ok_or(DynamicError::MissingSchemasKey)?;

    let mut out = Vec::with_capacity(schemas.len());
    for (key, def) in schemas {
        let name = key
            .as_str()
            .filter(|n| !n.trim().is_empty())
            .ok_or_else(|| DynamicError::InvalidSchemaName(format!("{key:?}")))?;
        let def = def
            .as_mapping()
            .ok_or_else(|| DynamicError::SchemaNotMapping {
                schema: name.to_string(),
            })?;
        let mut schema = MessageSchema::new(name);
        schema.description = def
            .get(serde_yaml::Value::from("description"))
            .and_then(|d| d.as_str())
            .map(str::to_string);
        let fields = def.get(serde_yaml::Value::from("fields"));
        if let Some(fields) = fields.filter(|f| !f.is_null()) {
            let fields = fields
                .as_mapping()
                .ok_or_else(|| DynamicError::FieldsNotMapping {
                    schema: name.to_string(),
                })?;
            for (field_key, _value) in fields {
                let key_str = field_key
                    .as_str()
                    .ok_or_else(|| DynamicError::InvalidFieldKey {
                        schema: name.to_string(),
                        key: format!("{field_key:?}"),
                        reason: "field key is not a string; expected '<type> <name>'".to_string(),
                    })?;
                let (field_type, field_name) = parse_field_key(name, key_str)?;
                if schema.fields.iter().any(|f| f.name == field_name) {
                    return Err(DynamicError::InvalidFieldKey {
                        schema: name.to_string(),
                        key: key_str.to_string(),
                        reason: format!("duplicate field name '{field_name}'"),
                    });
                }
                schema.add_field(FieldDef::new(field_name, field_type));
            }
        }
        out.push(schema);
    }
    Ok(out)
}

/// Parse one schema-YAML field key of the form `<type> <name>` (e.g.
/// `uint32 height`, `float64[] scores`, `Header[3] hdrs`).
///
/// Rejects keys without exactly two whitespace tokens, types
/// [`FieldType::parse`] refuses, and inline lengths above
/// [`MAX_FIXED_ARRAY_LEN`] (see [`validate_fixed_lengths`]).
pub fn parse_field_key(schema: &str, key: &str) -> Result<(FieldType, String), DynamicError> {
    let mut tokens = key.split_whitespace();
    let (Some(type_token), Some(name_token), None) = (tokens.next(), tokens.next(), tokens.next())
    else {
        return Err(DynamicError::InvalidFieldKey {
            schema: schema.to_string(),
            key: key.to_string(),
            reason: format!(
                "expected exactly '<type> <name>' (e.g. 'uint32 height'), got {} token(s)",
                key.split_whitespace().count()
            ),
        });
    };
    let field_type =
        FieldType::parse(type_token).map_err(|reason| DynamicError::InvalidFieldKey {
            schema: schema.to_string(),
            key: key.to_string(),
            reason,
        })?;
    validate_fixed_lengths(schema, key, &field_type)?;
    Ok((field_type, name_token.to_string()))
}

/// Recursively reject inline fixed lengths above [`MAX_FIXED_ARRAY_LEN`]:
/// `FixedArray { length }` (element count, through nested arrays) and
/// `StringFixed(n)` (inline byte capacity). Both feed the fixed-section
/// size, whose unchecked recipe panics on overflow - the cap keeps hostile
/// input on the error path.
pub fn validate_fixed_lengths(
    schema: &str,
    key: &str,
    field_type: &FieldType,
) -> Result<(), DynamicError> {
    let too_large = |variant: &'static str, length: usize| DynamicError::FixedLengthTooLarge {
        schema: schema.to_string(),
        key: key.to_string(),
        variant,
        length,
        max: MAX_FIXED_ARRAY_LEN,
    };
    match field_type {
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            if *length > MAX_FIXED_ARRAY_LEN {
                return Err(too_large("FixedArray", *length));
            }
            validate_fixed_lengths(schema, key, element_type)
        }
        FieldType::StringFixed(length) if *length > MAX_FIXED_ARRAY_LEN => {
            Err(too_large("StringFixed", *length))
        }
        _ => Ok(()),
    }
}

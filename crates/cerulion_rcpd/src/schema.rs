// SPDX-License-Identifier: AGPL-3.0-only
//! The JSON Schema subset a manifest tool's `input_schema` may use, and the
//! check of a call's arguments against it.
//!
//! The root is `{type: object, properties, required?, additionalProperties?}`.
//! A property is one of:
//!
//! - `{type: number | integer, minimum?, maximum?, description?}`; an
//!   integer is any number with no fractional part, so `3.0` is one
//! - `{type: string, maxLength?, description?}`
//! - `{type: boolean, description?}`
//! - `{type: array, items: <number | integer | string | boolean property>,
//!   minItems?, maxItems?, description?}`
//!
//! Any other keyword is refused when the manifest loads ([`check_schema`]),
//! so a schema never states a constraint the daemon would silently skip.
//! `additionalProperties` defaults to `false`: an argument the schema does not
//! name is refused.

use serde_json::{Map, Value};

const ROOT_KEYS: &[&str] = &[
    "type",
    "properties",
    "required",
    "additionalProperties",
    "description",
];
const NUMBER_KEYS: &[&str] = &["type", "minimum", "maximum", "description"];
const STRING_KEYS: &[&str] = &["type", "maxLength", "description"];
const BOOLEAN_KEYS: &[&str] = &["type", "description"];
const ARRAY_KEYS: &[&str] = &["type", "items", "minItems", "maxItems", "description"];

/// Refuse a schema outside the supported subset.
///
/// # Errors
///
/// What is unsupported, and where.
pub fn check_schema(schema: &Value) -> Result<(), String> {
    let root = schema
        .as_object()
        .ok_or_else(|| String::from("must be an object"))?;
    only_keys(root, ROOT_KEYS, "the root")?;
    if root.get("type").and_then(Value::as_str) != Some("object") {
        return Err(String::from("the root must have type: object"));
    }
    let properties = match root.get("properties") {
        None => &Map::new(),
        Some(Value::Object(p)) => p,
        Some(_) => return Err(String::from("properties must be an object")),
    };
    for (name, property) in properties {
        check_property(property, false).map_err(|e| format!("property {name:?}: {e}"))?;
    }
    if let Some(required) = root.get("required") {
        let list = required
            .as_array()
            .ok_or_else(|| String::from("required must be an array"))?;
        for item in list {
            let name = item
                .as_str()
                .ok_or_else(|| String::from("required must list strings"))?;
            if !properties.contains_key(name) {
                return Err(format!("required names {name:?}, which is not a property"));
            }
        }
    }
    if let Some(extra) = root.get("additionalProperties") {
        if !extra.is_boolean() {
            return Err(String::from("additionalProperties must be a boolean"));
        }
    }
    Ok(())
}

fn only_keys(object: &Map<String, Value>, allowed: &[&str], at: &str) -> Result<(), String> {
    match object.keys().find(|k| !allowed.contains(&k.as_str())) {
        Some(key) => Err(format!("{at} uses the unsupported keyword {key:?}")),
        None => Ok(()),
    }
}

fn check_property(property: &Value, in_array: bool) -> Result<(), String> {
    let object = property
        .as_object()
        .ok_or_else(|| String::from("must be an object"))?;
    let kind = object
        .get("type")
        .and_then(Value::as_str)
        .ok_or_else(|| String::from("needs a string type"))?;
    match kind {
        "number" | "integer" => {
            only_keys(object, NUMBER_KEYS, "a number")?;
            for key in ["minimum", "maximum"] {
                if object.get(key).is_some_and(|v| !v.is_number()) {
                    return Err(format!("{key} must be a number"));
                }
            }
            let bound = |key| object.get(key).and_then(Value::as_f64);
            if let (Some(min), Some(max)) = (bound("minimum"), bound("maximum")) {
                if min > max {
                    return Err(format!("minimum {min} is above maximum {max}"));
                }
            }
            Ok(())
        }
        "string" => {
            only_keys(object, STRING_KEYS, "a string")?;
            if object
                .get("maxLength")
                .is_some_and(|v| v.as_u64().is_none())
            {
                return Err(String::from("maxLength must be a non-negative integer"));
            }
            Ok(())
        }
        "boolean" => only_keys(object, BOOLEAN_KEYS, "a boolean"),
        "array" if !in_array => {
            only_keys(object, ARRAY_KEYS, "an array")?;
            let items = object
                .get("items")
                .ok_or_else(|| String::from("an array needs items"))?;
            check_property(items, true).map_err(|e| format!("items: {e}"))?;
            for key in ["minItems", "maxItems"] {
                if object.get(key).is_some_and(|v| v.as_u64().is_none()) {
                    return Err(format!("{key} must be a non-negative integer"));
                }
            }
            let count = |key| object.get(key).and_then(Value::as_u64);
            if let (Some(min), Some(max)) = (count("minItems"), count("maxItems")) {
                if min > max {
                    return Err(format!("minItems {min} is above maxItems {max}"));
                }
            }
            Ok(())
        }
        "array" => Err(String::from("nested arrays are not supported")),
        other => Err(format!("unsupported type {other:?}")),
    }
}

/// Check `args` against a schema that passed [`check_schema`].
///
/// # Errors
///
/// The first violation, naming the argument.
pub fn validate(schema: &Value, args: &Map<String, Value>) -> Result<(), String> {
    let empty = Map::new();
    let properties = schema
        .get("properties")
        .and_then(Value::as_object)
        .unwrap_or(&empty);
    if let Some(required) = schema.get("required").and_then(Value::as_array) {
        for name in required.iter().filter_map(Value::as_str) {
            if !args.contains_key(name) {
                return Err(format!("missing required argument {name:?}"));
            }
        }
    }
    let allow_extra = schema
        .get("additionalProperties")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    for (name, value) in args {
        match properties.get(name) {
            Some(property) => {
                validate_value(property, value).map_err(|e| format!("argument {name:?}: {e}"))?;
            }
            None if allow_extra => {}
            None => return Err(format!("unknown argument {name:?}")),
        }
    }
    Ok(())
}

fn validate_value(property: &Value, value: &Value) -> Result<(), String> {
    let kind = property.get("type").and_then(Value::as_str).unwrap_or("");
    match kind {
        "number" | "integer" => {
            let number = value.as_f64().ok_or_else(|| format!("must be a {kind}"))?;
            // JSON Schema counts 3.0 as an integer; Python clients send it.
            if kind == "integer" && number.fract() != 0.0 {
                return Err(String::from("must be an integer"));
            }
            if let Some(min) = property.get("minimum").and_then(Value::as_f64) {
                if number < min {
                    return Err(format!("{number} is below the minimum {min}"));
                }
            }
            if let Some(max) = property.get("maximum").and_then(Value::as_f64) {
                if number > max {
                    return Err(format!("{number} is above the maximum {max}"));
                }
            }
            Ok(())
        }
        "string" => {
            let text = value
                .as_str()
                .ok_or_else(|| String::from("must be a string"))?;
            if let Some(max) = property.get("maxLength").and_then(Value::as_u64) {
                if text.chars().count() as u64 > max {
                    return Err(format!("is longer than {max} characters"));
                }
            }
            Ok(())
        }
        "boolean" => value
            .is_boolean()
            .then_some(())
            .ok_or_else(|| String::from("must be a boolean")),
        "array" => {
            let items = value
                .as_array()
                .ok_or_else(|| String::from("must be an array"))?;
            if let Some(min) = property.get("minItems").and_then(Value::as_u64) {
                if (items.len() as u64) < min {
                    return Err(format!("needs at least {min} items"));
                }
            }
            if let Some(max) = property.get("maxItems").and_then(Value::as_u64) {
                if items.len() as u64 > max {
                    return Err(format!("takes at most {max} items"));
                }
            }
            let item_schema = property.get("items").unwrap_or(&Value::Null);
            for (i, item) in items.iter().enumerate() {
                validate_value(item_schema, item).map_err(|e| format!("item {i}: {e}"))?;
            }
            Ok(())
        }
        other => Err(format!("schema type {other:?} cannot be checked")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn step_schema() -> Value {
        json!({
            "type": "object",
            "properties": {
                "q_target": {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6},
                "gripper": {"type": "number", "minimum": 0.0, "maximum": 1.0}
            },
            "required": ["q_target", "gripper"]
        })
    }

    fn args(v: Value) -> Map<String, Value> {
        v.as_object().cloned().expect("object")
    }

    #[test]
    fn a_supported_schema_loads_and_accepts_matching_args() {
        check_schema(&step_schema()).expect("schema");
        validate(
            &step_schema(),
            &args(json!({"q_target": [0,0,0,0,0,0.5], "gripper": 1})),
        )
        .expect("valid");
    }

    #[test]
    fn violations_are_refused_with_the_argument_named() {
        let s = step_schema();
        let cases = [
            (
                json!({"q_target": [0,0,0,0,0,0]}),
                "missing required argument \"gripper\"",
            ),
            (
                json!({"q_target": [0,0,0], "gripper": 0}),
                "needs at least 6",
            ),
            (
                json!({"q_target": [0,0,0,0,0,"x"], "gripper": 0}),
                "item 5: must be a number",
            ),
            (
                json!({"q_target": [0,0,0,0,0,0], "gripper": 2}),
                "above the maximum",
            ),
            (
                json!({"q_target": [0,0,0,0,0,0], "gripper": 0, "speed": 9}),
                "unknown argument \"speed\"",
            ),
        ];
        for (a, expected) in cases {
            let err = validate(&s, &args(a.clone())).expect_err("refused");
            assert!(err.contains(expected), "{a}: {err}");
        }
    }

    #[test]
    fn unsupported_keywords_are_refused_at_load() {
        for schema in [
            json!({"type": "object", "properties": {"a": {"type": "string", "pattern": "x"}}}),
            json!({"type": "object", "oneOf": []}),
            json!({"type": "array"}),
            json!({"type": "object", "properties": {"a": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}}}}),
            json!({"type": "object", "properties": {}, "required": ["ghost"]}),
            json!({"type": "object", "properties": {"a": {"type": "string", "minimum": 1}}}),
            json!({"type": "object", "properties": {"a": {"type": "number", "maxLength": 3}}}),
            json!({"type": "object", "properties": {"a": {"type": "boolean", "maximum": 1}}}),
            json!({"type": "object", "properties": {"a": {"type": "number", "minimum": 2, "maximum": 1}}}),
            json!({"type": "object", "properties": {"a": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 2}}}),
        ] {
            assert!(check_schema(&schema).is_err(), "{schema}");
        }
    }

    #[test]
    fn integers_refuse_fractions() {
        let s =
            json!({"type": "object", "properties": {"seed": {"type": "integer", "minimum": 0}}});
        check_schema(&s).expect("schema");
        assert!(validate(&s, &args(json!({"seed": 3}))).is_ok());
        assert!(validate(&s, &args(json!({"seed": 3.0}))).is_ok());
        assert!(validate(&s, &args(json!({"seed": 3.5}))).is_err());
        assert!(validate(&s, &args(json!({"seed": -1}))).is_err());
    }
}

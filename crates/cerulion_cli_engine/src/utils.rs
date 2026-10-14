// SPDX-License-Identifier: AGPL-3.0-only
//! Shared utility functions for the CLI engine.

use crate::error::{CliError, CliResult};

/// Convert snake_case to PascalCase.
pub fn to_pascal_case(s: &str) -> String {
    s.split('_')
        .map(|word| {
            let mut chars = word.chars();
            match chars.next() {
                None => String::new(),
                Some(c) => c.to_uppercase().to_string() + &chars.as_str().to_lowercase(),
            }
        })
        .collect()
}

/// Validate a node type name.
///
/// Rules (matching `cerulion_macros/src/validate.rs`):
/// 1. Non-empty
/// 2. Only `[a-zA-Z0-9_]`
/// 3. Must not start with a digit
/// 4. Max 128 characters
pub fn validate_node_type(name: &str) -> CliResult<()> {
    if name.is_empty() {
        return Err(CliError::Validation(
            "node type name must not be empty".to_string(),
        ));
    }
    if !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err(CliError::Validation(format!(
            "node type name '{}' must contain only ASCII alphanumeric characters and underscores",
            name
        )));
    }
    if name.starts_with(|c: char| c.is_ascii_digit()) {
        return Err(CliError::Validation(format!(
            "node type name '{}' must not start with a digit",
            name
        )));
    }
    if name.len() > 128 {
        return Err(CliError::Validation(format!(
            "node type name '{}' must not exceed 128 characters",
            name
        )));
    }
    Ok(())
}

/// Validate a port name used as an attribute on an embedded Python node.
pub fn validate_python_identifier(name: &str) -> CliResult<()> {
    let mut chars = name.chars();
    let valid_start = chars
        .next()
        .is_some_and(|c| c == '_' || c.is_ascii_alphabetic());
    let valid_rest = chars.all(|c| c == '_' || c.is_ascii_alphanumeric());
    if !valid_start || !valid_rest {
        return Err(CliError::Validation(format!(
            "port name '{name}' is not a valid Python identifier"
        )));
    }
    const KEYWORDS: &[&str] = &[
        "False", "None", "True", "and", "as", "assert", "async", "await", "break", "class",
        "continue", "def", "del", "elif", "else", "except", "finally", "for", "from", "global",
        "if", "import", "in", "is", "lambda", "nonlocal", "not", "or", "pass", "raise", "return",
        "try", "while", "with", "yield",
    ];
    if KEYWORDS.contains(&name) {
        return Err(CliError::Validation(format!(
            "port name '{name}' is a Python keyword"
        )));
    }
    const RESERVED: &[&str] = &[
        "now_ns",
        "request_shutdown",
        "env",
        "loan",
        "tick",
        "init",
        "shutdown",
    ];
    if RESERVED.contains(&name) {
        return Err(CliError::Validation(format!(
            "port name '{name}' is reserved by the Python node runtime"
        )));
    }
    if name.starts_with("_cer_") || name.starts_with("__cerulion") || name.starts_with("__") {
        return Err(CliError::Validation(format!(
            "port name '{name}' uses a prefix reserved by the Python node runtime"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_pascal_case() {
        assert_eq!(to_pascal_case("camera"), "Camera");
        assert_eq!(to_pascal_case("my_node"), "MyNode");
        assert_eq!(to_pascal_case("imu_fusion_v2"), "ImuFusionV2");
    }

    #[test]
    fn test_to_pascal_case_empty() {
        assert_eq!(to_pascal_case(""), "");
    }

    #[test]
    fn test_validate_node_type_valid() {
        assert!(validate_node_type("camera").is_ok());
        assert!(validate_node_type("my_node").is_ok());
        assert!(validate_node_type("Node123").is_ok());
        assert!(validate_node_type("a").is_ok());
    }

    #[test]
    fn test_validate_node_type_empty() {
        let err = validate_node_type("").unwrap_err();
        assert!(err.to_string().contains("must not be empty"));
    }

    #[test]
    fn test_validate_node_type_special_chars() {
        let err = validate_node_type("my-node").unwrap_err();
        assert!(err.to_string().contains("alphanumeric"));
        let err = validate_node_type("my node").unwrap_err();
        assert!(err.to_string().contains("alphanumeric"));
    }

    #[test]
    fn test_validate_node_type_digit_start() {
        let err = validate_node_type("2camera").unwrap_err();
        assert!(err.to_string().contains("must not start with a digit"));
    }

    #[test]
    fn test_validate_node_type_too_long() {
        let long_name = "a".repeat(129);
        let err = validate_node_type(&long_name).unwrap_err();
        assert!(err.to_string().contains("must not exceed 128"));
    }

    #[test]
    fn validate_python_identifier_covers_all_contract_arms() {
        for name in ["inp", "_x", "a1"] {
            assert!(
                validate_python_identifier(name).is_ok(),
                "{name} should pass"
            );
        }
        for name in ["1abc", "with-dash", ""] {
            let err = validate_python_identifier(name).unwrap_err();
            assert!(err.to_string().contains("is not a valid Python identifier"));
        }
        for name in ["class", "None"] {
            let err = validate_python_identifier(name).unwrap_err();
            assert!(err.to_string().contains("is a Python keyword"));
        }
        for name in ["loan", "tick"] {
            let err = validate_python_identifier(name).unwrap_err();
            assert!(err
                .to_string()
                .contains("is reserved by the Python node runtime"));
        }
        for name in [
            "_cer_ctx",
            "_cer_inputs",
            "__cerulion_ports__",
            "__init__",
            "__dict__",
            "__",
            "___",
            "__foo",
        ] {
            let err = validate_python_identifier(name).unwrap_err();
            assert_eq!(
                err.to_string(),
                format!("port name '{name}' uses a prefix reserved by the Python node runtime")
            );
        }
        for name in ["_cerx", "cer_ctx"] {
            assert!(
                validate_python_identifier(name).is_ok(),
                "{name} should pass"
            );
        }
    }
}

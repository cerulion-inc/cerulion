// SPDX-License-Identifier: AGPL-3.0-only
//! The check the build script uses to leave out a workspace schema the
//! generator refused: a real `compile_error!` call in the emitted source
//! counts, a description that only mentions the macro does not.
//!
//! The check is pulled in from `build/refusal.rs` by path, the same file
//! `build.rs` declares, and runs over the generator's real output.

#[path = "../build/refusal.rs"]
mod refusal;

use cerulion_core::codegen::{generate_schema, FieldDef, FieldType, MessageSchema};
use refusal::generator_refused;

fn schema(description: Option<&str>, fields: Vec<FieldDef>) -> MessageSchema {
    MessageSchema {
        name: "Probe".to_string(),
        package: None,
        description: description.map(str::to_string),
        fields,
    }
}

fn described(name: &str, description: &str) -> FieldDef {
    let mut field = FieldDef::new(name, FieldType::U32);
    field.description = Some(description.to_string());
    field
}

const MENTION: &str = "do not emit compile_error!(\"x\") here";

#[test]
fn a_schema_description_that_mentions_the_macro_is_not_a_refusal() {
    let code = generate_schema(&schema(
        Some(MENTION),
        vec![FieldDef::new("count", FieldType::U32)],
    ));
    assert!(
        code.contains("compile_error!("),
        "the text reaches the source"
    );
    assert!(!generator_refused(&code));
}

#[test]
fn a_field_description_that_mentions_the_macro_is_not_a_refusal() {
    let code = generate_schema(&schema(None, vec![described("count", MENTION)]));
    assert!(
        code.contains("compile_error!("),
        "the text reaches the source"
    );
    assert!(!generator_refused(&code));
}

#[test]
fn a_reserved_prefix_field_name_is_a_refusal() {
    let code = generate_schema(&schema(
        None,
        vec![FieldDef::new("__cer_x", FieldType::U32)],
    ));
    assert!(generator_refused(&code));
}

/// The accessor collision check runs on a type with a variable section, so
/// the probe carries a string beside the colliding fixed field.
#[test]
fn a_fixed_field_named_like_a_generated_method_is_a_refusal() {
    let code = generate_schema(&schema(
        None,
        vec![
            FieldDef::new("payload", FieldType::U32),
            FieldDef::new("label", FieldType::String),
        ],
    ));
    assert!(generator_refused(&code));
}

#[test]
fn a_clean_schema_is_not_a_refusal() {
    let code = generate_schema(&schema(None, vec![FieldDef::new("count", FieldType::U32)]));
    assert!(!generator_refused(&code));
}

#[test]
fn only_a_call_at_the_head_of_a_line_counts() {
    assert!(generator_refused("struct A;\ncompile_error!(\"no\");\n"));
    assert!(generator_refused("    compile_error!(\"no\");"));
    assert!(!generator_refused("/// compile_error!(\"no\")\nstruct A;"));
    assert!(!generator_refused("    /// see compile_error!( below"));
    assert!(!generator_refused("// see compile_error! below."));
    assert!(!generator_refused("const S: &str = \"compile_error!(\";"));
    assert!(!generator_refused(""));
}

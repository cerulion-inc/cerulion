// SPDX-License-Identifier: AGPL-3.0-only
//! Generates the fixture messages in `msg/` through the same parse,
//! resolve and generate steps the built-in corpus uses.

use std::fs;
use std::path::Path;

use cerulion_core::codegen::{generate_schema, parse_rosmsg, resolve_fixed_nested};

const PACKAGE: &str = "codegen_fixture";

fn main() {
    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    let mut paths: Vec<_> = fs::read_dir("msg")
        .expect("read msg/")
        .map(|e| e.expect("msg/ entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == "msg"))
        .collect();
    paths.sort();
    let mut schemas = Vec::new();
    for path in &paths {
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .expect("utf-8 name");
        let text = fs::read_to_string(path).expect("read .msg");
        let schema = parse_rosmsg(&text, name, Some(PACKAGE))
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        schemas.push(schema);
    }
    let warnings = resolve_fixed_nested(&mut schemas);
    assert!(
        warnings.is_empty(),
        "fixture schemas must resolve: {warnings:?}"
    );
    let mut code = String::from("#[allow(dead_code, unused_imports)]\npub mod codegen_fixture {\n");
    for schema in &schemas {
        code.push_str(&generate_schema(schema));
        code.push('\n');
    }
    code.push_str("}\n");
    fs::write(Path::new(&out_dir).join("fixture_msgs.rs"), code).expect("write generated code");
    println!("cargo:rerun-if-changed=msg/");
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Generates the fixture messages in `msg/` and the request and response
//! messages of the services in `srv/` through the same parse, resolve and
//! generate steps the built-in corpus uses.

use std::fs;
use std::path::{Path, PathBuf};

use cerulion_core::codegen::{
    generate_schema, parse_rosmsg, parse_rossrv, resolve_fixed_nested, MessageSchema,
};

const PACKAGE: &str = "codegen_fixture";

fn main() {
    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for path in sorted_files("msg", "msg") {
        let text = fs::read_to_string(&path).expect("read .msg");
        let schema = parse_rosmsg(&text, file_stem(&path), Some(PACKAGE))
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        schemas.push(schema);
    }
    for path in sorted_files("srv", "srv") {
        let text = fs::read_to_string(&path).expect("read .srv");
        let file = path.display().to_string();
        let srv = parse_rossrv(&text, file_stem(&path), Some(PACKAGE), &file)
            .unwrap_or_else(|e| panic!("{e}"));
        schemas.push(srv.request);
        schemas.push(srv.response);
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
    println!("cargo:rerun-if-changed=srv/");
}

/// The files in `dir` with extension `ext`, sorted by path.
fn sorted_files(dir: &str, ext: &str) -> Vec<PathBuf> {
    let mut paths: Vec<_> = fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {dir}/: {e}"))
        .map(|e| e.expect("directory entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == ext))
        .collect();
    paths.sort();
    paths
}

/// The file name without its extension, for example `Probe`.
fn file_stem(path: &Path) -> &str {
    path.file_stem()
        .and_then(|s| s.to_str())
        .expect("utf-8 name")
}

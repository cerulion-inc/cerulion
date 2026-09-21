// Generate the workspace `DetectionArray` Rust type from the shared schema
// file `schemas/detections.yaml`.
//
// We read the SAME yaml that the `--resim … --verify` tolerance field-registry
// parses, then run it through the identical codegen phases (parse ->
// resolve_fixed_nested -> generate_schema). That guarantees the wire
// `schema_hash` the generated types STAMP matches the hash the replayer EXPECTS.
// A hardcoded schema here could silently drift and surface as a replay
// schema-drift refusal (exit 2). One yaml, one hash.
use cerulion_core::codegen::{
    generate_schema, resolve_fixed_nested, FieldDef, FieldType, MessageSchema,
};
use std::path::Path;

fn main() {
    // Codegen includes optional test helpers; this node does not enable that feature.
    println!("cargo:rustc-check-cfg=cfg(feature, values(\"test-helpers\"))");
    let manifest = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    let schema_path = Path::new(&manifest).join("../../schemas/detections.yaml");
    println!("cargo:rerun-if-changed={}", schema_path.display());

    let content = std::fs::read_to_string(&schema_path)
        .unwrap_or_else(|e| panic!("read {}: {e}", schema_path.display()));
    let doc: serde_yaml::Value =
        serde_yaml::from_str(&content).expect("parse detections.yaml as YAML");
    let schemas_map = doc
        .get("schemas")
        .and_then(|s| s.as_mapping())
        .expect("detections.yaml: missing top-level 'schemas' mapping");

    // Same parse shape as cerulion_cli_engine::schema_cmd::parse_message_schemas
    // (package-less MessageSchema, "<type> <name>" field keys, declaration
    // order preserved by serde_yaml's ordered Mapping).
    let mut all: Vec<MessageSchema> = Vec::new();
    for (name_v, def_v) in schemas_map {
        let name = name_v.as_str().expect("schema name key must be a string");
        let mut schema = MessageSchema::new(name);
        if let Some(fields) = def_v.get("fields").and_then(|f| f.as_mapping()) {
            for (key_v, _unused) in fields {
                let key = key_v.as_str().expect("field key must be a string");
                let toks: Vec<&str> = key.split_whitespace().collect();
                assert_eq!(
                    toks.len(),
                    2,
                    "field key {key:?} must be '<type> <name>' (e.g. 'float64[] boxes')"
                );
                let ft =
                    FieldType::parse(toks[0]).unwrap_or_else(|e| panic!("field key {key:?}: {e}"));
                schema.add_field(FieldDef::new(toks[1], ft));
            }
        }
        all.push(schema);
    }
    // No-op for all-primitive-array schemas, but run it for exact parity with
    // the codegen + replay-registry resolution phase.
    let _ = resolve_fixed_nested(&mut all);

    let mut out = String::new();
    for schema in &all {
        out.push_str(&generate_schema(schema));
        out.push('\n');
    }

    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR");
    let dest = Path::new(&out_dir).join("detections_schema.rs");
    std::fs::write(&dest, out).expect("write detections_schema.rs");
}

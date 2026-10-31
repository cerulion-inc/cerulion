//! Generates `bench_msgs/ImuSample` through Cerulion's own codegen: the same
//! parse, resolve and generate steps `crates/native_ros2_messages/build.rs`
//! uses. The FlatBuffers and Cap'n Proto code is vendored in `src/` instead,
//! so a build needs no `flatc` or `capnp` binary (README.md, "Generated code").
use cerulion_core::codegen::{generate_schema, parse_rosmsg, resolve_fixed_nested};
use std::fs;
use std::path::Path;

fn main() {
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let text = fs::read_to_string("msg/ImuSample.msg").unwrap();
    let mut schemas = vec![parse_rosmsg(&text, "ImuSample", Some("bench_msgs")).unwrap()];
    let w = resolve_fixed_nested(&mut schemas);
    assert!(w.is_empty(), "{w:?}");
    let mut code = String::from("#[allow(dead_code, unused_imports)]\npub mod bench_msgs {\n");
    for s in &schemas {
        code.push_str(&generate_schema(s));
        code.push('\n');
    }
    code.push_str("}\n");
    fs::write(Path::new(&out_dir).join("bench_msgs.rs"), code).unwrap();
    println!("cargo:rerun-if-changed=msg/");
}

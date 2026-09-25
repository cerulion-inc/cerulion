// SPDX-License-Identifier: AGPL-3.0-only
//! Unit tests for the dynamic facade - hand-written oracles throughout.
//!
//! The `Probe` schema is the shared fixture:
//!
//! ```text
//! uint32    id        fixed @0, 4 B
//! uint8     flag      fixed @4, 1 B  (+3 B trailing padding, align 4)
//! string    name      variable #0 (align 1)
//! float64[] samples   variable #1 (align 8)
//! ```
//!
//! fixed_size 8, offset table at payload [8, 24), data floor 24.

use super::*;

const PROBE_YAML: &str = "\
schemas:
  Probe:
    description: probe fixture
    fields:
      uint32 id: {}
      uint8 flag: {}
      string name: {}
      float64[] samples: {}
";

const TS: u64 = 0x1122_3344_5566_7788;

fn probe_set() -> SchemaSet {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    let warnings = set.add_yaml_str(PROBE_YAML).expect("fixture YAML parses");
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    set
}

/// The `Probe` frame for id=0x01020304, flag=7, name="ab", samples=[1.0,
/// 2.0], timestamp TS, sequence 0 - every byte after the hash hand-derived.
fn probe_oracle_frame(schema_hash: u64) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&schema_hash.to_le_bytes());
    f.extend_from_slice(&[0x50, 0, 0, 0]); // total_size 80
    f.extend_from_slice(&[0x28, 0, 0, 0]); // offset_table_offset 32 + 8
    f.extend_from_slice(&[0x02, 0, 0, 0]); // offset_table_count
    f.extend_from_slice(&[0, 0, 0, 0]); // sequence
    f.extend_from_slice(&[0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]); // ts
                                                                            // fixed section
    f.extend_from_slice(&[0x04, 0x03, 0x02, 0x01, 0x07, 0, 0, 0]);
    // offset table: name @24 len 2, samples @32 len 16
    f.extend_from_slice(&[0x18, 0, 0, 0, 0x02, 0, 0, 0]);
    f.extend_from_slice(&[0x20, 0, 0, 0, 0x10, 0, 0, 0]);
    // name "ab" + 6 B alignment padding to reach 32
    f.extend_from_slice(&[0x61, 0x62, 0, 0, 0, 0, 0, 0]);
    // samples 1.0, 2.0 (f64 LE)
    f.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xF0, 0x3F]);
    f.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0x40]);
    assert_eq!(f.len(), 80);
    f
}

fn encode_probe(set: &SchemaSet) -> Vec<u8> {
    let layout = set.layout("Probe").expect("Probe present");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut buf = vec![0xAAu8; 96];
    let mut cur = enc.begin(&mut buf, &[2, 16], TS).expect("begin");
    cur.fixed_field_mut("id")
        .expect("id")
        .copy_from_slice(&0x0102_0304u32.to_le_bytes());
    cur.fixed_field_mut("flag").expect("flag")[0] = 7;
    cur.variable_field_mut("name")
        .expect("name")
        .copy_from_slice(b"ab");
    let s = cur.variable_field_mut("samples").expect("samples");
    s[..8].copy_from_slice(&1.0f64.to_le_bytes());
    s[8..].copy_from_slice(&2.0f64.to_le_bytes());
    let n = cur.finish();
    assert_eq!(n, 80);
    assert!(
        buf[80..].iter().all(|&b| b == 0xAA),
        "bytes past the frame untouched"
    );
    buf.truncate(n);
    buf
}

// ---------------------------------------------------------------- SchemaSet

#[test]
fn yaml_schema_resolves_to_hand_written_layout() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe present");
    assert_eq!(layout.qualified_name, "Probe");
    assert_eq!(layout.fixed_size, 8);
    assert_eq!(layout.fixed_align, 4);
    assert_eq!(layout.data_floor(), 24);
    assert_eq!(
        layout.fixed_fields,
        vec![
            FieldLayout {
                name: "id".into(),
                offset: 0,
                size: 4,
                align: 4,
                field_type: FieldType::U32,
            },
            FieldLayout {
                name: "flag".into(),
                offset: 4,
                size: 1,
                align: 1,
                field_type: FieldType::U8,
            },
        ]
    );
    assert_eq!(
        layout.variable_fields,
        vec![
            VariableFieldLayout {
                name: "name".into(),
                field_type: FieldType::String,
            },
            VariableFieldLayout {
                name: "samples".into(),
                field_type: FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F64),
                },
            },
        ]
    );
    assert_eq!(set.schemas().len(), 1);
    assert_eq!(
        set.schemas()[0].description.as_deref(),
        Some("probe fixture")
    );
    let hash = set.schema_hash("Probe").expect("hash");
    assert_eq!(hash, layout.schema_hash);
    assert_eq!(set.layout_for_hash(hash), Some(layout));
    assert_eq!(set.schema_name_for_hash(hash), Some("Probe"));
    assert_eq!(set.walker().schema_hash_for("Probe"), Some(hash));
    assert!(set.layout("Nope").is_none());
    assert!(set.layout_for_hash(hash ^ 1).is_none());
}

#[test]
fn yaml_error_arms() {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    assert!(matches!(
        set.add_yaml_str("schemas: [unclosed"),
        Err(DynamicError::Yaml(_))
    ));
    assert_eq!(
        set.add_yaml_str("nodes: {}"),
        Err(DynamicError::MissingSchemasKey)
    );
    assert!(matches!(
        set.add_yaml_str("schemas:\n  '': {}\n"),
        Err(DynamicError::InvalidSchemaName(_))
    ));
    assert!(matches!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      uint32 a b: {}\n"),
        Err(DynamicError::InvalidFieldKey { schema, key, .. }) if schema == "A" && key == "uint32 a b"
    ));
    assert!(matches!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      uint8[x] q: {}\n"),
        Err(DynamicError::InvalidFieldKey { .. })
    ));
    assert!(matches!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      7: {}\n"),
        Err(DynamicError::InvalidFieldKey { .. })
    ));
    assert!(matches!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      uint8 x: {}\n      uint16 x: {}\n"),
        Err(DynamicError::InvalidFieldKey { schema, key, reason })
            if schema == "A" && key == "uint16 x" && reason == "duplicate field name 'x'"
    ));
    // Hostile inline lengths stop at the CLI's cap, one above it for both
    // sized variants (the `uint8[usize::MAX]` class that once overflowed
    // `LayoutResolver` never reaches the size recipe).
    assert_eq!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      uint8[1048577] a: {}\n"),
        Err(DynamicError::FixedLengthTooLarge {
            schema: "A".into(),
            key: "uint8[1048577] a".into(),
            variant: "FixedArray",
            length: 1_048_577,
            max: MAX_FIXED_ARRAY_LEN,
        })
    );
    assert!(matches!(
        set.add_yaml_str(
            "schemas:\n  A:\n    fields:\n      string_fixed[18446744073709551615] a: {}\n"
        ),
        Err(DynamicError::FixedLengthTooLarge {
            variant: "StringFixed",
            ..
        })
    ));
    assert!(matches!(
        set.add_yaml_str("schemas:\n  A:\n    fields:\n      uint8[18446744073709551615] a: {}\n"),
        Err(DynamicError::FixedLengthTooLarge { .. })
    ));
    assert!(matches!(
        set.add_rosmsg_str("uint8[1048577] a\n", "R", None),
        Err(DynamicError::FixedLengthTooLarge { schema, key, .. }) if schema == "R" && key == "a"
    ));
    let mut at_cap = set.clone();
    at_cap
        .add_yaml_str("schemas:\n  B:\n    fields:\n      uint8[1048576] a: {}\n")
        .expect("exactly the cap is accepted");
    assert_eq!(
        at_cap.layout("B").expect("B").fixed_size,
        MAX_FIXED_ARRAY_LEN
    );
    assert!(
        set.schemas().is_empty(),
        "a failed add leaves the set unchanged"
    );
}

fn overflow_chain_yaml() -> String {
    "\
schemas:
  S0:
    fields:
      uint8[1048576] a: {}
  S1:
    fields:
      S0[1048576] b: {}
  S2:
    fields:
      S1[1048576] c: {}
  S3:
    fields:
      S2[1048576] d: {}
"
    .to_string()
}

fn overflow_chain_ir() -> Vec<MessageSchema> {
    let array = |element_type| FieldType::FixedArray {
        element_type: Box::new(element_type),
        length: 1_048_576,
    };
    let mut s0 = MessageSchema::new("S0");
    s0.add_field(FieldDef::new("a", array(FieldType::U8)));
    let nested = |name: &str| FieldType::Nested {
        schema_name: name.to_string(),
        package: None,
        fixed: None,
    };
    let mut s1 = MessageSchema::new("S1");
    s1.add_field(FieldDef::new("b", array(nested("S0"))));
    let mut s2 = MessageSchema::new("S2");
    s2.add_field(FieldDef::new("c", array(nested("S1"))));
    let mut s3 = MessageSchema::new("S3");
    s3.add_field(FieldDef::new("d", array(nested("S2"))));
    vec![s0, s1, s2, s3]
}

fn schema_set_snapshot(set: &SchemaSet) -> (Vec<MessageSchema>, Option<WireLayout>, Option<u64>) {
    (
        set.schemas().to_vec(),
        set.layout("Probe").cloned(),
        set.schema_hash("Probe"),
    )
}

fn assert_schema_set_unchanged(
    set: &SchemaSet,
    before: &(Vec<MessageSchema>, Option<WireLayout>, Option<u64>),
) {
    assert_eq!(format!("{:?}", set.schemas()), format!("{:?}", before.0));
    assert_eq!(set.layout("Probe"), before.1.as_ref());
    assert_eq!(set.schema_hash("Probe"), before.2);
}

#[test]
fn composed_overflow_is_refused_before_resolution_and_leaves_the_set_unchanged() {
    let mut set = probe_set();
    let schemas = set.schemas().to_vec();
    let layout = set.layout("Probe").cloned();
    let hash = set.schema_hash("Probe");
    let err = set
        .add_yaml_str(&overflow_chain_yaml())
        .expect_err("composed overflow");
    match err {
        DynamicError::SchemaNotWireRepresentable { detail, .. } => {
            assert!(detail.contains("composed"));
        }
        other => panic!("unexpected error: {other:?}"),
    }
    assert_eq!(format!("{:?}", set.schemas()), format!("{:?}", schemas));
    assert_eq!(set.layout("Probe"), layout.as_ref());
    assert_eq!(set.schema_hash("Probe"), hash);
    assert!(set.layout("S0").is_none());

    let err = SchemaSet::from_schemas(overflow_chain_ir()).expect_err("composed overflow");
    match err {
        DynamicError::SchemaNotWireRepresentable { detail, .. } => {
            assert!(detail.contains("composed"));
        }
        other => panic!("unexpected error: {other:?}"),
    }

    let mut set = SchemaSet::from_schemas(Vec::new()).unwrap().0;
    set.add_rosmsg_str("uint8[1048576] a\n", "S0", None)
        .expect("S0");
    let err = set
        .add_rosmsg_str("S0[1048576] b\n", "S1", None)
        .expect_err("composed overflow");
    assert!(matches!(
        err,
        DynamicError::SchemaNotWireRepresentable { .. }
    ));
}

#[test]
fn frame_prefix_over_u32_is_refused_declared_and_resolved() {
    let fields = (0..4096)
        .map(|i| format!("      uint8[1048576] f{i}: {{}}\n"))
        .collect::<String>();
    let yaml = format!("schemas:\n  Huge:\n    fields:\n{fields}");
    let mut set = SchemaSet::from_schemas(Vec::new()).unwrap().0;
    let err = set.add_yaml_str(&yaml).expect_err("prefix exceeds wire");
    assert!(matches!(
        err,
        DynamicError::SchemaNotWireRepresentable { ref detail, .. } if detail.contains("frame prefix")
    ));

    set.add_yaml_str("schemas:\n  Big:\n    fields:\n      uint8[1048576] a: {}\n")
        .expect("Big alone fits");
    let err = set
        .add_yaml_str("schemas:\n  Wrap:\n    fields:\n      Big[4096] b: {}\n")
        .expect_err("resolved prefix exceeds wire");
    assert!(matches!(
        err,
        DynamicError::SchemaNotWireRepresentable { ref detail, .. } if detail.contains("frame prefix")
    ));
    assert!(set.layout("Wrap").is_none());
}

#[test]
fn yaml_structural_arms() {
    let mut set = SchemaSet::from_schemas(Vec::new()).unwrap().0;
    for value in ["null", "3", "[a]"] {
        assert!(matches!(
            set.add_yaml_str(&format!("schemas:\n  Foo: {value}\n")),
            Err(DynamicError::SchemaNotMapping { schema }) if schema == "Foo"
        ));
    }
    for fields in ["bad", "[x]"] {
        assert!(matches!(
            set.add_yaml_str(&format!("schemas:\n  Foo:\n    fields: {fields}\n")),
            Err(DynamicError::FieldsNotMapping { schema }) if schema == "Foo"
        ));
    }
    for yaml in ["schemas:\n  Foo: {}\n", "schemas:\n  Foo:\n    fields:\n"] {
        set.add_yaml_str(yaml).expect("empty fields accepted");
        assert_eq!(set.layout("Foo").expect("Foo").fixed_size, 0);
    }
}

#[test]
fn layout_by_name_returns_the_named_schema() {
    let mut set = SchemaSet::from_schemas(Vec::new()).unwrap().0;
    set.add_yaml_str(
        "schemas:\n  A:\n    fields:\n      uint8 a: {}\n  B:\n    fields:\n      uint16 b: {}\n  C:\n    fields:\n      string c: {}\n",
    )
    .expect("schemas");
    for name in ["A", "B", "C"] {
        let layout = set.layout(name).expect("named layout");
        assert_eq!(layout.qualified_name, name);
        assert_eq!(Some(layout), set.walker().layout_for_name(name));
    }
}

#[test]
fn rollback_leaves_the_set_unchanged_for_every_failing_input_class() {
    let mut set = probe_set();

    let before = schema_set_snapshot(&set);
    assert!(matches!(
        set.add_yaml_str("schemas:\n  Bad:\n    fields:\n      uint8[1048577] x: {}\n"),
        Err(DynamicError::FixedLengthTooLarge { .. })
    ));
    assert_schema_set_unchanged(&set, &before);
    let before = schema_set_snapshot(&set);
    assert!(matches!(
        set.add_yaml_str("schemas: [unclosed"),
        Err(DynamicError::Yaml(_))
    ));
    assert_schema_set_unchanged(&set, &before);
    let before = schema_set_snapshot(&set);
    assert!(matches!(
        set.add_yaml_str("nodes: {}"),
        Err(DynamicError::MissingSchemasKey)
    ));
    assert_schema_set_unchanged(&set, &before);
    let before = schema_set_snapshot(&set);
    assert!(matches!(
        set.add_yaml_str("schemas:\n  Bad: 3\n"),
        Err(DynamicError::SchemaNotMapping { .. })
    ));
    assert_schema_set_unchanged(&set, &before);
    let before = schema_set_snapshot(&set);
    assert!(matches!(
        set.add_yaml_str("schemas:\n  Bad:\n    fields: bad\n"),
        Err(DynamicError::FieldsNotMapping { .. })
    ));
    assert_schema_set_unchanged(&set, &before);
    let before = schema_set_snapshot(&set);
    let err = set
        .add_yaml_str(&overflow_chain_yaml())
        .expect_err("composed overflow");
    match err {
        DynamicError::SchemaNotWireRepresentable { detail, .. } => {
            assert!(detail.contains("composed"));
        }
        other => panic!("unexpected error: {other:?}"),
    }
    assert_schema_set_unchanged(&set, &before);
}

#[test]
fn workspace_dir_skips_composed_overflow_and_over_ceiling_schemas_loudly() {
    let dir = tempfile::tempdir().expect("tempdir");
    let ws = dir.path();
    std::fs::create_dir_all(ws.join("schemas")).expect("mkdir");
    std::fs::write(ws.join("schemas/good.yaml"), PROBE_YAML).expect("write");
    std::fs::write(ws.join("schemas/chain.yaml"), overflow_chain_yaml()).expect("write");
    std::fs::write(
        ws.join("schemas/wrap.yaml"),
        "schemas:\n  Big:\n    fields:\n      uint8[1048576] a: {}\n  Wrap:\n    fields:\n      Big[4096] b: {}\n",
    )
    .expect("write");

    let (first, warnings_first) = SchemaSet::from_workspace_dir(ws).expect("loads");
    let (second, warnings_second) = SchemaSet::from_workspace_dir(ws).expect("loads");
    assert_eq!(warnings_first, warnings_second);
    assert!(first.layout("Probe").is_some());
    assert_eq!(
        first
            .schemas()
            .iter()
            .map(MessageSchema::qualified_name)
            .collect::<Vec<_>>(),
        second
            .schemas()
            .iter()
            .map(MessageSchema::qualified_name)
            .collect::<Vec<_>>()
    );
    for name in ["S1", "S2", "S3", "Wrap"] {
        assert!(first.layout(name).is_none(), "{name} unexpectedly loaded");
    }
    for name in ["S1", "S2", "S3", "Wrap"] {
        assert!(
            warnings_first.iter().any(|warning| warning.contains(name)),
            "warning missing {name}: {warnings_first:?}"
        );
    }
    for name in ["S0", "Big"] {
        assert!(first.layout(name).is_some(), "{name} unexpectedly skipped");
    }
}

#[test]
fn dynamic_parse_rosmsg_is_checked() {
    assert!(matches!(
        parse_rosmsg("float64[18446744073709551615] a\n", "Bad", None),
        Err(DynamicError::FixedLengthTooLarge { .. })
    ));
    let schema = parse_rosmsg("uint8 x\n", "Ok", Some("pkg")).expect("checked parse");
    assert_eq!(schema.qualified_name(), "pkg/Ok");
}

#[test]
fn rosmsg_schema_in_package_resolves_and_errors_report_name() {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    let warnings = set
        .add_rosmsg_str("uint32 a\nstring b\n", "Msg", Some("pkg"))
        .expect("parses");
    assert!(warnings.is_empty());
    let layout = set.layout("pkg/Msg").expect("qualified name");
    assert_eq!(layout.fixed_size, 4);
    assert_eq!(layout.variable_fields.len(), 1);
    assert!(
        set.layout("Msg").is_none(),
        "bare name is not the qualified name"
    );

    let err = set
        .add_rosmsg_str("uint32\n", "Broken", None)
        .expect_err("a type without a name is rejected");
    assert!(
        matches!(err, DynamicError::Rosmsg { ref name, .. } if name == "Broken"),
        "{err}"
    );
    assert_eq!(set.schemas().len(), 1);
}

#[test]
fn nested_fixed_target_inlines_and_unresolved_target_warns() {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    let w = set
        .add_rosmsg_str("int32 sec\nuint32 nanosec\n", "Time", Some("bi"))
        .expect("Time");
    assert!(w.is_empty());
    let w = set
        .add_rosmsg_str("bi/Time stamp\nstring frame_id\n", "Header", Some("std"))
        .expect("Header");
    assert!(w.is_empty(), "{w:?}");
    let header = set.layout("std/Header").expect("Header");
    assert_eq!(header.fixed_size, 8, "Time inlined as 8 fixed bytes");
    assert_eq!(header.fixed_fields[0].name, "stamp");
    assert_eq!(header.fixed_fields[0].size, 8);
    assert!(matches!(
        header.fixed_fields[0].field_type,
        FieldType::Nested { fixed: Some(_), .. }
    ));

    let w = set
        .add_rosmsg_str("ghost/Missing m\n", "Dangling", Some("std"))
        .expect("parses");
    assert!(
        w.iter().any(|s| s.contains("Missing")),
        "unresolved nested must surface a warning naming the target: {w:?}"
    );
    let dangling = set.layout("std/Dangling").expect("still laid out");
    assert_eq!(dangling.fixed_size, 0);
    assert_eq!(
        dangling.variable_fields.len(),
        1,
        "unresolved nested is variable"
    );
}

#[test]
fn from_schemas_owns_the_ir_and_matches_add_paths() {
    let mut set_a = SchemaSet::from_schemas(Vec::new()).unwrap().0;
    set_a.add_yaml_str(PROBE_YAML).expect("yaml");
    let (set_b, w) = SchemaSet::from_schemas(set_a.schemas().to_vec()).expect("schemas preflight");
    assert!(w.is_empty());
    assert_eq!(set_b.layout("Probe"), set_a.layout("Probe"));
}

#[test]
fn workspace_dir_loads_yaml_and_msg_store_and_skips_bad_files_loudly() {
    let dir = tempfile::tempdir().expect("tempdir");
    let ws = dir.path();
    std::fs::create_dir_all(ws.join("schemas/pkg/msg")).expect("mkdir");
    std::fs::write(ws.join("schemas/probe.yaml"), PROBE_YAML).expect("write");
    std::fs::write(ws.join("schemas/pkg/msg/Pt.msg"), "float32 x\nfloat32 y\n").expect("write");
    std::fs::write(ws.join("schemas/broken.yaml"), "schemas: [").expect("write");
    std::fs::write(ws.join("schemas/pkg/msg/Bad.msg"), "uint32\n").expect("write");
    std::fs::write(ws.join("schemas/notes.txt"), "ignored").expect("write");

    let (set, warnings) = SchemaSet::from_workspace_dir(ws).expect("loads");
    assert_eq!(set.schemas().len(), 2, "{warnings:?}");
    assert!(set.layout("Probe").is_some());
    let pt = set
        .layout("pkg/Pt")
        .expect("msg store schema is package-qualified");
    assert_eq!(pt.fixed_size, 8);
    assert_eq!(warnings.len(), 2, "{warnings:?}");
    assert!(
        warnings[0].contains("broken.yaml"),
        "yaml files first: {warnings:?}"
    );
    assert!(warnings[1].contains("Bad.msg"), "{warnings:?}");
}

#[test]
fn workspace_without_schemas_dir_is_empty_and_unreadable_dir_is_io_error() {
    let dir = tempfile::tempdir().expect("tempdir");
    let (set, w) = SchemaSet::from_workspace_dir(dir.path()).expect("ok");
    assert!(set.schemas().is_empty() && w.is_empty());

    // `schemas` exists but is a FILE: listing it fails.
    std::fs::write(dir.path().join("schemas"), "not a dir").expect("write");
    let err = SchemaSet::from_workspace_dir(dir.path()).expect_err("cannot list");
    assert!(
        matches!(err, DynamicError::Io { ref path, .. } if path.ends_with("schemas")),
        "{err}"
    );
}

// --------------------------------------------------------------------- JSON

fn hand_built_probe_layout() -> WireLayout {
    WireLayout {
        qualified_name: "Probe".into(),
        schema_hash: 0x0102_0304_0506_0708,
        fixed_size: 8,
        fixed_align: 4,
        fixed_fields: vec![
            FieldLayout {
                name: "id".into(),
                offset: 0,
                size: 4,
                align: 4,
                field_type: FieldType::U32,
            },
            FieldLayout {
                name: "flag".into(),
                offset: 4,
                size: 1,
                align: 1,
                field_type: FieldType::U8,
            },
        ],
        variable_fields: vec![
            VariableFieldLayout {
                name: "name".into(),
                field_type: FieldType::String,
            },
            VariableFieldLayout {
                name: "samples".into(),
                field_type: FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F64),
                },
            },
        ],
    }
}

const PROBE_JSON: &str = concat!(
    r#"{"qualified_name":"Probe","schema_hash":72623859790382856,"fixed_size":8,"fixed_align":4,"#,
    r#""fixed_fields":[{"name":"id","offset":0,"size":4,"align":4,"field_type":"U32"},"#,
    r#"{"name":"flag","offset":4,"size":1,"align":1,"field_type":"U8"}],"#,
    r#""variable_fields":[{"name":"name","field_type":"String"},"#,
    r#"{"name":"samples","field_type":{"DynamicArray":{"element_type":"F64"}}}]}"#
);

#[test]
fn layout_json_matches_hand_written_oracle_and_is_stable() {
    let layout = hand_built_probe_layout();
    let json = layout.to_json().expect("serializes");
    assert_eq!(json, PROBE_JSON);
    assert_eq!(layout.to_json().expect("again"), json, "deterministic");
    let back: WireLayout = serde_json::from_str(PROBE_JSON).expect("oracle deserializes");
    assert_eq!(back, layout);

    // The resolver's layout for the same schema differs from the hand-built
    // one only in the real hash.
    let set = probe_set();
    let resolved = set.layout("Probe").expect("Probe");
    let expected = PROBE_JSON.replace("72623859790382856", &resolved.schema_hash.to_string());
    assert_eq!(resolved.to_json().expect("json"), expected);
}

#[test]
fn field_type_json_shapes_are_pinned() {
    let cases: [(FieldType, &str); 5] = [
        (FieldType::Bool, r#""Bool""#),
        (FieldType::StringFixed(16), r#"{"StringFixed":16}"#),
        (
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F32),
                length: 3,
            },
            r#"{"FixedArray":{"element_type":"F32","length":3}}"#,
        ),
        (
            FieldType::Nested {
                schema_name: "Header".into(),
                package: Some("std_msgs".into()),
                fixed: None,
            },
            r#"{"Nested":{"schema_name":"Header","package":"std_msgs","fixed":null}}"#,
        ),
        (
            FieldType::Nested {
                schema_name: "Time".into(),
                package: None,
                fixed: Some(NestedFixedInfo {
                    has_large_array: false,
                    fixed_size: 8,
                    alignment: 4,
                    target_hash: 5,
                }),
            },
            concat!(
                r#"{"Nested":{"schema_name":"Time","package":null,"#,
                r#""fixed":{"has_large_array":false,"fixed_size":8,"alignment":4,"target_hash":5}}}"#
            ),
        ),
    ];
    for (ft, oracle) in cases {
        assert_eq!(serde_json::to_string(&ft).expect("ser"), oracle);
        let back: FieldType = serde_json::from_str(oracle).expect("de");
        assert_eq!(back, ft);
    }
}

// ------------------------------------------------------------------ encoder

#[test]
fn encoder_output_matches_hand_written_frame_and_is_deterministic() {
    let set = probe_set();
    let hash = set.schema_hash("Probe").expect("hash");
    let first = encode_probe(&set);
    assert_eq!(first, probe_oracle_frame(hash));
    let second = encode_probe(&set);
    assert_eq!(
        first, second,
        "two encodes of the same input are bit-identical"
    );
}

#[test]
fn encoder_required_len_and_placement_rules() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    assert_eq!(enc.layout(), layout);
    // header 32 + floor 24 + "ab" 2 → 58, align to 8 → 64 (payload 32), +16.
    assert_eq!(enc.required_len(&[2, 16]), Ok(80));
    // Empty variable fields: nothing after the floor except alignment.
    assert_eq!(enc.required_len(&[0, 0]), Ok(56));
    // Name of 8 bytes lands the cursor already aligned: no padding.
    assert_eq!(enc.required_len(&[8, 8]), Ok(72));
    assert_eq!(
        enc.required_len(&[2]),
        Err(DynamicError::VariableCountMismatch {
            expected: 2,
            got: 1
        })
    );
    assert_eq!(
        enc.required_len(&[0, 12]),
        Err(DynamicError::LengthNotElementMultiple {
            field: "samples".into(),
            len: 12,
            elem_size: 8,
        })
    );
    // Max slice length: the largest frame the u32 total_size admits, and
    // one element past it.
    let max_samples = (u32::MAX as usize - 32 - 24) / 8 * 8;
    assert_eq!(
        enc.required_len(&[0, max_samples]),
        Ok(u32::MAX as usize - 7)
    );
    assert!(matches!(
        enc.required_len(&[0, max_samples + 8]),
        Err(DynamicError::FrameTooLarge { .. })
    ));
    assert!(matches!(
        enc.required_len(&[usize::MAX - 1, 0]),
        Err(DynamicError::FrameTooLarge { .. })
    ));
    assert!(matches!(
        enc.required_len(&[usize::MAX, 0]),
        Err(DynamicError::FrameTooLarge { .. })
    ));
    #[cfg(target_pointer_width = "64")]
    assert!(matches!(
        enc.required_len(&[u32::MAX as usize + 1, 0]),
        Err(DynamicError::FrameTooLarge { .. })
    ));

    let mut small = [0u8; 79];
    assert_eq!(
        enc.begin(&mut small, &[2, 16], TS).map(|c| c.finish()),
        Err(DynamicError::BufferTooSmall { need: 80, have: 79 })
    );
}

#[test]
fn encoder_empty_variable_fields_frame_is_all_zero_payload_after_table() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut buf = [0xFFu8; 56];
    let mut cur = enc.begin(&mut buf, &[0, 0], 0).expect("begin");
    assert!(cur.variable_field_mut("name").expect("name").is_empty());
    assert!(cur
        .variable_field_mut("samples")
        .expect("samples")
        .is_empty());
    assert_eq!(cur.fixed_section_mut(), &[0u8; 8]);
    cur.set_sequence(0x0A0B_0C0D);
    assert_eq!(cur.finish(), 56);
    let mut oracle = Vec::new();
    oracle.extend_from_slice(&layout.schema_hash.to_le_bytes());
    oracle.extend_from_slice(&[56, 0, 0, 0, 40, 0, 0, 0, 2, 0, 0, 0, 0x0D, 0x0C, 0x0B, 0x0A]);
    oracle.extend_from_slice(&[0; 8]); // timestamp 0
    oracle.extend_from_slice(&[0; 8]); // fixed
    oracle.extend_from_slice(&[24, 0, 0, 0, 0, 0, 0, 0]); // name @24 len 0
    oracle.extend_from_slice(&[24, 0, 0, 0, 0, 0, 0, 0]); // samples @24 len 0
    assert_eq!(&buf[..], &oracle[..]);
    let view = FrameView::new(set.walker(), &buf).expect("valid");
    assert_eq!(view.sequence(), 0x0A0B_0C0D);
    assert_eq!(view.variable_field("name"), Ok(&[][..]));
    assert_eq!(view.str_field("name"), Ok(""));
    assert_eq!(view.prim_array_field("samples").expect("array").count, 0);
}

#[test]
fn encoder_fixed_only_and_fixed_size_zero_schemas() {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    set.add_yaml_str(
        "schemas:\n  Fixed:\n    fields:\n      uint16 a: {}\n      uint8 b: {}\n  Var:\n    fields:\n      string s: {}\n",
    )
    .expect("yaml");

    let fixed = set.layout("Fixed").expect("Fixed");
    assert_eq!((fixed.fixed_size, fixed.data_floor()), (4, 4));
    let enc = FrameEncoder::new(fixed).expect("valid layout");
    assert_eq!(enc.required_len(&[]), Ok(36));
    let mut buf = [0u8; 36];
    let mut cur = enc.begin(&mut buf, &[], 9).expect("begin");
    cur.fixed_field_mut("a")
        .expect("a")
        .copy_from_slice(&[0x34, 0x12]);
    cur.fixed_field_mut("b").expect("b")[0] = 0x56;
    assert_eq!(
        cur.fixed_field_mut("zz"),
        Err(DynamicError::UnknownFixedField("zz".into()))
    );
    assert_eq!(
        cur.variable_field_mut("a"),
        Err(DynamicError::UnknownVariableField("a".into()))
    );
    assert_eq!(cur.frame().len(), 36);
    assert_eq!(cur.finish(), 36);
    let mut oracle = fixed.schema_hash.to_le_bytes().to_vec();
    oracle.extend_from_slice(&[
        36, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0,
    ]);
    oracle.extend_from_slice(&[0x34, 0x12, 0x56, 0]);
    assert_eq!(&buf[..], &oracle[..]);
    let view = FrameView::new(set.walker(), &buf).expect("valid");
    assert_eq!(view.fixed_field("a"), Ok(&[0x34, 0x12][..]));
    assert_eq!(view.fixed_field_at(1), Ok(&[0x56][..]));
    assert_eq!(view.timestamp_ns(), 9);

    let var = set.layout("Var").expect("Var");
    assert_eq!((var.fixed_size, var.data_floor()), (0, 8));
    let enc = FrameEncoder::new(var).expect("valid layout");
    let mut buf = [0u8; 43];
    let mut cur = enc.begin(&mut buf, &[3], 0).expect("begin");
    assert!(cur.fixed_section_mut().is_empty());
    cur.variable_field_mut_at(0)
        .expect("s")
        .copy_from_slice(b"hey");
    assert_eq!(cur.finish(), 43);
    let mut oracle = var.schema_hash.to_le_bytes().to_vec();
    oracle.extend_from_slice(&[
        43, 0, 0, 0, 32, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ]);
    oracle.extend_from_slice(&[8, 0, 0, 0, 3, 0, 0, 0]);
    oracle.extend_from_slice(b"hey");
    assert_eq!(&buf[..], &oracle[..]);
    let view = FrameView::new(set.walker(), &buf).expect("valid");
    assert_eq!(view.str_field("s"), Ok("hey"));
    assert!(view.fixed_section().is_empty());
}

// --------------------------------------------------------------------- view

#[test]
fn view_reads_every_field_of_the_oracle_frame() {
    let set = probe_set();
    let hash = set.schema_hash("Probe").expect("hash");
    let mut frame = probe_oracle_frame(hash);
    frame.extend_from_slice(&[0xEE; 5]); // trailing junk past total_size is ignored
    let view = FrameView::new(set.walker(), &frame).expect("valid");
    assert_eq!(view.schema_hash(), hash);
    assert_eq!(view.timestamp_ns(), TS);
    assert_eq!(view.sequence(), 0);
    assert_eq!(view.total_size(), 80);
    assert_eq!(view.frame().len(), 80);
    assert_eq!(view.payload().len(), 48);
    assert_eq!(view.header().offset_table_count, 2);
    assert_eq!(view.layout(), set.layout("Probe").expect("Probe"));
    assert_eq!(view.fixed_section(), &[4, 3, 2, 1, 7, 0, 0, 0]);
    assert_eq!(view.fixed_field("id"), Ok(&[4, 3, 2, 1][..]));
    assert_eq!(view.fixed_field("flag"), Ok(&[7][..]));
    assert_eq!(view.variable_field("name"), Ok(&b"ab"[..]));
    assert_eq!(view.str_field("name"), Ok("ab"));
    assert_eq!(view.variable_field_at(1).map(<[u8]>::len), Ok(16));
    let arr = view.prim_array_field("samples").expect("array");
    assert_eq!((arr.elem, arr.count), (PrimType::F64, 2));
    assert_eq!(
        arr.bytes.as_ptr() as usize % 8,
        frame[32..].as_ptr() as usize % 8
    );

    assert_eq!(
        view.fixed_field("name"),
        Err(DynamicError::UnknownFixedField("name".into()))
    );
    assert_eq!(
        view.fixed_field_at(2),
        Err(DynamicError::UnknownFixedField("#2".into()))
    );
    assert_eq!(
        view.variable_field("id"),
        Err(DynamicError::UnknownVariableField("id".into()))
    );
    assert_eq!(
        view.variable_field_at(2),
        Err(DynamicError::UnknownVariableField("#2".into()))
    );
    assert_eq!(
        view.str_field("samples"),
        Err(DynamicError::NotAStringField("samples".into()))
    );
    assert_eq!(
        view.prim_array_field("name"),
        Err(DynamicError::NotAPrimitiveArrayField("name".into()))
    );

    let value = view.decode(set.walker()).expect("walker agrees");
    assert_eq!(value.schema_name, "Probe");
    assert_eq!(value.fields.len(), 4);
    assert!(matches!(
        value.fields[0].value,
        FrameValueKind::U32(0x0102_0304)
    ));
    assert!(matches!(value.fields[2].value, FrameValueKind::Str("ab")));

    let by_layout = FrameView::with_layout(view.layout(), &frame).expect("valid");
    assert_eq!(by_layout.fixed_field("id"), view.fixed_field("id"));
}

fn probe_frame_and_walker() -> (Vec<u8>, SchemaSet) {
    let set = probe_set();
    let frame = probe_oracle_frame(set.schema_hash("Probe").expect("hash"));
    (frame, set)
}

fn view_err(set: &SchemaSet, frame: &[u8]) -> DynamicError {
    FrameView::new(set.walker(), frame)
        .map(|_| ())
        .expect_err("must be rejected")
}

#[test]
fn view_rejects_short_frame_and_unknown_hash() {
    let (frame, set) = probe_frame_and_walker();
    assert_eq!(
        view_err(&set, &frame[..31]),
        DynamicError::FrameTooShort { have: 31, need: 32 }
    );
    let mut bad = frame.clone();
    bad[0] ^= 0xFF;
    let bad_hash = u64::from_le_bytes(bad[..8].try_into().expect("8"));
    assert_eq!(
        view_err(&set, &bad),
        DynamicError::UnknownSchemaHash(bad_hash)
    );
    assert_eq!(
        FrameView::with_layout(set.layout("Probe").expect("Probe"), &bad).map(|_| ()),
        Err(DynamicError::SchemaHashMismatch {
            expected: set.schema_hash("Probe").expect("hash"),
            found: bad_hash,
        })
    );
}

#[test]
fn view_rejects_total_size_out_of_range() {
    let (frame, set) = probe_frame_and_walker();
    let mut big = frame.clone();
    big[8..12].copy_from_slice(&81u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &big),
        DynamicError::TotalSizeExceedsBuffer {
            total_size: 81,
            have: 80
        }
    );
    let mut tiny = frame.clone();
    tiny[8..12].copy_from_slice(&55u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &tiny),
        DynamicError::TotalSizeBelowPrefix {
            schema: "Probe".into(),
            total_size: 55,
            need: 56,
        }
    );
}

#[test]
fn view_rejects_header_offset_table_mismatch() {
    let (frame, set) = probe_frame_and_walker();
    let mut bad = frame.clone();
    bad[12..16].copy_from_slice(&41u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &bad),
        DynamicError::OffsetTableMismatch {
            expected_offset: 40,
            offset: 41,
            expected_count: 2,
            count: 2,
        }
    );
    let mut bad = frame;
    bad[16..20].copy_from_slice(&3u32.to_le_bytes());
    assert!(matches!(
        view_err(&set, &bad),
        DynamicError::OffsetTableMismatch { count: 3, .. }
    ));
}

#[test]
fn view_rejects_offset_below_data_floor() {
    let (mut frame, set) = probe_frame_and_walker();
    // name entry (payload [8..16)) → offset 23, one below the floor.
    frame[40..44].copy_from_slice(&23u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &frame),
        DynamicError::OffsetBelowDataFloor {
            field: "name".into(),
            offset: 23,
            data_floor: 24,
        }
    );
}

#[test]
fn view_rejects_entry_past_total_size() {
    let (mut frame, set) = probe_frame_and_walker();
    // samples entry (payload [16..24)) → len 24 runs past payload_len 48.
    frame[52..56].copy_from_slice(&24u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &frame),
        DynamicError::VariableFieldOutOfBounds {
            field: "samples".into(),
            offset: 32,
            length: 24,
            payload_len: 48,
        }
    );
    // Overflowing offset+len must not wrap.
    frame[48..52].copy_from_slice(&u32::MAX.to_le_bytes());
    frame[52..56].copy_from_slice(&8u32.to_le_bytes());
    assert!(matches!(
        view_err(&set, &frame),
        DynamicError::VariableFieldOutOfBounds {
            offset: u32::MAX,
            ..
        }
    ));
}

#[test]
fn view_rejects_misaligned_and_partial_elements() {
    let (frame, set) = probe_frame_and_walker();
    let mut bad = frame.clone();
    bad[48..52].copy_from_slice(&28u32.to_le_bytes()); // samples @28 (4-aligned only)
    assert_eq!(
        view_err(&set, &bad),
        DynamicError::MisalignedElements {
            field: "samples".into(),
            offset: 28,
            length: 16,
            elem_size: 8,
        }
    );
    let mut bad = frame;
    bad[52..56].copy_from_slice(&12u32.to_le_bytes()); // 1.5 elements
    assert!(matches!(
        view_err(&set, &bad),
        DynamicError::MisalignedElements { length: 12, .. }
    ));
}

#[test]
fn view_rejects_overlapping_entries() {
    let (mut frame, set) = probe_frame_and_walker();
    // name @24 len 10 reaches into samples @32.
    frame[44..48].copy_from_slice(&10u32.to_le_bytes());
    assert_eq!(
        view_err(&set, &frame),
        DynamicError::OverlappingEntries {
            first: "name".into(),
            second: "samples".into(),
        }
    );
    // Empty entries never overlap, wherever they point.
    frame[44..48].copy_from_slice(&0u32.to_le_bytes());
    frame[40..44].copy_from_slice(&40u32.to_le_bytes());
    assert!(FrameView::new(set.walker(), &frame).is_ok());
}

#[test]
fn view_accepts_zero_length_and_unwritten_entries() {
    let (frame, set) = probe_frame_and_walker();
    let mut empty_samples = frame.clone();
    empty_samples[8..12].copy_from_slice(&58u32.to_le_bytes());
    empty_samples[52..56].copy_from_slice(&[0; 4]);
    empty_samples.truncate(58);
    let view = FrameView::new(set.walker(), &empty_samples).expect("empty samples");
    assert_eq!(view.total_size(), 58);
    assert_eq!(view.str_field("name"), Ok("ab"));
    assert_eq!(view.variable_field("samples"), Ok(&[][..]));
    assert_eq!(view.prim_array_field("samples").expect("array").count, 0);

    let mut unwritten_samples = frame.clone();
    unwritten_samples[8..12].copy_from_slice(&58u32.to_le_bytes());
    unwritten_samples[48..56].copy_from_slice(&[0; 8]);
    unwritten_samples.truncate(58);
    let view = FrameView::new(set.walker(), &unwritten_samples).expect("unwritten samples");
    assert_eq!(view.variable_field("samples"), Ok(&[][..]));
    assert_eq!(view.prim_array_field("samples").expect("array").count, 0);

    let mut empty_name = frame;
    empty_name[40..48].copy_from_slice(&[0; 8]);
    let view = FrameView::new(set.walker(), &empty_name).expect("empty name");
    assert_eq!(view.variable_field("name"), Ok(&[][..]));
    assert_eq!(view.prim_array_field("samples").expect("array").count, 2);
}

#[repr(align(8))]
struct AlignedFrame([u8; 96]);

#[test]
fn view_rejects_misaligned_buffer_but_accepts_aligned() {
    let (frame, set) = probe_frame_and_walker();
    let mut aligned = AlignedFrame([0; 96]);
    aligned.0[..frame.len()].copy_from_slice(&frame);
    assert!(FrameView::new(set.walker(), &aligned.0[..frame.len()]).is_ok());

    let mut shifted = AlignedFrame([0; 96]);
    shifted.0[1..][..frame.len()].copy_from_slice(&frame);
    assert_eq!(
        FrameView::new(set.walker(), &shifted.0[1..1 + frame.len()]).map(|_| ()),
        Err(DynamicError::MisalignedBuffer {
            field: "samples".into(),
            offset: 32,
            elem_size: 8,
        })
    );

    let mut empty_samples = frame;
    empty_samples[8..12].copy_from_slice(&58u32.to_le_bytes());
    empty_samples[48..56].copy_from_slice(&[0; 8]);
    empty_samples.truncate(58);
    shifted.0[1..][..empty_samples.len()].copy_from_slice(&empty_samples);
    assert!(FrameView::new(set.walker(), &shifted.0[1..1 + empty_samples.len()]).is_ok());
}

#[test]
fn invalid_supplied_layout_is_refused_not_panicked() {
    let malformed: WireLayout = serde_json::from_str(
        r#"{"qualified_name":"Bad","schema_hash":1,"fixed_size":8,"fixed_align":1,"fixed_fields":[{"name":"x","offset":100,"size":4,"align":1,"field_type":"U8"}],"variable_fields":[]}"#,
    )
    .expect("layout");
    assert_eq!(
        FrameView::with_layout(&malformed, &[]).map(|_| ()),
        Err(DynamicError::InvalidLayout {
            schema: "Bad".into(),
            detail: "a fixed field lies outside the fixed section",
        })
    );
    assert_eq!(
        FrameEncoder::new(&malformed).map(|_| ()),
        Err(DynamicError::InvalidLayout {
            schema: "Bad".into(),
            detail: "a fixed field lies outside the fixed section",
        })
    );

    let prefix: WireLayout = serde_json::from_str(
        r#"{"qualified_name":"Prefix","schema_hash":1,"fixed_size":4294967295,"fixed_align":1,"fixed_fields":[],"variable_fields":[]}"#,
    )
    .expect("layout");
    assert_eq!(
        FrameEncoder::new(&prefix).map(|_| ()),
        Err(DynamicError::InvalidLayout {
            schema: "Prefix".into(),
            detail: "frame prefix exceeds the u32 wire total_size",
        })
    );

    let overflow: WireLayout = serde_json::from_str(
        &format!(
            r#"{{"qualified_name":"Overflow","schema_hash":1,"fixed_size":8,"fixed_align":1,"fixed_fields":[{{"name":"x","offset":{},"size":1,"align":1,"field_type":"U8"}}],"variable_fields":[]}}"#,
            usize::MAX
        ),
    )
    .expect("layout");
    assert!(matches!(
        FrameView::with_layout(&overflow, &[]),
        Err(DynamicError::InvalidLayout {
            detail: "a fixed field lies outside the fixed section",
            ..
        })
    ));
}

#[test]
fn view_reports_invalid_utf8_where_the_walker_degrades_to_bytes() {
    let (mut frame, set) = probe_frame_and_walker();
    frame[56] = 0xFF; // first byte of "ab"
    let view = FrameView::new(set.walker(), &frame).expect("structurally valid");
    assert_eq!(view.variable_field("name"), Ok(&[0xFF, 0x62][..]));
    assert_eq!(
        view.str_field("name"),
        Err(DynamicError::InvalidUtf8 {
            field: "name".into(),
            valid_up_to: 0,
        })
    );
    let value = view.decode(set.walker()).expect("walker accepts");
    assert!(matches!(value.fields[2].value, FrameValueKind::Bytes(_)));
}

#[test]
fn view_decode_surfaces_walker_error_as_walk_variant() {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    set.add_yaml_str("schemas:\n  Nest:\n    fields:\n      Ghost g: {}\n")
        .expect("parses");
    let layout = set.layout("Nest").expect("Nest");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut buf = [0u8; 40];
    enc.begin(&mut buf, &[0], 0).expect("begin").finish();
    let view = FrameView::new(set.walker(), &buf).expect("structurally valid");
    assert!(matches!(
        view.decode(set.walker()),
        Err(DynamicError::Walk(WalkError::NestedResolutionFailed { .. }))
    ));
}

#[test]
fn error_display_names_the_field() {
    let e = DynamicError::OffsetBelowDataFloor {
        field: "name".into(),
        offset: 23,
        data_floor: 24,
    };
    assert_eq!(
        e.to_string(),
        "variable field 'name' offset 23 is below the data floor 24"
    );
    assert!(DynamicError::SchemaNotMapping {
        schema: "Foo".into()
    }
    .to_string()
    .contains("Foo"));
    assert!(DynamicError::FieldsNotMapping {
        schema: "Foo".into()
    }
    .to_string()
    .contains("Foo"));
    assert!(DynamicError::InvalidLayout {
        schema: "Foo".into(),
        detail: "bad",
    }
    .to_string()
    .contains("Foo"));
    assert!(DynamicError::MisalignedBuffer {
        field: "samples".into(),
        offset: 32,
        elem_size: 8,
    }
    .to_string()
    .contains("samples"));
}

#[test]
fn workspace_yaml_wins_a_name_collision_with_the_msg_store() {
    let dir = tempfile::tempdir().expect("tempdir");
    let ws = dir.path();
    std::fs::create_dir_all(ws.join("schemas/pkg/msg")).expect("mkdir");
    std::fs::write(
        ws.join("schemas/a.yaml"),
        "schemas:\n  pkg/Point:\n    fields:\n      uint32 x: {}\n",
    )
    .expect("write");
    std::fs::write(ws.join("schemas/pkg/msg/Point.msg"), "uint64 x\n").expect("write");

    let (set, _) = SchemaSet::from_workspace_dir(ws).expect("loads");
    let point = set.layout("pkg/Point").expect("pkg/Point");
    assert_eq!(
        point.fixed_size, 4,
        "YAML (uint32) must win over the store (uint64)"
    );
}

#[test]
fn workspace_drops_parents_of_a_skipped_schema() {
    let dir = tempfile::tempdir().expect("tempdir");
    let ws = dir.path();
    std::fs::create_dir_all(ws.join("schemas")).expect("mkdir");
    std::fs::write(ws.join("schemas/good.yaml"), PROBE_YAML).expect("write");
    std::fs::write(
        ws.join("schemas/wrap.yaml"),
        "schemas:\n  Big:\n    fields:\n      uint8[1048576] a: {}\n  Wrap:\n    fields:\n      Big[4096] b: {}\n  Holder:\n    fields:\n      Wrap[] items: {}\n",
    )
    .expect("write");

    let (set, warnings) = SchemaSet::from_workspace_dir(ws).expect("loads");
    assert!(set.layout("Probe").is_some());
    assert!(set.layout("Big").is_some());
    assert!(set.layout("Wrap").is_none());
    assert!(set.layout("Holder").is_none(), "{warnings:?}");
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("'Holder'") && w.contains("references skipped schema 'Wrap'")),
        "{warnings:?}"
    );
}

#[test]
fn encoder_rejects_a_misaligned_buffer_before_writing() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe");
    let encoder = FrameEncoder::new(layout).expect("encoder");
    let idx = layout
        .variable_fields
        .iter()
        .position(|f| f.name == "samples")
        .expect("samples");
    let mut lens = vec![0; layout.variable_fields.len()];
    lens[idx] = 8;
    let total = encoder.required_len(&lens).expect("len");
    let mut buf = AlignedFrame([0xAA; 96]);
    let err = encoder
        .begin(&mut buf.0[1..1 + total], &lens, 0)
        .map(|_| ())
        .expect_err("misaligned");
    assert!(
        matches!(err, DynamicError::MisalignedBuffer { ref field, elem_size: 8, .. } if field == "samples"),
        "{err:?}"
    );
    assert!(buf.0.iter().all(|&b| b == 0xAA), "nothing written");

    let aligned = encoder.begin(&mut buf.0[..total], &lens, 0).map(|_| ());
    assert!(aligned.is_ok());
    assert!(FrameView::new(set.walker(), &buf.0[..total]).is_ok());

    let empty = vec![0; layout.variable_fields.len()];
    let short = encoder.required_len(&empty).expect("len");
    assert!(encoder.begin(&mut buf.0[1..1 + short], &empty, 0).is_ok());
}

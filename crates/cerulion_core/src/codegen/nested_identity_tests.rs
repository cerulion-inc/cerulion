// SPDX-License-Identifier: AGPL-3.0-only
//! Nested references bind on schema identity in the layout resolver AND the
//! frame walker: a schema's own `(package, name)`, never its qualified-name
//! text. Every expected offset, size and value below is written by hand from
//! the `#[repr(C)]` rules, never computed by the code under test.

use super::cdr_codec::{CdrCodec, CdrEndianness};
use super::frame_walker::{FrameValueKind, FrameWalker, WalkError};
use super::layout::LayoutResolver;
use super::schema::{FieldDef, FieldType, MessageSchema};
use super::{parse_rosmsg, resolve_fixed_nested};
use crate::shm_runtime::write_offset_entry;
use crate::wire::WireHeader;

fn nested(name: &str, package: Option<&str>) -> FieldType {
    FieldType::Nested {
        schema_name: name.to_string(),
        package: package.map(str::to_string),
        fixed: None,
    }
}

fn with_fields(mut schema: MessageSchema, fields: &[(&str, FieldType)]) -> MessageSchema {
    for (name, field_type) in fields {
        schema.add_field(FieldDef::new(*name, field_type.clone()));
    }
    schema
}

fn loose(name: &str, fields: &[(&str, FieldType)]) -> MessageSchema {
    with_fields(MessageSchema::new(name), fields)
}

fn packaged(pkg: &str, name: &str, fields: &[(&str, FieldType)]) -> MessageSchema {
    with_fields(MessageSchema::new_in_package(name, pkg), fields)
}

/// `[WireHeader][payload]`; the walker reads the schema by name, so the
/// header's hash is irrelevant here.
fn frame(payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(0).write_to_buf(&mut f);
    f.extend_from_slice(payload);
    f
}

/// `(name, offset, size)` of every fixed field of `qname`.
fn fixed_shape(resolver: &mut LayoutResolver, qname: &str) -> Vec<(String, usize, usize)> {
    let layout = resolver.layout_of(qname).expect("layout");
    layout
        .fixed_fields
        .iter()
        .map(|f| (f.name.clone(), f.offset, f.size))
        .collect()
}

fn shape(fields: &[(&str, usize, usize)]) -> Vec<(String, usize, usize)> {
    fields
        .iter()
        .map(|(n, o, s)| (n.to_string(), *o, *s))
        .collect()
}

/// `Root { Leaf leaf; uint8 tag }` in package `pkg2` beside a real
/// `pkg2/Leaf { float64 v }` AND a package-less schema NAMED `pkg2/Leaf`
/// (`{ uint8 v }`), listed last so it owns the qualified text. The bare
/// `Leaf` is the packaged one: leaf@0 size 8, tag@8, padded to 16.
fn twin_set() -> Vec<MessageSchema> {
    vec![
        packaged("pkg2", "Leaf", &[("v", FieldType::F64)]),
        packaged(
            "pkg2",
            "Root",
            &[("leaf", nested("Leaf", None)), ("tag", FieldType::U8)],
        ),
        loose("pkg2/Leaf", &[("v", FieldType::U8)]),
    ]
}

#[test]
fn a_slash_named_schema_never_takes_its_packaged_twins_place_in_a_layout() {
    let (mut resolver, _) = LayoutResolver::new(twin_set());
    assert_eq!(
        fixed_shape(&mut resolver, "pkg2/Root"),
        shape(&[("leaf", 0, 8), ("tag", 8, 1)])
    );
    let root = resolver.layout_of("pkg2/Root").expect("root");
    assert_eq!((root.fixed_size, root.fixed_align), (16, 8));
    // The qualified TEXT still names the later definition (the name lookup
    // is unchanged): the package-less `{ uint8 v }`.
    assert_eq!(
        fixed_shape(&mut resolver, "pkg2/Leaf"),
        shape(&[("v", 0, 1)])
    );
}

#[test]
fn a_slash_named_schema_never_takes_its_packaged_twins_place_in_a_walk() {
    let (walker, _) = FrameWalker::new(twin_set());
    let mut payload = vec![0u8; 16];
    payload[0..8].copy_from_slice(&2.5f64.to_le_bytes());
    payload[8] = 7;
    let f = frame(&payload);
    let value = walker.walk("pkg2/Root", &f).expect("walk");
    match value.field("leaf") {
        Some(FrameValueKind::Nested(leaf)) => {
            assert_eq!(leaf.schema_name, "pkg2/Leaf");
            assert_eq!(leaf.field("v"), Some(&FrameValueKind::F64(2.5)));
        }
        other => panic!("expected the packaged Leaf, got {other:?}"),
    }
    assert_eq!(value.field("tag"), Some(&FrameValueKind::U8(7)));
}

/// A package-less `Root { Leaf leaf }` beside `a/Leaf` NAMED with a slash
/// (package-less, `{ uint8 v }`) and a real `b/Leaf { float64 v }`. Only
/// `b/Leaf` carries the name `Leaf`, so the resolver binds it (a loud
/// cross-package binding); a text match on the last `/` segment saw two
/// candidates and hit a panic. Root: leaf@0 size 8, size 8.
#[test]
fn a_bare_name_two_qualified_texts_end_in_binds_the_one_schema_named_so() {
    let set = vec![
        loose("a/Leaf", &[("v", FieldType::U8)]),
        packaged("b", "Leaf", &[("v", FieldType::F64)]),
        loose("Root", &[("leaf", nested("Leaf", None))]),
    ];
    let (mut resolver, warnings) = LayoutResolver::new(set.clone());
    assert_eq!(warnings.len(), 1, "{warnings:?}");
    assert!(
        warnings[0].contains("CROSS-PACKAGE to 'b/Leaf'"),
        "{warnings:?}"
    );
    assert_eq!(fixed_shape(&mut resolver, "Root"), shape(&[("leaf", 0, 8)]));
    assert_eq!(resolver.layout_of("Root").expect("root").fixed_size, 8);

    let (walker, _) = FrameWalker::new(set);
    let f = frame(&(-1.5f64).to_le_bytes());
    let value = walker.walk("Root", &f).expect("walk");
    match value.field("leaf") {
        Some(FrameValueKind::Nested(leaf)) => {
            assert_eq!(leaf.schema_name, "b/Leaf");
            assert_eq!(leaf.field("v"), Some(&FrameValueKind::F64(-1.5)));
        }
        other => panic!("expected b/Leaf, got {other:?}"),
    }
}

/// Alone, a package-less `pkg2/Leaf` is not in package `pkg2` (a bare `Leaf`
/// inside `pkg2/UsesBare` does not bind it), not a bare `Leaf` (nor inside
/// the package-less `UsesBare`), and not the target of a qualified
/// `pkg2/Leaf`. It binds only by its own name. Unbound references stay
/// variable in the layout and fail the walk with a typed error.
#[test]
fn a_package_less_slash_named_schema_is_neither_in_that_package_nor_a_bare_name() {
    let set = vec![
        loose("pkg2/Leaf", &[("v", FieldType::U8)]),
        packaged("pkg2", "UsesBare", &[("leaf", nested("Leaf", None))]),
        loose("UsesBare", &[("leaf", nested("Leaf", None))]),
        loose("UsesQualified", &[("leaf", nested("Leaf", Some("pkg2")))]),
        loose("UsesOwnName", &[("leaf", nested("pkg2/Leaf", None))]),
    ];
    let (mut resolver, warnings) = LayoutResolver::new(set.clone());
    assert_eq!(warnings.len(), 3, "{warnings:?}");
    for qname in ["pkg2/UsesBare", "UsesBare", "UsesQualified"] {
        let layout = resolver.layout_of(qname).expect("layout");
        assert_eq!(layout.fixed_size, 0, "{qname}");
        assert_eq!(layout.variable_fields.len(), 1, "{qname}");
        assert_eq!(layout.variable_fields[0].name, "leaf", "{qname}");
    }
    assert_eq!(
        fixed_shape(&mut resolver, "UsesOwnName"),
        shape(&[("leaf", 0, 1)])
    );

    // All-variable holder: one offset entry, the nested sub-frame after it
    // (`{ uint8 v }` is fixed, so the sub-frame is its single byte).
    let mut payload = vec![0u8; 8];
    write_offset_entry(&mut payload, 0, 0, 8, 1);
    payload.push(9);
    let (walker, _) = FrameWalker::new(set);
    for (qname, target) in [
        ("pkg2/UsesBare", "Leaf"),
        ("UsesBare", "Leaf"),
        ("UsesQualified", "pkg2/Leaf"),
    ] {
        assert_eq!(
            walker.walk(qname, &frame(&payload)),
            Err(WalkError::NestedResolutionFailed {
                field: "<nested>".to_string(),
                target: target.to_string(),
            }),
            "{qname}"
        );
    }
    let f = frame(&[9]);
    let value = walker.walk("UsesOwnName", &f).expect("walk");
    match value.field("leaf") {
        Some(FrameValueKind::Nested(leaf)) => {
            assert_eq!(leaf.schema_name, "pkg2/Leaf");
            assert_eq!(leaf.field("v"), Some(&FrameValueKind::U8(9)));
        }
        other => panic!("expected the slash-named Leaf, got {other:?}"),
    }
}

/// The workspace twin: a YAML entry `pkg/Type` (`{ uint32 a }`, package-less
/// as the YAML loader builds it, last in the set) beside the store's
/// `pkg/Type` (`{ float64 x }`) and the store's `pkg/Holder { Type t; uint8
/// tag }`. The YAML entry keeps the qualified name; the store's own
/// reference keeps the store's `pkg/Type`, the definition the resolver
/// stamped into Holder (t@0 size 8, tag@8, size 16).
#[test]
fn a_workspace_yaml_qualified_twin_keeps_its_name_and_the_store_keeps_its_references() {
    let yaml =
        crate::dynamic::parse_yaml_schemas("schemas:\n  pkg/Type:\n    fields:\n      uint32 a:\n")
            .expect("yaml");
    let mut set = vec![
        parse_rosmsg("float64 x\n", "Type", Some("pkg")).expect("store Type"),
        parse_rosmsg("Type t\nuint8 tag\n", "Holder", Some("pkg")).expect("store Holder"),
    ];
    set.extend(yaml);

    let (mut resolver, _) = LayoutResolver::new(set.clone());
    assert_eq!(
        fixed_shape(&mut resolver, "pkg/Type"),
        shape(&[("a", 0, 4)])
    );
    assert_eq!(
        fixed_shape(&mut resolver, "pkg/Holder"),
        shape(&[("t", 0, 8), ("tag", 8, 1)])
    );
    let holder = resolver.layout_of("pkg/Holder").expect("holder");
    assert_eq!(holder.fixed_size, 16);
    // The layout's size is the one the resolver stamped (no drift between
    // the two passes).
    let mut resolved = set.clone();
    resolve_fixed_nested(&mut resolved);
    assert_eq!(resolved[1].wire_fixed_size(), 16);

    let (walker, _) = FrameWalker::new(set);
    let mut payload = vec![0u8; 16];
    payload[0..8].copy_from_slice(&4.25f64.to_le_bytes());
    payload[8] = 3;
    let f = frame(&payload);
    let value = walker.walk("pkg/Holder", &f).expect("walk");
    match value.field("t") {
        Some(FrameValueKind::Nested(t)) => {
            assert_eq!(t.schema_name, "pkg/Type");
            assert_eq!(t.field("x"), Some(&FrameValueKind::F64(4.25)));
        }
        other => panic!("expected the store Type, got {other:?}"),
    }
    let yaml_frame = frame(&11u32.to_le_bytes());
    let yaml_value = walker.walk("pkg/Type", &yaml_frame).expect("walk yaml");
    assert_eq!(yaml_value.field("a"), Some(&FrameValueKind::U32(11)));
}

/// Two walkers built from the same set decode the same frame identically,
/// and the identity layouts do not depend on how often they are computed.
#[test]
fn identity_binding_is_deterministic() {
    let mut payload = vec![0u8; 16];
    payload[0..8].copy_from_slice(&2.5f64.to_le_bytes());
    let f = frame(&payload);
    let (a, _) = FrameWalker::new(twin_set());
    let (b, _) = FrameWalker::new(twin_set());
    assert_eq!(a.walk("pkg2/Root", &f), b.walk("pkg2/Root", &f));
    assert_eq!(
        a.layout_for_name("pkg2/Root"),
        b.layout_for_name("pkg2/Root")
    );
}

/// The same twin through the public dynamic schema-set seam: the store
/// `pkg/Type { float64 x }` and `pkg/Holder { Type t; uint8 tag }` come in
/// through `from_schemas`, the YAML `pkg/Type { uint32 a }` through
/// `add_yaml_str`, so the set's own preflight runs on the twin set. Holder
/// keeps the store Type (t@0 size 8, tag@8, size 16), and a store-Type frame
/// still decodes by its hash although the YAML entry owns the name.
#[test]
fn the_dynamic_schema_set_keeps_both_twins_reachable() {
    use crate::dynamic::SchemaSet;

    let store = vec![
        parse_rosmsg("float64 x\n", "Type", Some("pkg")).expect("store Type"),
        parse_rosmsg("Type t\nuint8 tag\n", "Holder", Some("pkg")).expect("store Holder"),
    ];
    let (mut set, _) = SchemaSet::from_schemas(store).expect("store set");
    set.add_yaml_str("schemas:\n  pkg/Type:\n    fields:\n      uint32 a:\n")
        .expect("yaml twin");

    assert_eq!(set.layout("pkg/Holder").expect("holder").fixed_size, 16);
    assert_eq!(set.layout("pkg/Type").expect("yaml Type").fixed_size, 4);

    let mut payload = vec![0u8; 16];
    payload[0..8].copy_from_slice(&4.25f64.to_le_bytes());
    payload[8] = 3;
    let holder_frame = frame(&payload);
    let value = set
        .walker()
        .walk("pkg/Holder", &holder_frame)
        .expect("walk holder");
    match value.field("t") {
        Some(FrameValueKind::Nested(t)) => {
            assert_eq!(t.field("x"), Some(&FrameValueKind::F64(4.25)));
        }
        other => panic!("expected the store Type, got {other:?}"),
    }

    // The store Type's own hash, from its own definition (resolution does
    // not change a schema with no nested fields).
    let store_type_hash = parse_rosmsg("float64 x\n", "Type", Some("pkg"))
        .expect("store Type")
        .schema_hash();
    let yaml_type_hash = set.layout("pkg/Type").expect("yaml Type").schema_hash;
    assert_ne!(store_type_hash, yaml_type_hash);
    let mut store_frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(store_type_hash).write_to_buf(&mut store_frame);
    store_frame.extend_from_slice(&1.5f64.to_le_bytes());
    let by_hash = set.walker().walk_by_hash(&store_frame).expect("by hash");
    assert_eq!(by_hash.schema_name, "pkg/Type");
    assert_eq!(by_hash.field("x"), Some(&FrameValueKind::F64(1.5)));
    assert_eq!(
        set.walker().schema_name_for_hash(store_type_hash),
        Some("pkg/Type")
    );
}

/// A SHADOWED definition whose composed fixed section overflows `usize`:
/// `pkg/Big { Leaf[N] arr }` with `Leaf { float64 v }` (8 bytes) and
/// `N = usize::MAX / 4`, so `8 × N` overflows, then a later `pkg/Big
/// { uint8 a }` that takes the identity. The composed-overflow preflight
/// skips shadowed definitions, so the walker must leave the shadowed one
/// out, never panic laying it out. The active `pkg/Big` is 1 byte.
#[test]
fn a_shadowed_schema_whose_fixed_section_overflows_is_left_out_not_a_panic() {
    let huge = FieldType::FixedArray {
        element_type: Box::new(nested("Leaf", None)),
        length: usize::MAX / 4,
    };
    let set = vec![
        packaged("pkg", "Leaf", &[("v", FieldType::F64)]),
        packaged("pkg", "Big", &[("arr", huge)]),
        packaged("pkg", "Big", &[("a", FieldType::U8)]),
    ];
    let (walker, _) = FrameWalker::new(set);
    let big = walker.layout_for_name("pkg/Big").expect("active Big");
    assert_eq!((big.fixed_size, big.fixed_align), (1, 1));
    assert!(walker.wire_oversized().is_empty());
}

/// The layout resolver reports the same overflow as a typed error.
#[test]
fn a_fixed_section_that_overflows_is_a_typed_layout_error() {
    let huge = FieldType::FixedArray {
        element_type: Box::new(nested("Leaf", None)),
        length: usize::MAX / 4,
    };
    let set = vec![
        packaged("pkg", "Leaf", &[("v", FieldType::F64)]),
        packaged("pkg", "Big", &[("arr", huge)]),
        packaged("pkg", "Big", &[("a", FieldType::U8)]),
    ];
    let (mut resolver, _) = LayoutResolver::new(set);
    assert_eq!(
        resolver.layout_at(1),
        Err(super::layout::LayoutError::FixedSectionOverflow {
            schema: "pkg/Big".to_string(),
            field: "arr".to_string(),
        })
    );
}

/// A twin hidden from name lookups is still checked against the wire limit:
/// packaged `pkg/Leaf { float64[2^20] v }` (8 MiB) and packaged `pkg/Big
/// { Leaf[2^20] big }` (2^43 bytes, past the `u32` total size), then a
/// later package-less schema NAMED `pkg/Big { uint8 a }` that owns the
/// qualified text. The packaged Big still decodes by its hash, so the set
/// must refuse it.
#[test]
fn a_hidden_twin_past_the_wire_limit_is_refused() {
    use crate::dynamic::{DynamicError, SchemaSet};

    let big_leaf = FieldType::FixedArray {
        element_type: Box::new(FieldType::F64),
        length: 1 << 20,
    };
    let big_array = FieldType::FixedArray {
        element_type: Box::new(nested("Leaf", None)),
        length: 1 << 20,
    };
    let set = vec![
        packaged("pkg", "Leaf", &[("v", big_leaf)]),
        packaged("pkg", "Big", &[("big", big_array)]),
        loose("pkg/Big", &[("a", FieldType::U8)]),
    ];
    let (walker, _) = FrameWalker::new(set.clone());
    let oversized: Vec<usize> = walker.wire_oversized().iter().map(|&(i, _)| i).collect();
    assert_eq!(oversized, vec![1]);
    match SchemaSet::from_schemas(set) {
        Err(DynamicError::SchemaNotWireRepresentable { schema, detail }) => {
            assert_eq!(schema, "pkg/Big");
            assert_eq!(
                detail,
                "frame prefix (40-byte header + 8796093022208-byte fixed section + 0-entry offset table) exceeds the u32 wire total_size"
            );
        }
        other => panic!("expected a wire-limit refusal, got {:?}", other.map(|_| ())),
    }
}

/// The SAME text with a later definition whose layout fails: `pkg/Big
/// { uint8 a }`, then `pkg/Big { Leaf[N] arr }` with `N = usize::MAX / 4`
/// (overflows). The later definition owns the name, so the name decodes
/// nothing: the earlier layout must not stand in for it, in the walker, the
/// codec or the resolver.
#[test]
fn a_name_whose_final_owner_has_no_layout_decodes_nothing() {
    let huge = FieldType::FixedArray {
        element_type: Box::new(nested("Leaf", None)),
        length: usize::MAX / 4,
    };
    let set = vec![
        packaged("pkg", "Leaf", &[("v", FieldType::F64)]),
        packaged("pkg", "Big", &[("a", FieldType::U8)]),
        packaged("pkg", "Big", &[("arr", huge)]),
    ];
    let (walker, _) = FrameWalker::new(set.clone());
    assert!(!walker.knows("pkg/Big"));
    assert!(walker.layout_for_name("pkg/Big").is_none());
    assert!(walker.walk("pkg/Big", &frame(&[7])).is_err());
    let (codec, _) = CdrCodec::new(set.clone());
    assert!(!codec.knows("pkg/Big"));
    assert!(codec.schema_hash("pkg/Big").is_none());
    let (mut resolver, _) = LayoutResolver::new(set);
    assert!(resolver.layout_of("pkg/Big").is_none());
}

/// The CDR codec binds nested references on identity too. In
/// [`twin_set`], packaged `pkg2/Root { Leaf leaf; uint8 tag }` embeds the
/// packaged `pkg2/Leaf { float64 v }`, while a later package-less schema
/// NAMED `pkg2/Leaf { uint8 v }` owns that text. CDR body: `v = 1.5` (8
/// bytes LE at 0), `tag = 7` at 8. Cerulion payload, by hand: leaf@0 (8
/// bytes), tag@8, padded to 16. Decoding the twin's `uint8 v` instead would
/// shift `tag` and leave bytes unread.
#[test]
fn the_cdr_codec_never_decodes_a_nested_field_with_its_text_twin() {
    let (codec, _) = CdrCodec::new(twin_set());
    let mut body = 1.5f64.to_le_bytes().to_vec();
    body.push(7);
    let frame = codec
        .decode("pkg2/Root", CdrEndianness::Little, &body, 0, 0)
        .expect("decodes with the packaged Leaf");
    let mut payload = 1.5f64.to_le_bytes().to_vec();
    payload.push(7);
    payload.extend_from_slice(&[0; 7]);
    assert_eq!(&frame[WireHeader::SIZE..], payload.as_slice());
    assert_eq!(
        codec.encode("pkg2/Root", CdrEndianness::Little, &frame),
        Ok(body)
    );
}

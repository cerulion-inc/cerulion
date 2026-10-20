// SPDX-License-Identifier: AGPL-3.0-only
//! Nested references bind on schema identity in the layout resolver AND the
//! frame walker: a schema's own `(package, name)`, never its qualified-name
//! text. Every expected offset, size and value below is written by hand from
//! the `#[repr(C)]` rules, never computed by the code under test.

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

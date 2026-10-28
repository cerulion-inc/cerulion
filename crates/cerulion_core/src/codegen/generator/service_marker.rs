// SPDX-License-Identifier: AGPL-3.0-only
//! Service marker emission: one `cerulion_core::__service_marker!` call per
//! `.srv` file, which declares `pub struct <Name>;` with `impl ServiceType`
//! when `cerulion_core`'s `unstable-services` feature is on. The two parts are ordinary schemas that
//! [`super::generate_schema`] emits.

use crate::codegen::nested_index::NestedIndex;
use crate::codegen::rossrv_parser::ServiceDefinition;
use crate::codegen::schema::{FieldType, MessageSchema};

/// Generate the service marker for a parsed `.srv` file.
///
/// Emits a `cerulion_core::__service_marker!` call that declares
/// `pub struct <Name>;` with `impl ServiceType` naming the request and
/// response markers, or nothing with the `unstable-services` feature off. The two message types themselves come from
/// [`super::generate_schema`] over `service.request` and `service.response`, in the
/// same module.
pub fn generate_service_marker(service: &ServiceDefinition) -> String {
    use std::fmt::Write;

    let mut out = String::new();
    let (name, request, response) = (&service.name, &service.request.name, &service.response.name);
    writeln!(out, "cerulion_core::__service_marker! {{").unwrap();
    writeln!(out, "    /// Service `{}`.", service.qualified_name()).unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Service marker. Zero-sized. The request is [`{request}`] and the response"
    )
    .unwrap();
    writeln!(
        out,
        "    /// is [`{response}`]. Declared only with the `unstable-services` feature."
    )
    .unwrap();
    writeln!(
        out,
        "    {name}, {:?}, {request}, {response}",
        service.qualified_name()
    )
    .unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
    out
}

/// Why the marker for `service` cannot be declared in a module whose
/// generated messages pass `is_generated`, or `None` when it can.
///
/// The marker names its two parts, so both must be generated; it also
/// declares a type of the service's own name, so no generated message may
/// take that name. Either failure would stop the whole module compiling.
pub fn service_marker_conflict(
    service: &ServiceDefinition,
    is_generated: impl Fn(&str) -> bool,
) -> Option<String> {
    let missing: Vec<&str> = [&service.request.name, &service.response.name]
        .into_iter()
        .map(String::as_str)
        .filter(|part| !is_generated(part))
        .collect();
    if !missing.is_empty() {
        return Some(format!(
            "service '{}' is not generated: its part(s) {} are not generated",
            service.qualified_name(),
            missing.join(", ")
        ));
    }
    if is_generated(&service.name) {
        return Some(format!(
            "service '{}' is not generated: a message of the same name takes its name",
            service.qualified_name()
        ));
    }
    None
}

/// Why the marker for `service` would name a part the message set has
/// replaced, itself or through a message it embeds, or `None` when the set
/// holds every part, and every message a part reaches, as vendored.
///
/// `vendored` is the corpus the parts were written against, and
/// `in_set(package, name)` returns the message the set holds under that
/// name, before nested resolution. A workspace schema of the same
/// qualified name (a YAML file or a `.msg` in the workspace store) replaces
/// the vendored one; its schema hash stays the same, since the hash covers
/// the name, but its fields can differ, and then `ServiceType` would name
/// the wrong wire type. That holds for a part, and for every message a part
/// embeds, directly or through other messages: an override of
/// `rcl_interfaces/ParameterValue` changes the wire type of
/// `SetParameters_Request` although its own field list stays the same. A
/// message counts as replaced when its fields (name, type, default) or
/// constants differ from the vendored definition; descriptions do not count.
/// A part missing from the set is left to [`service_marker_conflict`].
///
/// Each reference made by a part, or by a message a part reaches, binds over
/// `vendored` through the resolver's own ladder (`NestedIndex::bind`: a
/// qualified name exactly, then the containing message's package, then a
/// bare `Header` to `std_msgs/Header`, then a bare name exactly one vendored
/// message carries), so this check and the layout never disagree on which
/// message a part embeds. A reference that binds nowhere (an unknown name,
/// or a bare name two packages carry) reaches no message here; the resolver
/// reports it.
pub fn service_part_replaced<'a>(
    service: &ServiceDefinition,
    vendored: &[MessageSchema],
    in_set: impl Fn(&str, &str) -> Option<&'a MessageSchema>,
) -> Option<String> {
    let package = service.package.as_deref().unwrap_or_default();
    let parts = [&service.request, &service.response];
    let replaced: Vec<&str> = parts
        .into_iter()
        .filter(|part| in_set(package, &part.name).is_some_and(|held| !same_definition(part, held)))
        .map(|part| part.name.as_str())
        .collect();
    if !replaced.is_empty() {
        return Some(format!(
            "service '{}' is not generated: the workspace replaces its part(s) {} with a different definition",
            service.qualified_name(),
            replaced.join(", ")
        ));
    }

    // Every message the parts reach, walked over the vendored definitions,
    // each visited once (in first-reached order, so the report is stable).
    let index = NestedIndex::from_schemas(vendored);
    let mut reached: Vec<usize> = Vec::new();
    for part in parts {
        push_nested(part, &index, &mut reached);
    }
    let mut embedded: Vec<String> = Vec::new();
    let mut next = 0;
    while next < reached.len() {
        let declared = &vendored[reached[next]];
        next += 1;
        let pkg = declared.package.as_deref().unwrap_or_default();
        if in_set(pkg, &declared.name).is_some_and(|held| !same_definition(declared, held)) {
            embedded.push(qualified(pkg, &declared.name));
        }
        push_nested(declared, &index, &mut reached);
    }
    if embedded.is_empty() {
        return None;
    }
    Some(format!(
        "service '{}' is not generated: its parts embed {}, which the workspace replaces with a different definition",
        service.qualified_name(),
        embedded.join(", ")
    ))
}

/// Append the index of each vendored message `schema` references to
/// `reached`, unless it is there already. Each reference binds through
/// `index` with `schema`'s own package as the parent, exactly as the layout
/// resolver binds it; one that binds nowhere is skipped.
fn push_nested(schema: &MessageSchema, index: &NestedIndex, reached: &mut Vec<usize>) {
    for field in &schema.fields {
        let Some((pkg, name)) = nested_target(&field.field_type) else {
            continue;
        };
        let Ok(bound) = index.bind(pkg, name, schema.package.as_deref()) else {
            continue;
        };
        if !reached.contains(&bound.idx) {
            reached.push(bound.idx);
        }
    }
}

/// The `(package, name)` a field references, through any array.
fn nested_target(field_type: &FieldType) -> Option<(Option<&str>, &str)> {
    match field_type {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => Some((package.as_deref(), schema_name.as_str())),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            nested_target(element_type)
        }
        _ => None,
    }
}

fn qualified(package: &str, name: &str) -> String {
    if package.is_empty() {
        name.to_string()
    } else {
        format!("{package}/{name}")
    }
}

/// Same fields (name, type, default) in the same order, and same constants.
fn same_definition(a: &MessageSchema, b: &MessageSchema) -> bool {
    a.constants == b.constants
        && a.fields.len() == b.fields.len()
        && a.fields.iter().zip(&b.fields).all(|(x, y)| {
            x.name == y.name
                && x.field_type == y.field_type
                && x.default_literal == y.default_literal
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A service marker is one marker macro call naming the service and its
    /// two parts.
    #[test]
    fn test_generate_service_marker() {
        let srv = trigger();
        let code = generate_service_marker(&srv);
        let expected = "    Trigger, \"std_srvs/Trigger\", Trigger_Request, Trigger_Response\n}\n";
        assert!(
            code.starts_with("cerulion_core::__service_marker! {\n"),
            "service marker is not a marker macro call:\n{code}"
        );
        assert!(code.contains(expected), "service marker missing:\n{code}");
        assert_eq!(code, generate_service_marker(&srv), "deterministic");
    }

    fn trigger() -> ServiceDefinition {
        crate::codegen::parse_rossrv(
            "---\nbool success\n",
            "Trigger",
            Some("std_srvs"),
            "std_srvs/Trigger.srv",
        )
        .unwrap()
    }

    /// Both parts generated and the name free: the marker is declared.
    #[test]
    fn a_marker_with_both_parts_and_a_free_name_has_no_conflict() {
        let generated = ["Trigger_Request", "Trigger_Response", "Other"];
        assert_eq!(
            service_marker_conflict(&trigger(), |n| generated.contains(&n)),
            None
        );
    }

    /// A dropped part (a workspace override shadows a type it embeds) names
    /// exactly the missing part.
    #[test]
    fn a_missing_request_part_is_named() {
        let generated = ["Trigger_Response"];
        assert_eq!(
            service_marker_conflict(&trigger(), |n| generated.contains(&n)).as_deref(),
            Some("service 'std_srvs/Trigger' is not generated: its part(s) Trigger_Request are not generated")
        );
    }

    /// With no part generated, both are named, request first.
    #[test]
    fn both_missing_parts_are_named_in_order() {
        assert_eq!(
            service_marker_conflict(&trigger(), |_| false).as_deref(),
            Some("service 'std_srvs/Trigger' is not generated: its part(s) Trigger_Request, Trigger_Response are not generated")
        );
    }

    /// A generated message of the marker's own name blocks the marker.
    #[test]
    fn a_message_of_the_marker_name_is_a_conflict() {
        let generated = ["Trigger", "Trigger_Request", "Trigger_Response"];
        assert_eq!(
            service_marker_conflict(&trigger(), |n| generated.contains(&n)).as_deref(),
            Some("service 'std_srvs/Trigger' is not generated: a message of the same name takes its name")
        );
    }

    /// The set holds the part the `.srv` declares: no replacement.
    #[test]
    fn a_part_the_set_holds_unchanged_is_not_replaced() {
        let srv = trigger();
        let parts = [srv.request.clone(), srv.response.clone()];
        let lookup = |n: &str| parts.iter().find(|p| p.name == n);
        assert_eq!(service_part_replaced(&srv, &[], |_, n| lookup(n)), None);
    }

    /// A workspace part of the same name with other fields is named, and only
    /// that part: the marker would bind it as the response.
    #[test]
    fn a_part_with_other_fields_is_replaced() {
        let srv = trigger();
        let other =
            crate::codegen::parse_rosmsg("int32 code\n", "Trigger_Response", Some("std_srvs"))
                .unwrap();
        let parts = [srv.request.clone(), other];
        let lookup = |n: &str| parts.iter().find(|p| p.name == n);
        assert_eq!(
            service_part_replaced(&srv, &[], |_, n| lookup(n)).as_deref(),
            Some("service 'std_srvs/Trigger' is not generated: the workspace replaces its part(s) Trigger_Response with a different definition")
        );
    }

    /// A different comment is not a different definition.
    #[test]
    fn a_part_that_differs_only_in_comments_is_not_replaced() {
        let srv = trigger();
        let commented = crate::codegen::parse_rosmsg(
            "# the result\nbool success # ok\n",
            "Trigger_Response",
            Some("std_srvs"),
        )
        .unwrap();
        let parts = [srv.request.clone(), commented];
        let lookup = |n: &str| parts.iter().find(|p| p.name == n);
        assert_eq!(service_part_replaced(&srv, &[], |_, n| lookup(n)), None);
    }

    fn msg(text: &str, name: &str, package: &str) -> MessageSchema {
        crate::codegen::parse_rosmsg(text, name, Some(package)).unwrap()
    }

    fn srv(text: &str, name: &str) -> ServiceDefinition {
        crate::codegen::parse_rossrv(text, name, Some("demo"), &format!("demo/{name}.srv")).unwrap()
    }

    /// `demo/Set`: the request embeds `demo/Value` (bare) and a bare
    /// `Header`, and `demo/Value` embeds `other/Inner` (qualified).
    fn set_service() -> (ServiceDefinition, Vec<MessageSchema>) {
        let corpus = vec![
            msg("other/Inner inner\n", "Value", "demo"),
            msg("int32 a\n", "Inner", "other"),
            msg("uint32 seq\n", "Header", "std_msgs"),
        ];
        (
            srv("Header header\nValue[] values\n---\nbool ok\n", "Set"),
            corpus,
        )
    }

    /// A set lookup by a schema's own package and name.
    fn find<'a>(set: &'a [MessageSchema]) -> impl Fn(&str, &str) -> Option<&'a MessageSchema> {
        move |pkg, name| {
            set.iter()
                .find(|s| s.package.as_deref() == Some(pkg) && s.name == name)
        }
    }

    /// The set holds every embedded message as the corpus declares it: the
    /// marker stays.
    #[test]
    fn unchanged_embedded_messages_are_not_replacements() {
        let (srv, corpus) = set_service();
        assert_eq!(service_part_replaced(&srv, &corpus, find(&corpus)), None);
    }

    /// An override two levels below a part (`demo/Value` keeps its fields,
    /// `other/Inner` changes) still changes the request's wire type, so the
    /// marker is left out and the override is named.
    #[test]
    fn a_transitive_override_of_an_embedded_message_is_a_replacement() {
        let (srv, corpus) = set_service();
        let mut held = corpus.clone();
        held[1] = msg("float64 a\n", "Inner", "other");
        assert_eq!(
            service_part_replaced(&srv, &corpus, find(&held)).as_deref(),
            Some("service 'demo/Set' is not generated: its parts embed other/Inner, which the workspace replaces with a different definition")
        );
    }

    /// A bare `Header` with no same-package `Header` is `std_msgs/Header`, so
    /// an override of that one is seen.
    #[test]
    fn a_bare_header_resolves_to_std_msgs() {
        let (srv, corpus) = set_service();
        let mut held = corpus.clone();
        held[2] = msg("uint64 seq\n", "Header", "std_msgs");
        assert_eq!(
            service_part_replaced(&srv, &corpus, find(&held)).as_deref(),
            Some("service 'demo/Set' is not generated: its parts embed std_msgs/Header, which the workspace replaces with a different definition")
        );
    }

    /// Two messages that embed each other are walked once each, and a
    /// change to either is found.
    #[test]
    fn a_reference_cycle_terminates() {
        let srv = srv("A a\n---\n", "Loop");
        let corpus = vec![msg("B[] b\n", "A", "demo"), msg("A[] a\n", "B", "demo")];
        assert_eq!(service_part_replaced(&srv, &corpus, find(&corpus)), None);
        let mut held = corpus.clone();
        held[1] = msg("A[] a\nint8 x\n", "B", "demo");
        assert_eq!(
            service_part_replaced(&srv, &corpus, find(&held)).as_deref(),
            Some("service 'demo/Loop' is not generated: its parts embed demo/B, which the workspace replaces with a different definition")
        );
    }

    /// A part missing from the set is not a replacement; the conflict check
    /// reports it.
    #[test]
    fn a_missing_part_is_not_a_replacement() {
        assert_eq!(service_part_replaced(&trigger(), &[], |_, _| None), None);
    }

    /// A bare name no same-package message carries, that exactly one other
    /// package does (the resolver's last step): the part embeds that one, so
    /// an override of it is seen.
    #[test]
    fn a_unique_bare_name_binds_to_the_one_package_that_carries_it() {
        let srv = srv("Leaf leaf\n---\n", "Use");
        let corpus = vec![msg("int32 a\n", "Leaf", "other")];
        assert_eq!(service_part_replaced(&srv, &corpus, find(&corpus)), None);
        let mut held = corpus.clone();
        held[0] = msg("float64 a\n", "Leaf", "other");
        assert_eq!(
            service_part_replaced(&srv, &corpus, find(&held)).as_deref(),
            Some("service 'demo/Use' is not generated: its parts embed other/Leaf, which the workspace replaces with a different definition")
        );
    }

    /// A same-package message wins over a bare-name twin in another package,
    /// and over `std_msgs/Header` for a bare `Header`: the part embeds the
    /// same-package ones, so only their overrides are seen, in field order.
    #[test]
    fn a_same_package_message_wins_over_a_bare_twin_elsewhere() {
        let srv = srv("Header header\nLeaf leaf\n---\n", "Use");
        let corpus = vec![
            msg("int32 a\n", "Leaf", "demo"),
            msg("int32 a\n", "Leaf", "other"),
            msg("uint32 seq\n", "Header", "demo"),
            msg("uint32 seq\n", "Header", "std_msgs"),
        ];
        let mut elsewhere = corpus.clone();
        elsewhere[1] = msg("float64 a\n", "Leaf", "other");
        elsewhere[3] = msg("uint64 seq\n", "Header", "std_msgs");
        assert_eq!(service_part_replaced(&srv, &corpus, find(&elsewhere)), None);
        let mut same = corpus.clone();
        same[0] = msg("float64 a\n", "Leaf", "demo");
        same[2] = msg("uint64 seq\n", "Header", "demo");
        assert_eq!(
            service_part_replaced(&srv, &corpus, find(&same)).as_deref(),
            Some("service 'demo/Use' is not generated: its parts embed demo/Header, demo/Leaf, which the workspace replaces with a different definition")
        );
    }

    /// A bare name two other packages carry binds nowhere, as it does for
    /// the resolver: the check reaches no message through it, so an override
    /// of either twin is not this check's to report.
    #[test]
    fn an_ambiguous_bare_name_reaches_no_message() {
        let srv = srv("Leaf leaf\n---\n", "Use");
        let corpus = vec![msg("int32 a\n", "Leaf", "a"), msg("int32 a\n", "Leaf", "b")];
        let mut held = corpus.clone();
        held[0] = msg("float64 a\n", "Leaf", "a");
        assert_eq!(service_part_replaced(&srv, &corpus, find(&held)), None);
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Service marker emission: one `pub struct <Name>;` with `impl ServiceType`
//! per `.srv` file. The two parts are ordinary schemas that
//! [`super::generate_schema`] emits.

use crate::codegen::rossrv_parser::ServiceDefinition;
use crate::codegen::schema::{FieldType, MessageSchema};

/// Generate the service marker for a parsed `.srv` file.
///
/// Emits `pub struct <Name>;` with `impl ServiceType` naming the request and
/// response markers. The two message types themselves come from
/// [`super::generate_schema`] over `service.request` and `service.response`, in the
/// same module.
pub fn generate_service_marker(service: &ServiceDefinition) -> String {
    use std::fmt::Write;

    let mut out = String::new();
    let (name, request, response) = (&service.name, &service.request.name, &service.response.name);
    writeln!(out, "/// Service `{}`.", service.qualified_name()).unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Service marker. Zero-sized. The request is [`{request}`] and the response"
    )
    .unwrap();
    writeln!(out, "/// is [`{response}`].").unwrap();
    writeln!(out, "#[derive(Debug, Clone, Copy, Default)]").unwrap();
    writeln!(out, "pub struct {name};").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "impl cerulion_core::message::ServiceType for {name} {{"
    )
    .unwrap();
    writeln!(
        out,
        "    const NAME: &'static str = {:?};",
        service.qualified_name()
    )
    .unwrap();
    writeln!(out, "    type Request = {request};").unwrap();
    writeln!(out, "    type Response = {response};").unwrap();
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
/// `in_set(package, name)` returns the message the set holds under that
/// name, before nested resolution, and `vendored(package, name)` the corpus
/// definition the parts were written against. A workspace schema of the same
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
pub fn service_part_replaced<'a>(
    service: &ServiceDefinition,
    vendored: impl Fn(&str, &str) -> Option<&'a MessageSchema>,
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
    let mut reached: Vec<(String, String)> = Vec::new();
    for part in parts {
        push_nested(part, package, &vendored, &mut reached);
    }
    let mut embedded: Vec<String> = Vec::new();
    let mut next = 0;
    while next < reached.len() {
        let (pkg, name) = reached[next].clone();
        next += 1;
        let Some(declared) = vendored(&pkg, &name) else {
            continue;
        };
        if in_set(&pkg, &name).is_some_and(|held| !same_definition(declared, held)) {
            embedded.push(qualified(&pkg, &name));
        }
        push_nested(declared, &pkg, &vendored, &mut reached);
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

/// Append each message `schema` (of `package`) references to `reached`,
/// unless it is there already.
fn push_nested<'a>(
    schema: &MessageSchema,
    package: &str,
    vendored: &impl Fn(&str, &str) -> Option<&'a MessageSchema>,
    reached: &mut Vec<(String, String)>,
) {
    for field in &schema.fields {
        let Some((pkg, name)) = nested_target(&field.field_type) else {
            continue;
        };
        let key = match pkg {
            Some(pkg) => (pkg.to_string(), name.to_string()),
            // A bare reference names a message of the same package; rosidl
            // binds a bare `Header` to `std_msgs/Header` otherwise.
            None if vendored(package, name).is_none() && name == "Header" => {
                ("std_msgs".to_string(), name.to_string())
            }
            None => (package.to_string(), name.to_string()),
        };
        if !reached.contains(&key) {
            reached.push(key);
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

    /// A service marker names its two parts through `ServiceType`.
    #[test]
    fn test_generate_service_marker() {
        let srv = trigger();
        let code = generate_service_marker(&srv);
        let expected = "#[derive(Debug, Clone, Copy, Default)]\n\
            pub struct Trigger;\n\
            \n\
            impl cerulion_core::message::ServiceType for Trigger {\n\
            \x20   const NAME: &'static str = \"std_srvs/Trigger\";\n\
            \x20   type Request = Trigger_Request;\n\
            \x20   type Response = Trigger_Response;\n\
            }\n";
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
        assert_eq!(
            service_part_replaced(&srv, |_, _| None, |_, n| lookup(n)),
            None
        );
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
            service_part_replaced(&srv, |_, _| None, |_, n| lookup(n)).as_deref(),
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
        assert_eq!(
            service_part_replaced(&srv, |_, _| None, |_, n| lookup(n)),
            None
        );
    }

    fn msg(text: &str, name: &str, package: &str) -> MessageSchema {
        crate::codegen::parse_rosmsg(text, name, Some(package)).unwrap()
    }

    /// `demo/Set`: the request embeds `demo/Value` (bare) and a bare
    /// `Header`, and `demo/Value` embeds `other/Inner` (qualified).
    fn set_service() -> (ServiceDefinition, Vec<(&'static str, MessageSchema)>) {
        let srv = crate::codegen::parse_rossrv(
            "Header header\nValue[] values\n---\nbool ok\n",
            "Set",
            Some("demo"),
            "demo/Set.srv",
        )
        .unwrap();
        let corpus = vec![
            ("demo", msg("other/Inner inner\n", "Value", "demo")),
            ("other", msg("int32 a\n", "Inner", "other")),
            ("std_msgs", msg("uint32 seq\n", "Header", "std_msgs")),
        ];
        (srv, corpus)
    }

    fn find<'a>(
        set: &'a [(&'static str, MessageSchema)],
    ) -> impl Fn(&str, &str) -> Option<&'a MessageSchema> {
        move |pkg, name| {
            set.iter()
                .find(|(p, s)| *p == pkg && s.name == name)
                .map(|(_, s)| s)
        }
    }

    /// The set holds every embedded message as the corpus declares it: the
    /// marker stays.
    #[test]
    fn unchanged_embedded_messages_are_not_replacements() {
        let (srv, corpus) = set_service();
        assert_eq!(
            service_part_replaced(&srv, find(&corpus), find(&corpus)),
            None
        );
    }

    /// An override two levels below a part (`demo/Value` keeps its fields,
    /// `other/Inner` changes) still changes the request's wire type, so the
    /// marker is left out and the override is named.
    #[test]
    fn a_transitive_override_of_an_embedded_message_is_a_replacement() {
        let (srv, corpus) = set_service();
        let mut held = corpus.clone();
        held[1].1 = msg("float64 a\n", "Inner", "other");
        assert_eq!(
            service_part_replaced(&srv, find(&corpus), find(&held)).as_deref(),
            Some("service 'demo/Set' is not generated: its parts embed other/Inner, which the workspace replaces with a different definition")
        );
    }

    /// A bare `Header` with no same-package `Header` is `std_msgs/Header`, so
    /// an override of that one is seen.
    #[test]
    fn a_bare_header_resolves_to_std_msgs() {
        let (srv, corpus) = set_service();
        let mut held = corpus.clone();
        held[2].1 = msg("uint64 seq\n", "Header", "std_msgs");
        assert_eq!(
            service_part_replaced(&srv, find(&corpus), find(&held)).as_deref(),
            Some("service 'demo/Set' is not generated: its parts embed std_msgs/Header, which the workspace replaces with a different definition")
        );
    }

    /// Two messages that embed each other are walked once each, and a
    /// change to either is found.
    #[test]
    fn a_reference_cycle_terminates() {
        let srv = crate::codegen::parse_rossrv("A a\n---\n", "Loop", Some("demo"), "demo/Loop.srv")
            .unwrap();
        let corpus = vec![
            ("demo", msg("B[] b\n", "A", "demo")),
            ("demo", msg("A[] a\n", "B", "demo")),
        ];
        assert_eq!(
            service_part_replaced(&srv, find(&corpus), find(&corpus)),
            None
        );
        let mut held = corpus.clone();
        held[1].1 = msg("A[] a\nint8 x\n", "B", "demo");
        assert_eq!(
            service_part_replaced(&srv, find(&corpus), find(&held)).as_deref(),
            Some("service 'demo/Loop' is not generated: its parts embed demo/B, which the workspace replaces with a different definition")
        );
    }

    /// A part missing from the set is not a replacement; the conflict check
    /// reports it.
    #[test]
    fn a_missing_part_is_not_a_replacement() {
        assert_eq!(
            service_part_replaced(&trigger(), |_, _| None, |_, _| None),
            None
        );
    }
}

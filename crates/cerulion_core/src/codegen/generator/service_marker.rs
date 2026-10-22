// SPDX-License-Identifier: AGPL-3.0-only
//! Service marker emission: one `pub struct <Name>;` with `impl ServiceType`
//! per `.srv` file. The two parts are ordinary schemas that
//! [`super::generate_schema`] emits.

use crate::codegen::rossrv_parser::ServiceDefinition;

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
}

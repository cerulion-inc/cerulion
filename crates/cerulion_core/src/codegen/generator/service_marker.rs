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

#[cfg(test)]
mod tests {
    use super::*;

    /// A service marker names its two parts through `ServiceType`.
    #[test]
    fn test_generate_service_marker() {
        let srv = crate::codegen::parse_rossrv(
            "---\nbool success\n",
            "Trigger",
            Some("std_srvs"),
            "std_srvs/Trigger.srv",
        )
        .unwrap();
        let code = generate_service_marker(&srv);
        let expected = "#[derive(Debug, Clone, Copy, Default)]\n\
            pub struct Trigger;\n\
            \n\
            impl cerulion_core::message::ServiceType for Trigger {\n\
            \x20   const NAME: &'static str = \"std_srvs/Trigger\";\n\
            \x20   type Request = TriggerRequest;\n\
            \x20   type Response = TriggerResponse;\n\
            }\n";
        assert!(code.contains(expected), "service marker missing:\n{code}");
        assert_eq!(code, generate_service_marker(&srv), "deterministic");
    }
}

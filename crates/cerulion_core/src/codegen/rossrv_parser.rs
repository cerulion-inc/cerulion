// SPDX-License-Identifier: AGPL-3.0-only
//! ROS 2 `.srv` file parser.
//!
//! A `.srv` file is two `.msg` bodies split by one line that reads `---`:
//!
//! ```text
//! # request part
//! bool data
//! ---
//! # response part
//! bool success
//! string message
//! ```
//!
//! Each part parses with [`parse_rosmsg`] into its own message schema. The
//! request is named `<Name>_Request` and the response `<Name>_Response`, in
//! the same package as the service, so `std_srvs/Trigger` gives
//! `std_srvs/Trigger_Request` and `std_srvs/Trigger_Response`. These are
//! the names ROS 2 gives the two parts, and the schema hash covers the
//! qualified name, so a native peer and a ROS 2 peer on `rmw_cerulion`
//! hash each part the same. Either part can be empty (`std_srvs/Empty` is
//! a lone `---`).

use super::rosmsg_parser::{parse_rosmsg, ParseError};
use super::schema::MessageSchema;

/// The line that splits a `.srv` file into its request and response parts.
pub const SRV_SEPARATOR: &str = "---";

/// The two raw text parts of a `.srv` file, borrowed from the source.
///
/// Each part is exact source text, so it can be stored and parsed again
/// later as an ordinary `.msg` body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SrvParts<'a> {
    /// The text before the separator line.
    pub request: &'a str,
    /// The text after the separator line.
    pub response: &'a str,
    /// The 1-based source line where the response text starts (the line
    /// after the separator). Add `response_line_offset()` to a line number
    /// inside `response` to get the line in the `.srv` file.
    pub response_first_line: usize,
}

impl SrvParts<'_> {
    /// Lines to add to a 1-based line number in the response text to get
    /// the 1-based line number in the `.srv` file.
    pub fn response_line_offset(&self) -> usize {
        // `split_rossrv` always sets a line >= 2; saturate so a hand-built
        // `SrvParts { response_first_line: 0, .. }` cannot underflow.
        self.response_first_line.saturating_sub(1)
    }
}

/// A parsed `.srv` file: the service name and its two message schemas.
#[derive(Debug, Clone)]
pub struct ServiceDefinition {
    /// Service name, for example `Trigger`.
    pub name: String,
    /// Package of the service, for example `std_srvs`.
    pub package: Option<String>,
    /// The request message, named `<name>_Request`.
    pub request: MessageSchema,
    /// The response message, named `<name>_Response`.
    pub response: MessageSchema,
}

impl ServiceDefinition {
    /// Qualified service name: `"pkg/Name"` or the bare `"Name"`.
    pub fn qualified_name(&self) -> String {
        match &self.package {
            Some(pkg) => format!("{pkg}/{}", self.name),
            None => self.name.clone(),
        }
    }
}

/// Name of the request message of service `srv_name`.
pub fn srv_request_name(srv_name: &str) -> String {
    format!("{srv_name}_Request")
}

/// Name of the response message of service `srv_name`.
pub fn srv_response_name(srv_name: &str) -> String {
    format!("{srv_name}_Response")
}

/// What is wrong with a `.srv` file.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SrvErrorKind {
    /// No `---` line in the file.
    #[error("missing `{sep}` line between the request and the response", sep = SRV_SEPARATOR)]
    MissingSeparator,
    /// A second `---` line; `first_line` is where the first one is.
    #[error(
        "a second `{sep}` line (the first is on line {first_line}); \
         a service has exactly one",
        sep = SRV_SEPARATOR
    )]
    ExtraSeparator { first_line: usize },
    /// A line in the request or response part is not a valid `.msg` line.
    #[error("{message}")]
    Field { message: String },
}

/// A `.srv` parse error. It names the file and the 1-based line.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{file}:{line}: {kind}")]
pub struct SrvParseError {
    /// The file name given to the parser, for example `std_srvs/Trigger.srv`.
    pub file: String,
    /// The 1-based line in the `.srv` file. For a missing separator it is
    /// the last line of the file (0 for an empty file).
    pub line: usize,
    /// What is wrong.
    pub kind: SrvErrorKind,
}

/// True when `line` is the request/response separator. Surrounding
/// whitespace is ignored; anything else on the line (a comment, a fourth
/// dash) makes it a normal line.
fn is_separator(line: &str) -> bool {
    line.trim() == SRV_SEPARATOR
}

/// Split `.srv` text into its request and response parts at the one `---`
/// line. `file` is used only in the error.
pub fn split_rossrv<'a>(srv_content: &'a str, file: &str) -> Result<SrvParts<'a>, SrvParseError> {
    let err = |line, kind| SrvParseError {
        file: file.to_string(),
        line,
        kind,
    };
    // (1-based line, byte start of the line, byte just past the line).
    let mut separator: Option<(usize, usize, usize)> = None;
    let mut line_count = 0;
    let mut start = 0;
    for (idx, raw) in srv_content.split_inclusive('\n').enumerate() {
        let line_num = idx + 1;
        line_count = line_num;
        let end = start + raw.len();
        if is_separator(raw) {
            if let Some((first_line, _, _)) = separator {
                return Err(err(line_num, SrvErrorKind::ExtraSeparator { first_line }));
            }
            separator = Some((line_num, start, end));
        }
        start = end;
    }
    match separator {
        Some((line_num, sep_start, sep_end)) => Ok(SrvParts {
            request: &srv_content[..sep_start],
            response: &srv_content[sep_end..],
            response_first_line: line_num + 1,
        }),
        None => Err(err(line_count, SrvErrorKind::MissingSeparator)),
    }
}

/// Parse `.srv` text into a [`ServiceDefinition`].
///
/// `srv_name` is the service name (`Trigger`), `package` its package (it
/// feeds both message schema hashes, as in [`parse_rosmsg`]), and `file` the
/// name shown in an error (`std_srvs/Trigger.srv`). Every error names the
/// file and the line in the `.srv` file.
pub fn parse_rossrv(
    srv_content: &str,
    srv_name: &str,
    package: Option<&str>,
    file: &str,
) -> Result<ServiceDefinition, SrvParseError> {
    let parts = split_rossrv(srv_content, file)?;
    let field_err = |e: ParseError, offset: usize| {
        let (line_num, message) = match e {
            ParseError::InvalidLine { line_num, message } => (line_num, message),
            ParseError::InvalidType { line_num, message } => {
                (line_num, format!("invalid type: {message}"))
            }
        };
        SrvParseError {
            file: file.to_string(),
            line: line_num + offset,
            kind: SrvErrorKind::Field { message },
        }
    };
    let request = parse_rosmsg(parts.request, &srv_request_name(srv_name), package)
        .map_err(|e| field_err(e, 0))?;
    let response = parse_rosmsg(parts.response, &srv_response_name(srv_name), package)
        .map_err(|e| field_err(e, parts.response_line_offset()))?;
    Ok(ServiceDefinition {
        name: srv_name.to_string(),
        package: package.map(str::to_string),
        request,
        response,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::FieldType;

    const FILE: &str = "pkg/Srv.srv";

    fn field_names(schema: &MessageSchema) -> Vec<&str> {
        schema.fields.iter().map(|f| f.name.as_str()).collect()
    }

    #[test]
    fn a_set_bool_service_splits_into_a_request_and_a_response_message() {
        let text = "bool data # enable or disable\n---\nbool success\nstring message\n";
        let srv = parse_rossrv(text, "SetBool", Some("std_srvs"), "std_srvs/SetBool.srv").unwrap();

        assert_eq!(srv.qualified_name(), "std_srvs/SetBool");
        assert_eq!(srv.request.qualified_name(), "std_srvs/SetBool_Request");
        assert_eq!(srv.response.qualified_name(), "std_srvs/SetBool_Response");
        assert_eq!(field_names(&srv.request), ["data"]);
        assert_eq!(srv.request.fields[0].field_type, FieldType::Bool);
        assert_eq!(field_names(&srv.response), ["success", "message"]);
        assert_eq!(srv.response.fields[1].field_type, FieldType::String);
    }

    #[test]
    fn each_part_hashes_like_the_same_text_parsed_as_a_msg() {
        // A service part is an ordinary message: its schema hash is the hash
        // of the same text parsed as `<Name>_Request.msg` in the package.
        let text = "string[] names\n---\nuint8[] types\n";
        let srv = parse_rossrv(text, "GetTypes", Some("p"), FILE).unwrap();
        let req = parse_rosmsg("string[] names\n", "GetTypes_Request", Some("p")).unwrap();
        let resp = parse_rosmsg("uint8[] types\n", "GetTypes_Response", Some("p")).unwrap();
        assert_eq!(srv.request.schema_hash(), req.schema_hash());
        assert_eq!(srv.response.schema_hash(), resp.schema_hash());
        assert_ne!(srv.request.schema_hash(), srv.response.schema_hash());
    }

    /// The qualified name `rmw_cerulion` gives a message from its rosidl
    /// introspection: the package is the namespace up to the first `__`
    /// (`std_srvs__srv` gives `std_srvs`), then `/`, then `message_name_`.
    /// rosidl names the two parts of service `SetBool` `SetBool_Request`
    /// and `SetBool_Response`.
    fn rmw_side_schema(body: &str, namespace: &str, message_name: &str) -> MessageSchema {
        let package = namespace.split("__").next().unwrap();
        parse_rosmsg(body, message_name, Some(package)).unwrap()
    }

    #[test]
    fn each_part_hashes_like_the_rmw_cerulion_peer_of_the_same_service() {
        let request = "bool data\n";
        let response = "bool success\nstring message\n";
        let text = format!("{request}---\n{response}");
        let srv = parse_rossrv(&text, "SetBool", Some("std_srvs"), FILE).unwrap();

        let rmw_req = rmw_side_schema(request, "std_srvs__srv", "SetBool_Request");
        let rmw_resp = rmw_side_schema(response, "std_srvs__srv", "SetBool_Response");
        assert_eq!(rmw_req.qualified_name(), "std_srvs/SetBool_Request");
        assert_eq!(srv.request.schema_hash(), rmw_req.schema_hash());
        assert_eq!(srv.response.schema_hash(), rmw_resp.schema_hash());
    }

    #[test]
    fn a_lone_separator_gives_two_empty_parts() {
        let srv = parse_rossrv("---\n", "Empty", Some("std_srvs"), FILE).unwrap();
        assert!(srv.request.fields.is_empty());
        assert!(srv.response.fields.is_empty());

        let parts = split_rossrv("---", FILE).unwrap();
        assert_eq!(
            parts,
            SrvParts {
                request: "",
                response: "",
                response_first_line: 2
            }
        );
    }

    #[test]
    fn an_empty_request_with_a_response_parses() {
        let text = "# Trigger\n---\nbool success   # ok\nstring message\n";
        let srv = parse_rossrv(text, "Trigger", Some("std_srvs"), FILE).unwrap();
        assert!(srv.request.fields.is_empty());
        assert_eq!(field_names(&srv.response), ["success", "message"]);
    }

    #[test]
    fn the_split_keeps_the_exact_source_text_of_each_part() {
        let text = "# c\nint32 a\n  ---  \r\nint32 b\n# tail";
        let parts = split_rossrv(text, FILE).unwrap();
        assert_eq!(parts.request, "# c\nint32 a\n");
        assert_eq!(parts.response, "int32 b\n# tail");
        assert_eq!(parts.response_first_line, 4);
        assert_eq!(parts.response_line_offset(), 3);
    }

    #[test]
    fn a_hand_built_first_line_of_zero_does_not_underflow_the_offset() {
        let parts = SrvParts {
            request: "",
            response: "",
            response_first_line: 0,
        };
        assert_eq!(parts.response_line_offset(), 0);
    }

    #[test]
    fn a_dash_line_that_is_not_exactly_three_dashes_is_not_the_separator() {
        // `----` and `--- # x` are ordinary lines, so the file has no
        // separator at all.
        let e = split_rossrv("----\n--- # x\n", FILE).unwrap_err();
        assert_eq!(
            e,
            SrvParseError {
                file: FILE.into(),
                line: 2,
                kind: SrvErrorKind::MissingSeparator
            }
        );
    }

    #[test]
    fn a_missing_separator_names_the_file_and_the_last_line() {
        let e = parse_rossrv("bool a\nbool b\n", "S", None, FILE).unwrap_err();
        assert_eq!(
            e,
            SrvParseError {
                file: FILE.into(),
                line: 2,
                kind: SrvErrorKind::MissingSeparator
            }
        );
        assert_eq!(
            e.to_string(),
            "pkg/Srv.srv:2: missing `---` line between the request and the response"
        );

        let e = parse_rossrv("", "S", None, FILE).unwrap_err();
        assert_eq!(e.line, 0);
        assert_eq!(e.kind, SrvErrorKind::MissingSeparator);
    }

    #[test]
    fn a_second_separator_names_both_lines() {
        let e = parse_rossrv("bool a\n---\nbool b\n---\nbool c\n", "S", None, FILE).unwrap_err();
        assert_eq!(
            e,
            SrvParseError {
                file: FILE.into(),
                line: 4,
                kind: SrvErrorKind::ExtraSeparator { first_line: 2 }
            }
        );
        assert_eq!(
            e.to_string(),
            "pkg/Srv.srv:4: a second `---` line (the first is on line 2); a service has exactly one"
        );
    }

    #[test]
    fn a_bad_request_field_names_its_line_in_the_srv_file() {
        let e = parse_rossrv("# head\nbool\n---\n", "S", None, FILE).unwrap_err();
        assert_eq!(
            e,
            SrvParseError {
                file: FILE.into(),
                line: 2,
                kind: SrvErrorKind::Field {
                    message: "expected 'type name', got: bool".into()
                },
            }
        );
        assert_eq!(
            e.to_string(),
            "pkg/Srv.srv:2: expected 'type name', got: bool"
        );
    }

    #[test]
    fn a_bad_response_field_names_its_line_in_the_srv_file_not_in_the_part() {
        let e = parse_rossrv("bool a\n---\n\nint32[x] b\n", "S", None, FILE).unwrap_err();
        assert_eq!(
            e,
            SrvParseError {
                file: FILE.into(),
                line: 4,
                kind: SrvErrorKind::Field {
                    message: "invalid type: invalid array length: x".into()
                },
            }
        );
    }

    #[test]
    fn a_bad_default_in_the_response_names_its_line() {
        let e = parse_rossrv("---\nbool ok maybe\n", "S", None, FILE).unwrap_err();
        assert_eq!(e.line, 2);
        assert_eq!(
            e.kind,
            SrvErrorKind::Field {
                message: "invalid bool default: maybe".into()
            }
        );
    }

    #[test]
    fn two_parses_of_the_same_text_are_identical() {
        let text =
            "uint64 DEPTH_RECURSIVE=0\nstring[] prefixes\nuint64 depth\n---\np/Result result\n";
        let a = parse_rossrv(text, "List", Some("p"), FILE).unwrap();
        let b = parse_rossrv(text, "List", Some("p"), FILE).unwrap();
        assert_eq!(format!("{a:?}"), format!("{b:?}"));
        assert_eq!(a.request.schema_hash(), b.request.schema_hash());
        assert_eq!(a.response.schema_hash(), b.response.schema_hash());
    }
}

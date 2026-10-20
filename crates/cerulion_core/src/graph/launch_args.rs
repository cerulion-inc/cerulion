// SPDX-License-Identifier: AGPL-3.0-only
//! Launch arguments: values a graph declares under a top-level `args:` map
//! and an operator overrides at launch with `name:=value`, so one graph file
//! serves many setups.
//!
//! An argument has a name, a default and an optional description. Precedence
//! is the declared default, then the command-line override. The resolved
//! values substitute into four places and nowhere else: the graph `prefix`,
//! an output's `topic`, an input's `source` and a node's `params:` values.
//! A reference is `${name}`; `$$` is a literal `$`. A `params:` value that
//! is exactly one reference takes the argument's typed value (a `bool`, a
//! number or text, whatever the default or the override parsed as); text
//! around a reference makes the result text.
//!
//! Resolution is one pure function from (graph text, overrides) to a
//! resolved graph: the `args:` section is lifted out of the document and
//! every reference is replaced before the graph types read it, so what gets
//! checked, run and recorded as a bag's `graph.yaml` is a plain graph with no
//! arguments left to resolve. Everything here is cold: it runs once per
//! graph load, before any transport resource exists.
//!
//! Every problem is a load error naming the argument and the YAML location:
//! an override for an argument the graph does not declare (with the nearest
//! declared name), a reference to an undeclared argument, an unterminated or
//! malformed reference, a reference in a key that takes none, and a value
//! that itself contains a reference (values are literal; nothing is
//! substituted twice).

use std::fmt;

use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use serde_yaml::Value;

use super::config::GraphConfig;
use super::params::ParamValue;
use crate::error::{TransportError, TransportResult};

/// The YAML key the arguments are declared under.
pub const ARGS_KEY: &str = "args";

/// The long form of one declaration: `args: {use_sim: {default: false,
/// description: "..."}}`. Denies unknown fields, so a misspelled `defualt:`
/// is a parse error naming the key, like every other graph YAML struct.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArgSpec {
    pub default: ParamValue,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// One declared argument: the short form `name: default` or the long form
/// with a description. Read by hand from the YAML value rather than through
/// an untagged `Deserialize`, so a mistake inside the long form keeps the
/// struct's own unknown-field error instead of "did not match any variant".
#[derive(Debug, Clone, PartialEq)]
pub enum ArgDecl {
    Value(ParamValue),
    Spec(ArgSpec),
}

impl ArgDecl {
    /// The default the graph declares for this argument.
    pub fn default_value(&self) -> &ParamValue {
        match self {
            Self::Value(v) => v,
            Self::Spec(spec) => &spec.default,
        }
    }

    /// The description, when the long form supplied one.
    pub fn description(&self) -> Option<&str> {
        match self {
            Self::Value(_) => None,
            Self::Spec(spec) => spec.description.as_deref(),
        }
    }
}

/// The declared arguments in declaration order.
pub type ArgDecls = IndexMap<String, ArgDecl>;

/// The resolved value of every declared argument, in declaration order.
pub type ResolvedArgs = IndexMap<String, ParamValue>;

/// One `name:=value` from the command line, with the value parsed as a YAML
/// scalar: `use_sim:=true` is a bool, `rate:=50` an integer, `name:=go2`
/// text, and `id:="'123'"` keeps the digits as text.
#[derive(Debug, Clone, PartialEq)]
pub struct LaunchOverride {
    pub name: String,
    pub value: ParamValue,
}

/// The separator between the name and the value of an override.
pub const OVERRIDE_SEPARATOR: &str = ":=";

/// Why a graph's launch arguments were refused. Rendered through
/// [`fmt::Display`] into the load error the user sees; every arm names the
/// argument or the location, and the fix.
#[derive(Debug, Clone, PartialEq)]
pub enum ArgProblem {
    /// A `name:=value` without the separator, with an empty or invalid
    /// name, or with a value that is not a scalar.
    MalformedOverride { text: String, reason: String },
    /// `name:=value` for an argument the graph does not declare.
    UnknownOverride {
        arg: String,
        declared: Vec<String>,
        nearest: Option<String>,
    },
    /// The same argument overridden twice on one command line.
    DuplicateOverride { arg: String },
    /// A declared name that is not an identifier.
    InvalidArgName { name: String },
    /// `args:` is not a map, or one declaration is neither a scalar nor a
    /// `default:` map.
    MalformedDeclaration { arg: Option<String>, reason: String },
    /// A default or an override whose text contains `${`: values are
    /// literal and are never substituted into each other.
    ValueContainsReference { arg: String, origin: &'static str },
    /// `${name}` where `name` is not declared.
    UndeclaredReference {
        arg: String,
        location: String,
        declared: Vec<String>,
        nearest: Option<String>,
    },
    /// `${` with no closing `}`.
    UnterminatedReference { location: String, text: String },
    /// `${}` or `${not an identifier}`.
    MalformedReference { location: String, reference: String },
    /// A `${` in a key that takes no substitution.
    ReferenceOutsideSupportedKey { location: String },
}

impl fmt::Display for ArgProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedOverride { text, reason } => write!(
                f,
                "launch argument override '{text}' is malformed: {reason}; write `name:=value`, \
                 for example `use_sim:=true`"
            ),
            Self::UnknownOverride {
                arg,
                declared,
                nearest,
            } => {
                write!(f, "launch argument '{arg}' is not declared by this graph")?;
                if declared.is_empty() {
                    return write!(
                        f,
                        "; this graph declares no `args:`, so `name:=value` overrides are not \
                         accepted on it"
                    );
                }
                if let Some(nearest) = nearest {
                    write!(f, " (did you mean '{nearest}'?)")?;
                }
                write!(f, "; declared args: {}", declared.join(", "))
            }
            Self::DuplicateOverride { arg } => write!(
                f,
                "launch argument '{arg}' is given more than once on the command line; pass each \
                 override once"
            ),
            Self::InvalidArgName { name } => write!(
                f,
                "launch argument name '{name}' is not accepted: a name is letters, digits and \
                 underscores and starts with a letter or an underscore"
            ),
            Self::MalformedDeclaration { arg, reason } => match arg {
                Some(arg) => write!(
                    f,
                    "launch argument '{arg}' is declared wrongly: {reason}; declare a scalar \
                     default (`{arg}: value`) or a map with `default` and an optional \
                     `description`"
                ),
                None => write!(
                    f,
                    "`args:` is declared wrongly: {reason}; declare a map of argument name to \
                     default, for example `args: {{use_sim: false}}`"
                ),
            },
            Self::ValueContainsReference { arg, origin } => write!(
                f,
                "launch argument '{arg}' {origin} contains '${{', but argument values are \
                 literal and are not substituted into each other"
            ),
            Self::UndeclaredReference {
                arg,
                location,
                declared,
                nearest,
            } => {
                write!(
                    f,
                    "{location} references launch argument '{arg}', which this graph does not \
                     declare"
                )?;
                if declared.is_empty() {
                    return write!(
                        f,
                        "; this graph declares no `args:`: add one with a default for '{arg}'"
                    );
                }
                if let Some(nearest) = nearest {
                    write!(f, " (did you mean '{nearest}'?)")?;
                }
                write!(f, "; declared args: {}", declared.join(", "))
            }
            Self::UnterminatedReference { location, text } => write!(
                f,
                "{location} has an unterminated substitution in {text:?}: '${{' needs a closing \
                 '}}'; write '$$' for a literal '$'"
            ),
            Self::MalformedReference {
                location,
                reference,
            } => write!(
                f,
                "{location} has a malformed substitution {reference:?}: write '${{name}}' where \
                 the name is letters, digits and underscores and starts with a letter or an \
                 underscore"
            ),
            Self::ReferenceOutsideSupportedKey { location } => write!(
                f,
                "{location} contains a substitution, but launch arguments substitute only into \
                 `prefix`, an output's `topic`, an input's `source` and `params:` values"
            ),
        }
    }
}

impl From<ArgProblem> for TransportError {
    fn from(problem: ArgProblem) -> Self {
        TransportError::GraphError {
            reason: problem.to_string(),
        }
    }
}

/// `[A-Za-z_][A-Za-z0-9_]*`.
fn is_valid_arg_name(name: &str) -> bool {
    let mut chars = name.chars();
    match chars.next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' => {}
        _ => return false,
    }
    chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// The declared name closest to `unknown`, when any is declared.
fn nearest_declared(unknown: &str, declared: &[String]) -> Option<String> {
    let legal: Vec<&str> = declared.iter().map(String::as_str).collect();
    (!legal.is_empty()).then(|| super::node::nearest_legal_key(unknown, &legal).to_string())
}

/// Parse one `name:=value` override. The value text is read as a YAML
/// scalar; an empty value is the empty string.
pub fn parse_override(text: &str) -> Result<LaunchOverride, ArgProblem> {
    let malformed = |reason: String| ArgProblem::MalformedOverride {
        text: text.to_string(),
        reason,
    };
    let Some((name, value_text)) = text.split_once(OVERRIDE_SEPARATOR) else {
        return Err(malformed(format!(
            "expected `name{OVERRIDE_SEPARATOR}value`, found no `{OVERRIDE_SEPARATOR}`"
        )));
    };
    if name.is_empty() {
        return Err(malformed("the argument name is empty".to_string()));
    }
    if !is_valid_arg_name(name) {
        return Err(malformed(format!(
            "'{name}' is not an argument name (letters, digits and underscores, starting with \
             a letter or an underscore)"
        )));
    }
    let value = parse_scalar(value_text).ok_or_else(|| {
        malformed(format!(
            "the value {value_text:?} is not a scalar; a value is a bool, a number or text"
        ))
    })?;
    Ok(LaunchOverride {
        name: name.to_string(),
        value,
    })
}

/// Read a value's text as the YAML scalar an author would get from writing
/// it in the graph file: `true`, `50`, `2.5` and `go2` take their types,
/// quoted text keeps digits as text. `None` for a map, a list or null.
fn parse_scalar(text: &str) -> Option<ParamValue> {
    if text.trim().is_empty() {
        return Some(ParamValue::Str(String::new()));
    }
    serde_yaml::from_str::<ParamValue>(text).ok()
}

/// Resolve the declared arguments against the overrides: every declared
/// argument takes its default, then each override replaces one. Pure.
pub fn resolve_args(
    declared: &ArgDecls,
    overrides: &[LaunchOverride],
) -> Result<ResolvedArgs, ArgProblem> {
    let names: Vec<String> = declared.keys().cloned().collect();
    let mut resolved: ResolvedArgs = IndexMap::with_capacity(declared.len());
    for (name, decl) in declared {
        let default = decl.default_value();
        if value_has_reference(default) {
            return Err(ArgProblem::ValueContainsReference {
                arg: name.clone(),
                origin: "default",
            });
        }
        resolved.insert(name.clone(), default.clone());
    }
    let mut seen: Vec<&str> = Vec::with_capacity(overrides.len());
    for override_ in overrides {
        if !declared.contains_key(&override_.name) {
            return Err(ArgProblem::UnknownOverride {
                nearest: nearest_declared(&override_.name, &names),
                arg: override_.name.clone(),
                declared: names,
            });
        }
        if seen.contains(&override_.name.as_str()) {
            return Err(ArgProblem::DuplicateOverride {
                arg: override_.name.clone(),
            });
        }
        if value_has_reference(&override_.value) {
            return Err(ArgProblem::ValueContainsReference {
                arg: override_.name.clone(),
                origin: "override",
            });
        }
        seen.push(&override_.name);
        resolved.insert(override_.name.clone(), override_.value.clone());
    }
    Ok(resolved)
}

fn value_has_reference(value: &ParamValue) -> bool {
    matches!(value, ParamValue::Str(s) if s.contains("${"))
}

/// A value as it reads inside a topic name or a prefix: text as written, a
/// float with its decimal point, a bool as `true`/`false`.
fn render_in_text(value: &ParamValue) -> String {
    match value {
        ParamValue::Str(s) => s.clone(),
        ParamValue::Bool(b) => b.to_string(),
        ParamValue::Int(i) => i.to_string(),
        ParamValue::UInt(u) => u.to_string(),
        ParamValue::Float(x) => format!("{x:?}"),
    }
}

/// What one scan of a text found.
enum Scanned {
    /// The text had no reference and is returned unchanged.
    Literal,
    /// Exactly one reference and nothing else: the argument's typed value.
    Whole(ParamValue),
    /// Text with references replaced.
    Text(String),
}

/// Replace every `${name}` in `text` with the argument's value and every
/// `$$` with `$`; any other `$` stays. `location` names the YAML key for the
/// error. Pure.
fn scan(text: &str, args: &ResolvedArgs, location: &str) -> Result<Scanned, ArgProblem> {
    if !text.contains('$') {
        return Ok(Scanned::Literal);
    }
    let mut out = String::with_capacity(text.len());
    let mut whole: Option<&ParamValue> = None;
    let mut pieces = 0usize;
    let mut rest = text;
    while let Some(dollar) = rest.find('$') {
        if dollar > 0 {
            out.push_str(&rest[..dollar]);
            pieces += 1;
        }
        let after = &rest[dollar + 1..];
        if let Some(tail) = after.strip_prefix('$') {
            out.push('$');
            pieces += 1;
            rest = tail;
            continue;
        }
        let Some(body) = after.strip_prefix('{') else {
            out.push('$');
            pieces += 1;
            rest = after;
            continue;
        };
        let Some(close) = body.find('}') else {
            return Err(ArgProblem::UnterminatedReference {
                location: location.to_string(),
                text: text.to_string(),
            });
        };
        let name = &body[..close];
        if !is_valid_arg_name(name) {
            return Err(ArgProblem::MalformedReference {
                location: location.to_string(),
                reference: format!("${{{name}}}"),
            });
        }
        let Some(value) = args.get(name) else {
            let declared: Vec<String> = args.keys().cloned().collect();
            return Err(ArgProblem::UndeclaredReference {
                arg: name.to_string(),
                location: location.to_string(),
                nearest: nearest_declared(name, &declared),
                declared,
            });
        };
        out.push_str(&render_in_text(value));
        pieces += 1;
        whole = Some(value);
        rest = &body[close + 1..];
    }
    if !rest.is_empty() {
        out.push_str(rest);
        pieces += 1;
    }
    match whole {
        Some(value) if pieces == 1 => Ok(Scanned::Whole(value.clone())),
        _ => Ok(Scanned::Text(out)),
    }
}

/// Substitute the resolved arguments into one text value (a prefix, a topic,
/// a source). Pure: the same inputs give the same text.
pub fn substitute_text(
    text: &str,
    args: &ResolvedArgs,
    location: &str,
) -> Result<String, ArgProblem> {
    Ok(match scan(text, args, location)? {
        Scanned::Literal => text.to_string(),
        Scanned::Whole(value) => render_in_text(&value),
        Scanned::Text(out) => out,
    })
}

/// Lift the `args:` section out of a parsed graph document. Returns the
/// declarations (empty when the document has none) and the document without
/// the key. A document that is not a map is returned as is: the graph types
/// refuse it with their own message.
fn split_args(document: Value) -> Result<(ArgDecls, Value), ArgProblem> {
    let Value::Mapping(mut map) = document else {
        return Ok((ArgDecls::new(), document));
    };
    let Some(section) = map.remove(ARGS_KEY) else {
        return Ok((ArgDecls::new(), Value::Mapping(map)));
    };
    let decls = parse_decls(section)?;
    Ok((decls, Value::Mapping(map)))
}

fn parse_decls(section: Value) -> Result<ArgDecls, ArgProblem> {
    let entries = match section {
        Value::Mapping(entries) => entries,
        Value::Null => {
            return Err(ArgProblem::MalformedDeclaration {
                arg: None,
                reason: "the section is empty".to_string(),
            })
        }
        other => {
            return Err(ArgProblem::MalformedDeclaration {
                arg: None,
                reason: format!("expected a map, found {}", kind_of(&other)),
            })
        }
    };
    let mut decls = ArgDecls::with_capacity(entries.len());
    for (key, value) in entries {
        let name = match key {
            Value::String(name) if is_valid_arg_name(&name) => name,
            other => {
                return Err(ArgProblem::InvalidArgName {
                    name: scalar_text(&other),
                })
            }
        };
        let decl = match value {
            Value::Mapping(_) => {
                let spec: ArgSpec = serde_yaml::from_value(value).map_err(|e| {
                    ArgProblem::MalformedDeclaration {
                        arg: Some(name.clone()),
                        reason: e.to_string(),
                    }
                })?;
                ArgDecl::Spec(spec)
            }
            Value::Null => {
                return Err(ArgProblem::MalformedDeclaration {
                    arg: Some(name),
                    reason: "it has no default".to_string(),
                })
            }
            scalar => {
                let kind = kind_of(&scalar);
                let default = serde_yaml::from_value::<ParamValue>(scalar).map_err(|_| {
                    ArgProblem::MalformedDeclaration {
                        arg: Some(name.clone()),
                        reason: format!("expected a scalar default, found {kind}"),
                    }
                })?;
                ArgDecl::Value(default)
            }
        };
        decls.insert(name, decl);
    }
    Ok(decls)
}

fn kind_of(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "a bool",
        Value::Number(_) => "a number",
        Value::String(_) => "text",
        Value::Sequence(_) => "a list",
        Value::Mapping(_) => "a map",
        Value::Tagged(_) => "a tagged value",
    }
}

fn scalar_text(value: &Value) -> String {
    match value {
        Value::String(s) => s.clone(),
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => n.to_string(),
        other => kind_of(other).to_string(),
    }
}

/// The YAML location of one node entry for error text: its `id` when it has
/// one, else its index.
fn node_label(node: &Value, index: usize) -> String {
    match node.get("id") {
        Some(Value::String(id)) => format!("node '{id}'"),
        _ => format!("nodes[{index}]"),
    }
}

/// Substitute into one text field of a map, when present and text.
fn substitute_field(
    map: &mut serde_yaml::Mapping,
    key: &str,
    args: &ResolvedArgs,
    location: &str,
) -> Result<(), ArgProblem> {
    if let Some(Value::String(text)) = map.get(key) {
        let resolved = substitute_text(text, args, location)?;
        map.insert(Value::String(key.to_string()), Value::String(resolved));
    }
    Ok(())
}

/// Refuse any `${` left in a value outside the four supported keys.
fn refuse_stray_references(value: &Value, location: &str) -> Result<(), ArgProblem> {
    match value {
        Value::String(text) if text.contains("${") => {
            Err(ArgProblem::ReferenceOutsideSupportedKey {
                location: location.to_string(),
            })
        }
        Value::Sequence(items) => items
            .iter()
            .enumerate()
            .try_for_each(|(i, item)| refuse_stray_references(item, &format!("{location}[{i}]"))),
        Value::Mapping(map) => map.iter().try_for_each(|(key, item)| {
            refuse_stray_references(item, &format!("{location}.{}", scalar_text(key)))
        }),
        _ => Ok(()),
    }
}

/// Substitute the resolved arguments into a graph document (with `args:`
/// already lifted out): `prefix`, every output `topic`, every input `source`
/// and every `params:` value. A reference anywhere else is refused.
fn substitute_document(document: Value, args: &ResolvedArgs) -> Result<Value, ArgProblem> {
    let Value::Mapping(mut map) = document else {
        return Ok(document);
    };
    substitute_field(&mut map, "prefix", args, "`prefix`")?;
    if let Some(Value::Sequence(nodes)) = map.get_mut("nodes") {
        for (index, node) in nodes.iter_mut().enumerate() {
            let label = node_label(node, index);
            let Value::Mapping(node_map) = node else {
                continue;
            };
            if let Some(Value::Sequence(outputs)) = node_map.get_mut("outputs") {
                for output in outputs.iter_mut() {
                    if let Value::Mapping(output_map) = output {
                        let name = output_map.get("name").map(scalar_text).unwrap_or_default();
                        let location = format!("{label} output '{name}' `topic`");
                        substitute_field(output_map, "topic", args, &location)?;
                    }
                }
            }
            if let Some(Value::Sequence(inputs)) = node_map.get_mut("inputs") {
                for input in inputs.iter_mut() {
                    if let Value::Mapping(input_map) = input {
                        let name = input_map.get("name").map(scalar_text).unwrap_or_default();
                        let location = format!("{label} input '{name}' `source`");
                        substitute_field(input_map, "source", args, &location)?;
                    }
                }
            }
            if let Some(Value::Mapping(params)) = node_map.get_mut("params") {
                for (key, value) in params.iter_mut() {
                    let Value::String(text) = value else {
                        continue;
                    };
                    let location = format!("{label} param '{}'", scalar_text(key));
                    *value = match scan(text, args, &location)? {
                        Scanned::Literal => continue,
                        Scanned::Whole(typed) => serde_yaml::to_value(&typed).map_err(|e| {
                            ArgProblem::MalformedDeclaration {
                                arg: None,
                                reason: e.to_string(),
                            }
                        })?,
                        Scanned::Text(out) => Value::String(out),
                    };
                }
            }
            // The keys above have been substituted; anything still carrying a
            // reference is in a key that takes none.
            for (key, value) in node_map.iter() {
                let key_text = scalar_text(key);
                match key_text.as_str() {
                    "outputs" | "inputs" | "params" => {
                        refuse_only_other_keys(value, &key_text, &label)?
                    }
                    _ => refuse_stray_references(value, &format!("{label} `{key_text}`"))?,
                }
            }
        }
    }
    for (key, value) in map.iter() {
        let key_text = scalar_text(key);
        if key_text != "nodes" && key_text != "prefix" {
            refuse_stray_references(value, &format!("`{key_text}`"))?;
        }
    }
    Ok(Value::Mapping(map))
}

/// Inside `outputs`, `inputs` and `params`, every key but the substituted
/// one (`topic`, `source`, the values) must carry no reference.
fn refuse_only_other_keys(value: &Value, section: &str, label: &str) -> Result<(), ArgProblem> {
    match (section, value) {
        ("params", _) => Ok(()),
        (_, Value::Sequence(items)) => {
            let substituted = if section == "outputs" {
                "topic"
            } else {
                "source"
            };
            for (i, item) in items.iter().enumerate() {
                if let Value::Mapping(entry) = item {
                    for (key, field) in entry.iter() {
                        let key_text = scalar_text(key);
                        if key_text != substituted {
                            refuse_stray_references(
                                field,
                                &format!("{label} {section}[{i}] `{key_text}`"),
                            )?;
                        }
                    }
                }
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

fn parse_failure(e: impl fmt::Display) -> TransportError {
    TransportError::GraphParseError {
        reason: e.to_string(),
    }
}

fn document_has_dollar(value: &Value) -> bool {
    match value {
        Value::String(text) => text.contains('$'),
        Value::Sequence(items) => items.iter().any(document_has_dollar),
        Value::Mapping(map) => map.values().any(document_has_dollar),
        _ => false,
    }
}

/// Parse graph YAML with its launch arguments resolved: the `args:` section
/// is lifted out, defaults then `overrides` give every argument its value,
/// and every reference is replaced before the graph types read the
/// document. A document with no `args:`, no `$` and no overrides parses
/// exactly as it always did, error text included.
///
/// Pure: the same text and overrides give the same graph, in the same
/// order, twice. No prefix default is filled in and no identity is adopted;
/// the callers that do that do it after this.
pub fn parse_graph_with_launch_args(
    yaml: &str,
    overrides: &[LaunchOverride],
) -> TransportResult<GraphConfig> {
    let document: Value = serde_yaml::from_str(yaml).map_err(parse_failure)?;
    let declares_args = matches!(&document, Value::Mapping(map) if map.contains_key(ARGS_KEY));
    if !declares_args && overrides.is_empty() && !document_has_dollar(&document) {
        return serde_yaml::from_str(yaml).map_err(parse_failure);
    }
    let (declared, body) = split_args(document)?;
    let resolved = resolve_args(&declared, overrides)?;
    let body = substitute_document(body, &resolved)?;
    serde_yaml::from_value(body).map_err(parse_failure)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROBOT: &str = "args:\n  ns: lab\n  side: left\n  gain: 2.5\n  use_sim:\n    default: false\n    description: drive the simulator instead of the robot\nprefix: ${ns}\nnodes:\n  - id: cam\n    type: cam\n    params:\n      gain: ${gain}\n      frame_id: ${side}_cam\n      enabled: ${use_sim}\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n        topic: /${ns}/${side}/cmd\n  - id: sink\n    type: sink\n    inputs:\n      - name: cmd\n        source: /${ns}/${side}/cmd\n";

    fn overrides(texts: &[&str]) -> Vec<LaunchOverride> {
        texts
            .iter()
            .map(|t| parse_override(t).expect("override parses"))
            .collect()
    }

    #[test]
    fn declared_defaults_and_overrides_substitute_into_the_four_sites_with_types() {
        let graph = parse_graph_with_launch_args(ROBOT, &overrides(&["side:=right", "gain:=0.5"]))
            .expect("the graph resolves");
        assert_eq!(graph.prefix, "lab");
        let cam = &graph.nodes[0];
        assert_eq!(cam.outputs[0].topic.as_deref(), Some("/lab/right/cmd"));
        assert_eq!(cam.params["gain"], ParamValue::Float(0.5));
        assert_eq!(
            cam.params["frame_id"],
            ParamValue::Str("right_cam".to_string())
        );
        assert_eq!(cam.params["enabled"], ParamValue::Bool(false));
        assert_eq!(graph.nodes[1].inputs[0].source, "/lab/right/cmd");
    }

    #[test]
    fn with_no_overrides_every_argument_takes_its_declared_default() {
        let graph = parse_graph_with_launch_args(ROBOT, &[]).expect("the graph resolves");
        assert_eq!(graph.prefix, "lab");
        let cam = &graph.nodes[0];
        assert_eq!(cam.outputs[0].topic.as_deref(), Some("/lab/left/cmd"));
        assert_eq!(cam.params["gain"], ParamValue::Float(2.5));
        assert_eq!(
            cam.params["frame_id"],
            ParamValue::Str("left_cam".to_string())
        );
        assert_eq!(graph.nodes[1].inputs[0].source, "/lab/left/cmd");
    }

    #[test]
    fn a_graph_without_args_or_dollar_signs_parses_as_before() {
        let plain = "prefix: lab\nnodes:\n  - id: cam\n    type: cam\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n";
        let graph = parse_graph_with_launch_args(plain, &[]).expect("parses");
        assert_eq!(graph.prefix, "lab");
        assert_eq!(graph.nodes[0].outputs[0].topic, None);
        // The untouched path keeps serde's own unknown-key message, line and
        // column included.
        let err = parse_graph_with_launch_args(&plain.replace("schema:", "shcema:"), &[])
            .expect_err("a misspelled key is refused");
        assert!(err.to_string().contains("`shcema`") && err.to_string().contains("line"));
    }

    #[test]
    fn resolve_args_applies_defaults_then_overrides_in_declaration_order() {
        let declared: ArgDecls = [
            ("a".to_string(), ArgDecl::Value(ParamValue::Int(1))),
            (
                "b".to_string(),
                ArgDecl::Spec(ArgSpec {
                    default: ParamValue::Str("x".to_string()),
                    description: Some("second".to_string()),
                }),
            ),
            ("c".to_string(), ArgDecl::Value(ParamValue::Bool(true))),
        ]
        .into_iter()
        .collect();
        let resolved = resolve_args(&declared, &overrides(&["c:=false", "a:=7"])).unwrap();
        let listed: Vec<(&str, &ParamValue)> =
            resolved.iter().map(|(k, v)| (k.as_str(), v)).collect();
        assert_eq!(
            listed,
            [
                ("a", &ParamValue::Int(7)),
                ("b", &ParamValue::Str("x".to_string())),
                ("c", &ParamValue::Bool(false)),
            ]
        );
        assert_eq!(declared["b"].description(), Some("second"));
        assert_eq!(declared["a"].description(), None);
    }

    #[test]
    fn overrides_parse_as_yaml_scalars_and_an_empty_value_is_empty_text() {
        let cases = [
            ("use_sim:=true", "use_sim", ParamValue::Bool(true)),
            ("rate:=50", "rate", ParamValue::Int(50)),
            ("offset:=-5", "offset", ParamValue::Int(-5)),
            ("gain:=2.5", "gain", ParamValue::Float(2.5)),
            ("name:=go2", "name", ParamValue::Str("go2".to_string())),
            ("id:='123'", "id", ParamValue::Str("123".to_string())),
            ("note:=a:=b", "note", ParamValue::Str("a:=b".to_string())),
            ("empty:=", "empty", ParamValue::Str(String::new())),
            ("_u1:=v", "_u1", ParamValue::Str("v".to_string())),
        ];
        for (text, name, value) in cases {
            let o = parse_override(text).unwrap_or_else(|e| panic!("{text}: {e}"));
            assert_eq!(o.name, name, "{text}");
            assert_eq!(o.value, value, "{text}");
        }
    }

    #[test]
    fn substitution_handles_repeats_escapes_and_bare_dollars() {
        let args: ResolvedArgs = [
            ("ns".to_string(), ParamValue::Str("lab".to_string())),
            ("n".to_string(), ParamValue::Int(3)),
            ("f".to_string(), ParamValue::Float(2.0)),
            ("b".to_string(), ParamValue::Bool(true)),
            ("empty".to_string(), ParamValue::Str(String::new())),
        ]
        .into_iter()
        .collect();
        let sub = |t: &str| substitute_text(t, &args, "`prefix`").unwrap();
        assert_eq!(sub("/${ns}/${ns}/${ns}"), "/lab/lab/lab");
        assert_eq!(sub("${n}-${f}-${b}"), "3-2.0-true");
        assert_eq!(sub("a${empty}b"), "ab");
        assert_eq!(sub("cost $$5 and $${ns}"), "cost $5 and ${ns}");
        assert_eq!(sub("a $ b"), "a $ b");
        assert_eq!(sub("trailing $"), "trailing $");
        assert_eq!(sub("no reference"), "no reference");
        assert_eq!(sub(""), "");
    }

    #[test]
    fn a_param_value_that_is_exactly_one_reference_takes_the_typed_value() {
        let yaml = "args:\n  n: 3\n  s: \"7\"\n  pct: 50\nnodes:\n  - id: a\n    type: a\n    params:\n      count: ${n}\n      label: ${s}\n      note: ${pct}%\n      quoted: \"${n}\"\n      plain: literal\n";
        let graph = parse_graph_with_launch_args(yaml, &[]).unwrap();
        let p = &graph.nodes[0].params;
        assert_eq!(p["count"], ParamValue::Int(3));
        assert_eq!(p["label"], ParamValue::Str("7".to_string()));
        assert_eq!(p["note"], ParamValue::Str("50%".to_string()));
        // YAML quoting does not survive into the document, so a quoted
        // reference is still exactly one reference: the argument's type wins.
        assert_eq!(p["quoted"], ParamValue::Int(3));
        assert_eq!(p["plain"], ParamValue::Str("literal".to_string()));
    }

    #[test]
    fn resolution_is_deterministic_across_two_parses() {
        let a = parse_graph_with_launch_args(ROBOT, &overrides(&["side:=right"])).unwrap();
        let b = parse_graph_with_launch_args(ROBOT, &overrides(&["side:=right"])).unwrap();
        assert_eq!(
            serde_yaml::to_string(&a).unwrap(),
            serde_yaml::to_string(&b).unwrap()
        );
    }

    #[test]
    fn a_huge_document_with_many_references_resolves_without_quadratic_blowup() {
        let mut yaml = String::from("args:\n  ns: lab\nprefix: ${ns}\nnodes:\n");
        for i in 0..2_000 {
            yaml.push_str(&format!(
                "  - id: n{i}\n    type: t\n    outputs:\n      - name: o\n        schema: s/T\n        topic: /${{ns}}/{i}/${{ns}}\n"
            ));
        }
        let graph = parse_graph_with_launch_args(&yaml, &[]).unwrap();
        assert_eq!(graph.nodes.len(), 2_000);
        assert_eq!(
            graph.nodes[1_999].outputs[0].topic.as_deref(),
            Some("/lab/1999/lab")
        );
        let long = format!("${{ns}}{}", "x".repeat(100_000));
        let args: ResolvedArgs = [("ns".to_string(), ParamValue::Str("lab".to_string()))]
            .into_iter()
            .collect();
        assert_eq!(
            substitute_text(&long, &args, "`prefix`").unwrap().len(),
            100_003
        );
    }

    fn refused(yaml: &str, overrides_: &[&str]) -> String {
        parse_graph_with_launch_args(yaml, &overrides(overrides_))
            .expect_err("the graph must be refused")
            .to_string()
    }

    #[test]
    fn an_override_for_an_undeclared_argument_is_refused_with_the_nearest_name() {
        let msg = refused(ROBOT, &["sdie:=right"]);
        assert!(
            msg.contains(
                "launch argument 'sdie' is not declared by this graph (did you mean 'side'?); \
                 declared args: ns, side, gain, use_sim"
            ),
            "{msg}"
        );
        let plain = "nodes:\n  - id: a\n    type: a\n";
        let msg = refused(plain, &["x:=1"]);
        assert!(
            msg.contains(
                "launch argument 'x' is not declared by this graph; this graph declares no \
                 `args:`, so `name:=value` overrides are not accepted on it"
            ),
            "{msg}"
        );
    }

    #[test]
    fn the_same_override_twice_is_refused() {
        let msg = refused(ROBOT, &["side:=a", "side:=b"]);
        assert!(
            msg.contains("launch argument 'side' is given more than once on the command line"),
            "{msg}"
        );
    }

    #[test]
    fn a_malformed_override_names_the_text_and_the_shape() {
        for (text, fragment) in [
            ("side=right", "found no `:=`"),
            (":=right", "the argument name is empty"),
            ("my-arg:=1", "'my-arg' is not an argument name"),
            ("1st:=1", "'1st' is not an argument name"),
            ("list:=[1, 2]", "is not a scalar"),
            ("map:={a: 1}", "is not a scalar"),
            ("nothing:=null", "is not a scalar"),
        ] {
            let err = parse_override(text).expect_err(text).to_string();
            assert!(
                err.starts_with(&format!("launch argument override '{text}' is malformed: "))
                    && err.contains(fragment)
                    && err.contains("write `name:=value`"),
                "{text}: {err}"
            );
        }
    }

    #[test]
    fn a_reference_to_an_undeclared_argument_names_the_location_and_the_nearest_name() {
        let msg = refused(
            &ROBOT.replace("topic: /${ns}/${side}/cmd", "topic: /${nz}/cmd"),
            &[],
        );
        assert!(
            msg.contains(
                "node 'cam' output 'cmd' `topic` references launch argument 'nz', which this \
                 graph does not declare (did you mean 'ns'?); declared args: ns, side, gain, \
                 use_sim"
            ),
            "{msg}"
        );
        let msg = refused(
            &ROBOT.replace("source: /${ns}/${side}/cmd", "source: ${cmd}"),
            &[],
        );
        assert!(
            msg.contains("node 'sink' input 'cmd' `source` references launch argument 'cmd'"),
            "{msg}"
        );
        let msg = refused(&ROBOT.replace("gain: ${gain}", "gain: ${gian}"), &[]);
        assert!(msg.contains("node 'cam' param 'gain' references launch argument 'gian', which this graph does not declare (did you mean 'gain'?)"), "{msg}");
        let msg = refused("prefix: ${ns}\nnodes:\n  - id: a\n    type: a\n", &[]);
        assert!(
            msg.contains(
                "`prefix` references launch argument 'ns', which this graph does not declare; \
                 this graph declares no `args:`: add one with a default for 'ns'"
            ),
            "{msg}"
        );
    }

    #[test]
    fn an_unterminated_or_malformed_reference_is_refused_at_its_location() {
        let msg = refused(&ROBOT.replace("prefix: ${ns}", "prefix: ${ns"), &[]);
        assert!(
            msg.contains(
                "`prefix` has an unterminated substitution in \"${ns\": '${' needs a closing '}'"
            ),
            "{msg}"
        );
        let msg = refused(&ROBOT.replace("prefix: ${ns}", "prefix: ${}"), &[]);
        assert!(
            msg.contains("`prefix` has a malformed substitution \"${}\""),
            "{msg}"
        );
        let msg = refused(&ROBOT.replace("prefix: ${ns}", "prefix: \"${n s}\""), &[]);
        assert!(
            msg.contains("`prefix` has a malformed substitution \"${n s}\""),
            "{msg}"
        );
        let msg = refused(&ROBOT.replace("prefix: ${ns}", "prefix: \"${${ns}}\""), &[]);
        assert!(
            msg.contains("`prefix` has a malformed substitution \"${${ns}\""),
            "{msg}"
        );
    }

    #[test]
    fn a_value_carrying_a_reference_is_refused_instead_of_resolved_twice() {
        let msg = refused(&ROBOT.replace("ns: lab", "ns: ${side}"), &[]);
        assert!(
            msg.contains(
                "launch argument 'ns' default contains '${', but argument values are literal and \
                 are not substituted into each other"
            ),
            "{msg}"
        );
        let msg = refused(ROBOT, &["ns:=${side}"]);
        assert!(
            msg.contains("launch argument 'ns' override contains '${'"),
            "{msg}"
        );
    }

    #[test]
    fn a_reference_outside_the_four_supported_keys_is_refused_by_location() {
        for (yaml, location) in [
            (
                ROBOT.replace("id: cam", "id: ${side}"),
                "node '${side}' `id`",
            ),
            (
                ROBOT.replace("type: cam", "type: ${side}"),
                "node 'cam' `type`",
            ),
            (
                ROBOT.replace("schema: geometry_msgs/Vector3", "schema: ${side}"),
                "node 'cam' outputs[0] `schema`",
            ),
            (
                ROBOT.replace(
                    "      - name: cmd\n        source",
                    "      - name: ${side}\n        source",
                ),
                "node 'sink' inputs[0] `name`",
            ),
            (
                format!("{ROBOT}multi_publisher_topics:\n  - /${{ns}}/tf\n"),
                "`multi_publisher_topics`[0]",
            ),
        ] {
            let msg = refused(&yaml, &[]);
            assert!(
                msg.contains(&format!(
                    "{location} contains a substitution, but launch arguments substitute only \
                     into `prefix`, an output's `topic`, an input's `source` and `params:` \
                     values"
                )),
                "{location}: {msg}"
            );
        }
    }

    #[test]
    fn a_malformed_args_section_or_declaration_is_refused_by_name() {
        let body = "nodes:\n  - id: a\n    type: a\n";
        for (section, fragment) in [
            (
                "args: [a, b]\n",
                "`args:` is declared wrongly: expected a map, found a list",
            ),
            (
                "args:\n",
                "`args:` is declared wrongly: the section is empty",
            ),
            (
                "args:\n  my-arg: 1\n",
                "launch argument name 'my-arg' is not accepted",
            ),
            (
                "args:\n  9x: 1\n",
                "launch argument name '9x' is not accepted",
            ),
            (
                "args:\n  a:\n",
                "launch argument 'a' is declared wrongly: it has no default",
            ),
            (
                "args:\n  a: [1, 2]\n",
                "launch argument 'a' is declared wrongly: expected a scalar default, found a list",
            ),
            (
                "args:\n  a:\n    description: no default\n",
                "launch argument 'a' is declared wrongly: missing field `default`",
            ),
            (
                "args:\n  a:\n    default: 1\n    extra: 2\n",
                "launch argument 'a' is declared wrongly: unknown field `extra`",
            ),
        ] {
            let msg = refused(&format!("{section}{body}"), &[]);
            assert!(msg.contains(fragment), "{section:?}: {msg}");
        }
    }

    #[test]
    fn the_resolved_graph_carries_no_args_section_and_no_reference() {
        let graph = parse_graph_with_launch_args(ROBOT, &[]).unwrap();
        let rendered = serde_yaml::to_string(&graph).unwrap();
        assert!(!rendered.contains("args:"), "{rendered}");
        assert!(!rendered.contains("${"), "{rendered}");
        // What a bag records is itself a graph that parses with nothing left
        // to resolve.
        parse_graph_with_launch_args(&rendered, &[]).expect("the resolved graph parses again");
    }
}

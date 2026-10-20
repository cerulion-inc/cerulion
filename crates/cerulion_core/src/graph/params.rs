// SPDX-License-Identifier: AGPL-3.0-only
//! Node parameters: typed knobs a node declares with `#[param]` on a plain
//! struct field and a graph sets per node under `params:`.
//!
//! This is the static tier. A value is resolved ONCE at graph load and is
//! fixed for the run. Precedence is the macro default, then the graph YAML
//! `params:` map. A param only supplies a VALUE for a knob the node declares;
//! it never overrides any other node fact (trigger policy, period, depth,
//! schema). Names are flat: a param is a struct field, so `gain`, never
//! `motor.gain`.
//!
//! Everything in this module is cold. It runs at graph build, before any
//! transport resource exists, and refuses the graph on the first problem.
//! Delivery to the node is one write per field before the node's `init`, so
//! the tick path reads a plain struct field and allocates nothing.
//!
//! Three refusal classes, each naming the node, the param and the fix:
//!
//! - an unknown name, with the nearest declared name when one exists;
//! - a value of the wrong type. There is no coercion: an `f64` param takes
//!   `2.0`, never `2`; an integer must fit the declared width and a float
//!   must fit `f32` on an `f32` param; a `String` param given `off` keeps
//!   the string `off`, and one given an unquoted `123` is told to quote it;
//! - a value outside the declared `min`/`max`. Bounds are inclusive.
//!
//! A float that is not finite is refused as well, bounds or not: a NaN cannot
//! be range-checked, recorded or replayed byte for byte.

use std::fmt;

use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

use crate::error::TransportError;

/// The declared type of a param: the Rust type of the `#[param]` field.
///
/// Serialized under the Rust spelling (`"f64"`, `"String"`) so the node info
/// JSON and the `node info` table read as the field is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ParamType {
    Bool,
    I8,
    I16,
    I32,
    I64,
    U8,
    U16,
    U32,
    U64,
    F32,
    F64,
    #[serde(rename = "String")]
    String,
}

impl ParamType {
    /// The Rust spelling of the type, as a node author writes it.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Bool => "bool",
            Self::I8 => "i8",
            Self::I16 => "i16",
            Self::I32 => "i32",
            Self::I64 => "i64",
            Self::U8 => "u8",
            Self::U16 => "u16",
            Self::U32 => "u32",
            Self::U64 => "u64",
            Self::F32 => "f32",
            Self::F64 => "f64",
            Self::String => "String",
        }
    }

    /// Integers and floats take `min`/`max`; `bool` and `String` do not.
    pub fn is_numeric(self) -> bool {
        !matches!(self, Self::Bool | Self::String)
    }

    /// `f32` or `f64`.
    pub fn is_float(self) -> bool {
        matches!(self, Self::F32 | Self::F64)
    }

    /// Check `value` against this type and return it in canonical form:
    /// signed widths as [`ParamValue::Int`], unsigned widths as
    /// [`ParamValue::UInt`], both float widths as [`ParamValue::Float`].
    /// `None` is a type mismatch: a float for an integer, an integer for a
    /// float, an integer outside the declared width, a float outside `f32`
    /// for an `f32`, a string for a bool.
    ///
    /// An `f32` value stays the `f64` the author wrote: the bounds of an
    /// `f32` param are `f64` literals too, so a value exactly at `max = 0.1`
    /// compares equal to it, and the value is listed, recorded and handed
    /// to a cdylib as `0.1`. The narrowing to `f32` happens once, in the
    /// field write ([`FromParamValue`] for `f32`), on both the in-process
    /// and the cdylib path.
    pub fn canonicalize(self, value: &ParamValue) -> Option<ParamValue> {
        match self {
            Self::Bool => bool::from_param_value(value).map(ParamValue::Bool),
            Self::I8 => i8::from_param_value(value).map(|v| ParamValue::Int(i64::from(v))),
            Self::I16 => i16::from_param_value(value).map(|v| ParamValue::Int(i64::from(v))),
            Self::I32 => i32::from_param_value(value).map(|v| ParamValue::Int(i64::from(v))),
            Self::I64 => i64::from_param_value(value).map(ParamValue::Int),
            Self::U8 => u8::from_param_value(value).map(|v| ParamValue::UInt(u64::from(v))),
            Self::U16 => u16::from_param_value(value).map(|v| ParamValue::UInt(u64::from(v))),
            Self::U32 => u32::from_param_value(value).map(|v| ParamValue::UInt(u64::from(v))),
            Self::U64 => u64::from_param_value(value).map(ParamValue::UInt),
            Self::F32 => match value {
                ParamValue::Float(x) if f32::from_param_value(value).is_some() => {
                    Some(ParamValue::Float(*x))
                }
                _ => None,
            },
            Self::F64 => f64::from_param_value(value).map(ParamValue::Float),
            Self::String => {
                <String as FromParamValue>::from_param_value(value).map(ParamValue::Str)
            }
        }
    }
}

impl fmt::Display for ParamType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A param value as a graph file or a macro default supplies it.
///
/// Untagged on the wire, so graph YAML and the node info JSON both write the
/// scalar itself: `gain: 2.5`, `"default":2.5`. The variants are tried in
/// declaration order, which is what keeps `5` an integer, `2.5` a float and
/// `off` a string.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Bool(bool),
    Int(i64),
    UInt(u64),
    Float(f64),
    Str(String),
}

impl ParamValue {
    /// The kind word used in error text: `bool`, `integer`, `float`, `string`.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Bool(_) => "bool",
            Self::Int(_) | Self::UInt(_) => "integer",
            Self::Float(_) => "float",
            Self::Str(_) => "string",
        }
    }

    fn as_number(&self) -> Option<Number> {
        match self {
            Self::Int(i) => Some(Number::Int(i128::from(*i))),
            Self::UInt(u) => Some(Number::Int(i128::from(*u))),
            Self::Float(f) => Some(Number::Float(*f)),
            Self::Bool(_) | Self::Str(_) => None,
        }
    }
}

impl fmt::Display for ParamValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bool(b) => write!(f, "{b}"),
            Self::Int(i) => write!(f, "{i}"),
            Self::UInt(u) => write!(f, "{u}"),
            // `{:?}` keeps the decimal point (`2.0`, not `2`), so a float
            // reads as a float wherever it is printed.
            Self::Float(x) => write!(f, "{x:?}"),
            Self::Str(s) => write!(f, "{s:?}"),
        }
    }
}

/// A number for range comparison: integers compare exactly in `i128`, floats
/// as `f64`, and a mixed pair compares as `f64`.
#[derive(Debug, Clone, Copy)]
enum Number {
    Int(i128),
    Float(f64),
}

impl Number {
    fn less_than(self, other: Number) -> bool {
        match (self, other) {
            (Self::Int(a), Self::Int(b)) => a < b,
            (Self::Float(a), Self::Float(b)) => a < b,
            (Self::Int(a), Self::Float(b)) => (a as f64) < b,
            (Self::Float(a), Self::Int(b)) => a < (b as f64),
        }
    }
}

/// The Rust types a `#[param]` field may have, and how each one reads a
/// [`ParamValue`]. One impl per type is the single source of truth for "does
/// this value fit this field": the graph build checks with it and the
/// generated node code applies with it.
pub trait FromParamValue: Sized {
    /// The [`ParamType`] this Rust type declares.
    const PARAM_TYPE: ParamType;

    /// `None` when the value is not this type or does not fit it.
    fn from_param_value(value: &ParamValue) -> Option<Self>;
}

macro_rules! integer_params {
    ($($t:ty => $variant:ident),* $(,)?) => {
        $(
            impl FromParamValue for $t {
                const PARAM_TYPE: ParamType = ParamType::$variant;

                fn from_param_value(value: &ParamValue) -> Option<Self> {
                    match value {
                        ParamValue::Int(i) => <$t>::try_from(*i).ok(),
                        ParamValue::UInt(u) => <$t>::try_from(*u).ok(),
                        ParamValue::Bool(_) | ParamValue::Float(_) | ParamValue::Str(_) => None,
                    }
                }
            }
        )*
    };
}

integer_params! {
    i8 => I8, i16 => I16, i32 => I32, i64 => I64,
    u8 => U8, u16 => U16, u32 => U32, u64 => U64,
}

impl FromParamValue for bool {
    const PARAM_TYPE: ParamType = ParamType::Bool;

    fn from_param_value(value: &ParamValue) -> Option<Self> {
        match value {
            ParamValue::Bool(b) => Some(*b),
            _ => None,
        }
    }
}

impl FromParamValue for f64 {
    const PARAM_TYPE: ParamType = ParamType::F64;

    fn from_param_value(value: &ParamValue) -> Option<Self> {
        match value {
            ParamValue::Float(x) if x.is_finite() => Some(*x),
            _ => None,
        }
    }
}

impl FromParamValue for f32 {
    const PARAM_TYPE: ParamType = ParamType::F32;

    fn from_param_value(value: &ParamValue) -> Option<Self> {
        match value {
            ParamValue::Float(x) if x.is_finite() => {
                let narrowed = *x as f32;
                narrowed.is_finite().then_some(narrowed)
            }
            _ => None,
        }
    }
}

impl FromParamValue for String {
    const PARAM_TYPE: ParamType = ParamType::String;

    fn from_param_value(value: &ParamValue) -> Option<Self> {
        match value {
            ParamValue::Str(s) => Some(s.clone()),
            _ => None,
        }
    }
}

/// One declared param: what `#[param(default = .., min = .., max = ..,
/// description = "..")]` says about one field. Emitted by the macro into the
/// node info (in process and in the cdylib info JSON) and shown by
/// `cerulion node info`.
///
/// `#[serde(deny_unknown_fields)]` on purpose, although this struct crosses the
/// cdylib boundary as JSON: a spec key this host does not know is a constraint
/// it cannot enforce, and a node must never load with one of its declared
/// bounds silently dropped.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ParamSpec {
    pub name: String,
    #[serde(rename = "type")]
    pub ty: ParamType,
    pub default: ParamValue,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<ParamValue>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<ParamValue>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

impl ParamSpec {
    /// The macro proves these at compile time for a `#[param]` field; a
    /// hand-written info JSON has no such proof, so the loader checks them
    /// again before it trusts the spec: the default has the declared type,
    /// `min`/`max` appear only on a numeric type and have it too, `min <= max`,
    /// and the default lies within the bounds.
    pub fn validate(&self) -> Result<(), String> {
        let name = &self.name;
        let ty = self.ty;
        if ty.canonicalize(&self.default).is_none() {
            return Err(format!(
                "param '{name}': default {} is not a {ty}",
                self.default
            ));
        }
        for (label, bound) in [("min", &self.min), ("max", &self.max)] {
            if let Some(bound) = bound {
                if !ty.is_numeric() {
                    return Err(format!(
                        "param '{name}': `{label}` is not accepted on a {ty} param; bounds \
                         apply to integer and float params only"
                    ));
                }
                if ty.canonicalize(bound).is_none() {
                    return Err(format!("param '{name}': {label} {bound} is not a {ty}"));
                }
            }
        }
        if let (Some(min), Some(max)) = (&self.min, &self.max) {
            if let (Some(lo), Some(hi)) = (min.as_number(), max.as_number()) {
                if hi.less_than(lo) {
                    return Err(format!(
                        "param '{name}': min {min} is greater than max {max}"
                    ));
                }
            }
        }
        if !within_bounds(&self.default, self.min.as_ref(), self.max.as_ref()) {
            return Err(format!(
                "param '{name}': default {} is outside {}",
                self.default,
                render_bounds(self.min.as_ref(), self.max.as_ref())
            ));
        }
        Ok(())
    }
}

/// The resolved values of one node, in the node's declaration order: every
/// declared param is present, each holding its default or the graph's value.
pub type ParamValues = IndexMap<String, ParamValue>;

/// Why a node's `params:` were refused. Rendered through [`fmt::Display`]
/// into the load error the user sees; every arm names the node and the
/// param.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamProblem {
    /// `params:` names a param the node type does not declare.
    UnknownParam {
        node: String,
        param: String,
        /// Every name the node type declares, in declaration order. Empty
        /// when the type declares no params at all.
        declared: Vec<String>,
        /// The declared name closest to the unknown one, when any is declared.
        nearest: Option<String>,
    },
    /// The value is not of the declared type, or does not fit its width.
    WrongType {
        node: String,
        param: String,
        expected: ParamType,
        got: ParamValue,
    },
    /// A float that is NaN or infinite.
    NotFinite {
        node: String,
        param: String,
        got: ParamValue,
    },
    /// The value lies outside the declared inclusive bounds.
    OutOfRange {
        node: String,
        param: String,
        value: ParamValue,
        min: Option<ParamValue>,
        max: Option<ParamValue>,
    },
}

impl fmt::Display for ParamProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownParam {
                node,
                param,
                declared,
                nearest,
            } => {
                write!(f, "node '{node}': unknown param '{param}'")?;
                if declared.is_empty() {
                    return write!(
                        f,
                        "; this node type declares no params, so `params:` is not accepted on it"
                    );
                }
                if let Some(nearest) = nearest {
                    write!(f, " (did you mean '{nearest}'?)")?;
                }
                write!(f, "; declared params: {}", declared.join(", "))
            }
            Self::WrongType {
                node,
                param,
                expected,
                got,
            } => {
                write!(
                    f,
                    "node '{node}': param '{param}' expects {expected}, got {} {got}",
                    got.kind()
                )?;
                match (expected, got) {
                    (ParamType::F32 | ParamType::F64, ParamValue::Int(_) | ParamValue::UInt(_)) => {
                        write!(
                            f,
                            "; a float param takes a float value, for example {got}.0"
                        )
                    }
                    // The only way a float misses an `f32` is by magnitude:
                    // `canonicalize` already proved it is a finite float.
                    (ParamType::F32, ParamValue::Float(_)) => {
                        write!(f, "; the value does not fit f32")
                    }
                    (_, ParamValue::Int(_) | ParamValue::UInt(_)) if expected.is_numeric() => {
                        write!(f, "; the value does not fit {expected}")
                    }
                    // An unquoted `123` or `true` in YAML is a number or a
                    // bool, not text; quoting it is the fix.
                    (ParamType::String, _) => {
                        write!(f, "; quote it to keep it as text, for example \"{got}\"")
                    }
                    _ => Ok(()),
                }
            }
            Self::NotFinite { node, param, got } => write!(
                f,
                "node '{node}': param '{param}' must be a finite number, got {got}"
            ),
            Self::OutOfRange {
                node,
                param,
                value,
                min,
                max,
            } => write!(
                f,
                "node '{node}': param '{param}' = {value} is outside {}",
                render_bounds(min.as_ref(), max.as_ref())
            ),
        }
    }
}

impl From<ParamProblem> for TransportError {
    fn from(problem: ParamProblem) -> Self {
        TransportError::GraphError {
            reason: problem.to_string(),
        }
    }
}

/// `[min, max]`, `[min, ..]` or `[.., max]`; bounds are inclusive.
fn render_bounds(min: Option<&ParamValue>, max: Option<&ParamValue>) -> String {
    match (min, max) {
        (Some(lo), Some(hi)) => format!("[{lo}, {hi}]"),
        (Some(lo), None) => format!("[{lo}, ..]"),
        (None, Some(hi)) => format!("[.., {hi}]"),
        (None, None) => "[.., ..]".to_string(),
    }
}

/// Inclusive on both ends. A value or bound that is not a number is never
/// out of range (the type check in front of this has already decided it).
fn within_bounds(value: &ParamValue, min: Option<&ParamValue>, max: Option<&ParamValue>) -> bool {
    let Some(v) = value.as_number() else {
        return true;
    };
    if let Some(lo) = min.and_then(ParamValue::as_number) {
        if v.less_than(lo) {
            return false;
        }
    }
    if let Some(hi) = max.and_then(ParamValue::as_number) {
        if hi.less_than(v) {
            return false;
        }
    }
    true
}

/// Resolve one node's params: the macro defaults in declaration order, then
/// the graph's `params:` values over them, each value checked against the
/// declared name, type and bounds. The first problem refuses the node.
///
/// Pure: no clock, no transport, no IO. Two calls with the same inputs return
/// the same map in the same order.
pub fn resolve_params(
    node: &str,
    specs: &[ParamSpec],
    supplied: &ParamValues,
) -> Result<ParamValues, ParamProblem> {
    // Defaults are seeded in the same canonical form a supplied value takes
    // (`UInt` for an unsigned width, whichever variant the info JSON parsed
    // it into), so a default and a graph value for one param list, record
    // and compare alike. Every spec reaching this point has passed
    // `ParamSpec::validate`, so the canonical form exists; the fallback
    // keeps the declared default rather than refusing a graph over a spec
    // the loader already accepted.
    let mut resolved: ParamValues = specs
        .iter()
        .map(|spec| {
            let default = spec
                .ty
                .canonicalize(&spec.default)
                .unwrap_or_else(|| spec.default.clone());
            (spec.name.clone(), default)
        })
        .collect();
    for (name, value) in supplied {
        let Some(spec) = specs.iter().find(|spec| &spec.name == name) else {
            let declared: Vec<String> = specs.iter().map(|spec| spec.name.clone()).collect();
            let legal: Vec<&str> = declared.iter().map(String::as_str).collect();
            let nearest = (!legal.is_empty())
                .then(|| super::node::nearest_legal_key(name, &legal).to_string());
            return Err(ParamProblem::UnknownParam {
                node: node.to_string(),
                param: name.clone(),
                declared,
                nearest,
            });
        };
        if let ParamValue::Float(x) = value {
            if !x.is_finite() {
                return Err(ParamProblem::NotFinite {
                    node: node.to_string(),
                    param: name.clone(),
                    got: value.clone(),
                });
            }
        }
        let Some(canonical) = spec.ty.canonicalize(value) else {
            return Err(ParamProblem::WrongType {
                node: node.to_string(),
                param: name.clone(),
                expected: spec.ty,
                got: value.clone(),
            });
        };
        if !within_bounds(&canonical, spec.min.as_ref(), spec.max.as_ref()) {
            return Err(ParamProblem::OutOfRange {
                node: node.to_string(),
                param: name.clone(),
                value: canonical,
                min: spec.min.clone(),
                max: spec.max.clone(),
            });
        }
        resolved.insert(name.clone(), canonical);
    }
    Ok(resolved)
}

/// Resolve the `params:` of every native node in `config` against the
/// declared specs in `infos` (keyed by node id, as the graph build gathers
/// them). The first problem refuses the graph. A node with no entry in
/// `infos` is skipped here; the build reports it as a missing node entry.
///
/// A `ros2:` entry carries its parameters under `ros2.params`; `params:` on
/// such an entry is refused by name so it is never a silently dead key.
pub fn resolve_graph_params(
    config: &super::config::GraphConfig,
    infos: &IndexMap<String, super::node::NodeInfo>,
) -> Result<IndexMap<String, ParamValues>, TransportError> {
    let mut resolved = IndexMap::new();
    for node in &config.nodes {
        if node.is_ros2() {
            if !node.params.is_empty() {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "node '{}': `params:` is not accepted on a `ros2:` entry; put ROS 2 \
                         parameters under `ros2.params` on that entry",
                        node.id
                    ),
                });
            }
            continue;
        }
        let Some(info) = infos.get(&node.id) else {
            continue;
        };
        let values = resolve_params(&node.id, info.params(), &node.params)?;
        resolved.insert(node.id.clone(), values);
    }
    Ok(resolved)
}

/// The resolved values as one JSON object, in order: how the host hands them
/// to a cdylib node through `cerulion_node_init_with_params`. Cold path (one
/// call per node at graph build).
pub fn params_to_json(values: &ParamValues) -> Result<String, String> {
    serde_json::to_string(values).map_err(|e| e.to_string())
}

/// Inverse of [`params_to_json`]; the cdylib side of the same handoff.
pub fn params_from_json(json: &str) -> Result<ParamValues, String> {
    serde_json::from_str(json).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(name: &str, ty: ParamType, default: ParamValue) -> ParamSpec {
        ParamSpec {
            name: name.to_string(),
            ty,
            default,
            min: None,
            max: None,
            description: None,
        }
    }

    fn bounded(mut spec: ParamSpec, min: Option<ParamValue>, max: Option<ParamValue>) -> ParamSpec {
        spec.min = min;
        spec.max = max;
        spec
    }

    /// The node under test: a float with bounds, a signed integer, a string.
    fn three_specs() -> Vec<ParamSpec> {
        vec![
            bounded(
                spec("gain", ParamType::F64, ParamValue::Float(1.5)),
                Some(ParamValue::Float(0.0)),
                Some(ParamValue::Float(5.0)),
            ),
            spec("offset", ParamType::I32, ParamValue::Int(-2)),
            spec("label", ParamType::String, ParamValue::Str("base".into())),
        ]
    }

    fn supplied(pairs: &[(&str, ParamValue)]) -> ParamValues {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    fn yaml_params(yaml: &str) -> Result<ParamValues, serde_yaml::Error> {
        serde_yaml::from_str(yaml)
    }

    // ---- happy ----

    #[test]
    fn defaults_alone_resolve_every_param_in_declaration_order() {
        let resolved = resolve_params("a", &three_specs(), &ParamValues::new()).unwrap();
        let got: Vec<(&str, &ParamValue)> = resolved.iter().map(|(k, v)| (k.as_str(), v)).collect();
        assert_eq!(
            got,
            vec![
                ("gain", &ParamValue::Float(1.5)),
                ("offset", &ParamValue::Int(-2)),
                ("label", &ParamValue::Str("base".into())),
            ]
        );
    }

    #[test]
    fn yaml_values_override_defaults_and_keep_declaration_order() {
        let yaml = supplied(&[
            ("label", ParamValue::Str("x".into())),
            ("gain", ParamValue::Float(2.5)),
        ]);
        let resolved = resolve_params("a", &three_specs(), &yaml).unwrap();
        let got: Vec<(&str, &ParamValue)> = resolved.iter().map(|(k, v)| (k.as_str(), v)).collect();
        assert_eq!(
            got,
            vec![
                ("gain", &ParamValue::Float(2.5)),
                ("offset", &ParamValue::Int(-2)),
                ("label", &ParamValue::Str("x".into())),
            ]
        );
    }

    #[test]
    fn a_string_param_given_off_stays_the_string_off() {
        let yaml = yaml_params("label: off\n").unwrap();
        assert_eq!(yaml["label"], ParamValue::Str("off".into()));
        let resolved = resolve_params("a", &three_specs(), &yaml).unwrap();
        assert_eq!(resolved["label"], ParamValue::Str("off".into()));
    }

    #[test]
    fn yaml_scalars_land_in_the_variant_their_spelling_says() {
        let yaml =
            yaml_params("a: 5\nb: -5\nc: 2.5\nd: true\ne: '5'\nf: 18446744073709551615\n").unwrap();
        assert_eq!(yaml["a"], ParamValue::Int(5));
        assert_eq!(yaml["b"], ParamValue::Int(-5));
        assert_eq!(yaml["c"], ParamValue::Float(2.5));
        assert_eq!(yaml["d"], ParamValue::Bool(true));
        assert_eq!(yaml["e"], ParamValue::Str("5".into()));
        assert_eq!(yaml["f"], ParamValue::UInt(u64::MAX));
    }

    // ---- edge ----

    #[test]
    fn bounds_are_inclusive_at_both_ends() {
        for ok in [0.0, 5.0, 2.5] {
            let yaml = supplied(&[("gain", ParamValue::Float(ok))]);
            let resolved = resolve_params("a", &three_specs(), &yaml).unwrap();
            assert_eq!(resolved["gain"], ParamValue::Float(ok));
        }
        let above = supplied(&[("gain", ParamValue::Float(5.000001))]);
        let err = resolve_params("a", &three_specs(), &above).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' = 5.000001 is outside [0.0, 5.0]"
        );
        let below = supplied(&[("gain", ParamValue::Float(-0.1))]);
        let err = resolve_params("a", &three_specs(), &below).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' = -0.1 is outside [0.0, 5.0]"
        );
    }

    #[test]
    fn a_one_sided_bound_renders_its_open_side() {
        let specs = vec![bounded(
            spec("level", ParamType::U8, ParamValue::UInt(1)),
            Some(ParamValue::UInt(1)),
            None,
        )];
        let err =
            resolve_params("a", &specs, &supplied(&[("level", ParamValue::Int(0))])).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'level' = 0 is outside [1, ..]"
        );
    }

    #[test]
    fn an_empty_params_map_resolves_to_the_defaults() {
        let yaml = yaml_params("{}\n").unwrap();
        assert!(yaml.is_empty());
        let resolved = resolve_params("a", &three_specs(), &yaml).unwrap();
        assert_eq!(resolved.len(), 3);
        assert_eq!(resolved["gain"], ParamValue::Float(1.5));
    }

    #[test]
    fn a_node_that_declares_no_params_resolves_to_an_empty_map() {
        let resolved = resolve_params("a", &[], &ParamValues::new()).unwrap();
        assert!(resolved.is_empty());
    }

    // ---- adversarial ----

    #[test]
    fn an_integer_that_does_not_fit_the_declared_width_is_refused() {
        let specs = vec![spec("level", ParamType::U8, ParamValue::UInt(1))];
        let err =
            resolve_params("a", &specs, &supplied(&[("level", ParamValue::Int(256))])).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'level' expects u8, got integer 256; the value does not fit u8"
        );
        let err =
            resolve_params("a", &specs, &supplied(&[("level", ParamValue::Int(-1))])).unwrap_err();
        assert!(matches!(err, ParamProblem::WrongType { .. }), "{err}");
    }

    #[test]
    fn a_huge_unsigned_value_fits_u64_but_not_i64() {
        let big = supplied(&[("big", ParamValue::UInt(u64::MAX))]);
        let u = vec![spec("big", ParamType::U64, ParamValue::UInt(0))];
        assert_eq!(
            resolve_params("a", &u, &big).unwrap()["big"],
            ParamValue::UInt(u64::MAX)
        );
        let i = vec![spec("big", ParamType::I64, ParamValue::Int(0))];
        assert!(matches!(
            resolve_params("a", &i, &big).unwrap_err(),
            ParamProblem::WrongType { .. }
        ));
    }

    #[test]
    fn nan_and_infinity_are_refused_even_without_bounds() {
        let specs = vec![spec("gain", ParamType::F64, ParamValue::Float(1.0))];
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let err = resolve_params("a", &specs, &supplied(&[("gain", ParamValue::Float(bad))]))
                .unwrap_err();
            assert!(matches!(err, ParamProblem::NotFinite { .. }), "{err}");
        }
        let yaml = yaml_params("gain: .nan\n").unwrap();
        let err = resolve_params("a", &specs, &yaml).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' must be a finite number, got NaN"
        );
    }

    #[test]
    fn a_float_too_large_for_f32_is_refused_naming_the_width() {
        let specs = vec![spec("gain", ParamType::F32, ParamValue::Float(1.0))];
        let err = resolve_params("a", &specs, &supplied(&[("gain", ParamValue::Float(1e40))]))
            .unwrap_err();
        assert!(matches!(err, ParamProblem::WrongType { .. }), "{err}");
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' expects f32, got float 1e40; the value does not fit f32"
        );
    }

    /// `0.1`, `0.3` and `0.7` all round when narrowed to `f32` (`0.1f32` is
    /// above `0.1`, `0.7f32` below `0.7`), so a bounds check on the narrowed
    /// value would refuse a value exactly at the declared bound. The check
    /// runs on the value as written, and the resolved value stays as
    /// written too.
    #[test]
    fn an_f32_value_exactly_at_a_bound_passes_and_keeps_the_literal_as_written() {
        for (lo, hi, at) in [(0.7, 1.0, 0.7), (0.0, 0.1, 0.1), (0.0, 0.3, 0.3)] {
            let specs = vec![bounded(
                spec("gain", ParamType::F32, ParamValue::Float(lo)),
                Some(ParamValue::Float(lo)),
                Some(ParamValue::Float(hi)),
            )];
            let resolved =
                resolve_params("a", &specs, &supplied(&[("gain", ParamValue::Float(at))]))
                    .unwrap_or_else(|e| panic!("{at} at a bound of [{lo}, {hi}] must pass: {e}"));
            assert_eq!(resolved["gain"], ParamValue::Float(at));
            assert_eq!(resolved["gain"].to_string(), format!("{at:?}"));
        }
        // Just past the bound is still refused, and the refusal shows the
        // value as written.
        let specs = vec![bounded(
            spec("gain", ParamType::F32, ParamValue::Float(0.05)),
            None,
            Some(ParamValue::Float(0.1)),
        )];
        let err = resolve_params(
            "a",
            &specs,
            &supplied(&[("gain", ParamValue::Float(0.1000001))]),
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' = 0.1000001 is outside [.., 0.1]"
        );
        // The field write is where the narrowing happens.
        assert_eq!(f32::from_param_value(&ParamValue::Float(0.1)), Some(0.1f32));
    }

    #[test]
    fn a_default_is_seeded_in_the_same_canonical_form_as_a_supplied_value() {
        // A `u8` default read from an info JSON parses as `Int(1)`; a
        // supplied `1` canonicalizes to `UInt(1)`. Both land as `UInt`.
        let specs = vec![spec("level", ParamType::U8, ParamValue::Int(1))];
        let from_default = resolve_params("a", &specs, &ParamValues::new()).unwrap();
        let from_graph =
            resolve_params("a", &specs, &supplied(&[("level", ParamValue::Int(1))])).unwrap();
        assert_eq!(from_default["level"], ParamValue::UInt(1));
        assert_eq!(from_default, from_graph);
    }

    #[test]
    fn yaml_shapes_that_are_not_scalars_are_refused_at_parse() {
        assert!(yaml_params("gain: [1, 2]\n").is_err());
        assert!(yaml_params("gain: {a: 1}\n").is_err());
        assert!(
            yaml_params("gain:\n").is_err(),
            "a null value is not a scalar"
        );
    }

    // ---- every error arm ----

    #[test]
    fn an_unknown_name_is_refused_with_the_nearest_declared_name() {
        let err = resolve_params(
            "a",
            &three_specs(),
            &supplied(&[("gian", ParamValue::Float(1.0))]),
        )
        .unwrap_err();
        assert_eq!(
            err,
            ParamProblem::UnknownParam {
                node: "a".into(),
                param: "gian".into(),
                declared: vec!["gain".into(), "offset".into(), "label".into()],
                nearest: Some("gain".into()),
            }
        );
        assert_eq!(
            err.to_string(),
            "node 'a': unknown param 'gian' (did you mean 'gain'?); declared params: gain, offset, label"
        );
    }

    #[test]
    fn an_unknown_name_on_a_node_that_declares_no_params_says_so() {
        let err =
            resolve_params("a", &[], &supplied(&[("gain", ParamValue::Float(1.0))])).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': unknown param 'gain'; this node type declares no params, so `params:` is not accepted on it"
        );
    }

    #[test]
    fn a_dotted_name_is_an_unknown_name_not_a_nested_path() {
        let err = resolve_params(
            "a",
            &three_specs(),
            &supplied(&[("motor.gain", ParamValue::Float(1.0))]),
        )
        .unwrap_err();
        assert!(
            matches!(err, ParamProblem::UnknownParam { ref param, .. } if param == "motor.gain")
        );
    }

    #[test]
    fn an_integer_for_a_float_param_is_the_wrong_type_and_names_the_fix() {
        let err = resolve_params(
            "a",
            &three_specs(),
            &supplied(&[("gain", ParamValue::Int(2))]),
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'gain' expects f64, got integer 2; a float param takes a float value, for example 2.0"
        );
    }

    #[test]
    fn a_string_for_a_bool_param_is_the_wrong_type() {
        let specs = vec![spec("enabled", ParamType::Bool, ParamValue::Bool(true))];
        let yaml = yaml_params("enabled: off\n").unwrap();
        let err = resolve_params("a", &specs, &yaml).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'enabled' expects bool, got string \"off\""
        );
    }

    #[test]
    fn a_float_for_an_integer_param_is_the_wrong_type() {
        let err = resolve_params(
            "a",
            &three_specs(),
            &supplied(&[("offset", ParamValue::Float(1.5))]),
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'offset' expects i32, got float 1.5"
        );
    }

    #[test]
    fn a_number_for_a_string_param_is_the_wrong_type() {
        let err = resolve_params(
            "a",
            &three_specs(),
            &supplied(&[("label", ParamValue::Int(7))]),
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'label' expects String, got integer 7; quote it to keep it as text, for example \"7\""
        );
        // An unquoted `true` in YAML is a bool; the hint shows the quoted form.
        let yaml = yaml_params("label: true\n").unwrap();
        let err = resolve_params("a", &three_specs(), &yaml).unwrap_err();
        assert_eq!(
            err.to_string(),
            "node 'a': param 'label' expects String, got bool true; quote it to keep it as text, for example \"true\""
        );
    }

    #[test]
    fn a_problem_becomes_a_graph_error_carrying_the_same_text() {
        let problem = ParamProblem::NotFinite {
            node: "a".into(),
            param: "gain".into(),
            got: ParamValue::Float(f64::NAN),
        };
        let text = problem.to_string();
        match TransportError::from(problem) {
            TransportError::GraphError { reason } => assert_eq!(reason, text),
            other => panic!("expected GraphError, got {other:?}"),
        }
    }

    // ---- spec validation (the loader's second look at a hand-written spec) ----

    #[test]
    fn spec_validation_catches_each_inconsistency() {
        let bad_default = spec("gain", ParamType::F64, ParamValue::Int(2));
        assert_eq!(
            bad_default.validate().unwrap_err(),
            "param 'gain': default 2 is not a f64"
        );
        let bounds_on_string = bounded(
            spec("label", ParamType::String, ParamValue::Str("x".into())),
            Some(ParamValue::Int(0)),
            None,
        );
        assert_eq!(
            bounds_on_string.validate().unwrap_err(),
            "param 'label': `min` is not accepted on a String param; bounds apply to integer and float params only"
        );
        let inverted = bounded(
            spec("gain", ParamType::F64, ParamValue::Float(1.0)),
            Some(ParamValue::Float(5.0)),
            Some(ParamValue::Float(0.0)),
        );
        assert_eq!(
            inverted.validate().unwrap_err(),
            "param 'gain': min 5.0 is greater than max 0.0"
        );
        let default_outside = bounded(
            spec("gain", ParamType::F64, ParamValue::Float(9.0)),
            Some(ParamValue::Float(0.0)),
            Some(ParamValue::Float(5.0)),
        );
        assert_eq!(
            default_outside.validate().unwrap_err(),
            "param 'gain': default 9.0 is outside [0.0, 5.0]"
        );
        let bound_of_wrong_type = bounded(
            spec("gain", ParamType::F64, ParamValue::Float(1.0)),
            None,
            Some(ParamValue::Int(5)),
        );
        assert_eq!(
            bound_of_wrong_type.validate().unwrap_err(),
            "param 'gain': max 5 is not a f64"
        );
        for good in three_specs() {
            good.validate().unwrap();
        }
    }

    // ---- determinism and the wire shapes ----

    #[test]
    fn two_resolutions_are_identical_and_the_json_handoff_round_trips() {
        let yaml = yaml_params("gain: 2.5\nlabel: x\n").unwrap();
        let first = resolve_params("a", &three_specs(), &yaml).unwrap();
        let second = resolve_params("a", &three_specs(), &yaml).unwrap();
        assert_eq!(first, second);
        let json = params_to_json(&first).unwrap();
        assert_eq!(json, r#"{"gain":2.5,"offset":-2,"label":"x"}"#);
        assert_eq!(params_from_json(&json).unwrap(), first);
        // A whole-number float keeps its point on the wire, so the cdylib
        // side reads a float back, never an integer.
        let whole = supplied(&[("gain", ParamValue::Float(2.0))]);
        assert_eq!(params_to_json(&whole).unwrap(), r#"{"gain":2.0}"#);
        assert_eq!(params_from_json(r#"{"gain":2.0}"#).unwrap(), whole);
    }

    /// The cdylib side must hold the SAME bits the host resolved, or the
    /// in-process and cdylib paths disagree and a recording does not replay
    /// byte for byte. These values need all 17 significant digits, which
    /// `serde_json` only reads back exactly under `float_roundtrip`.
    #[test]
    fn the_json_handoff_keeps_every_float_bit() {
        let hard: [f64; 6] = [
            0.1 + 0.2,
            1.0 / 3.0,
            f64::MIN_POSITIVE,
            f64::MAX,
            5e-324,
            -2.2250738585072014e-308,
        ];
        for x in hard {
            let values = supplied(&[("gain", ParamValue::Float(x))]);
            let json = params_to_json(&values).unwrap();
            let back = params_from_json(&json).unwrap();
            let ParamValue::Float(y) = &back["gain"] else {
                panic!("{json} did not read back as a float");
            };
            assert_eq!(
                y.to_bits(),
                x.to_bits(),
                "{x:?} via {json} came back as {y:?}"
            );
        }
    }

    /// The host side of the two-sided pin on the cdylib info JSON: a
    /// description holding a newline, a tab and another control character
    /// arrives escaped (the macro's `param_attr::json_string` writes exactly
    /// this text) and parses back to the characters the author wrote.
    #[test]
    fn a_spec_whose_text_holds_control_characters_parses_from_the_escaped_fragment() {
        let spec = spec_json(
            r#"{"name":"gain","type":"f64","default":1.0,"description":"gain\nused by\tcmd.x\u0001"}"#,
        );
        assert_eq!(
            spec.description.as_deref(),
            Some("gain\nused by\tcmd.x\u{1}")
        );
        let text = spec_json(r#"{"name":"frame_id","type":"String","default":"a\"b\\c\r\n"}"#);
        assert_eq!(text.default, ParamValue::Str("a\"b\\c\r\n".into()));
    }

    #[test]
    fn a_spec_serializes_under_the_rust_type_spelling() {
        let mut spec = bounded(
            spec("gain", ParamType::F64, ParamValue::Float(2.0)),
            Some(ParamValue::Float(0.0)),
            Some(ParamValue::Float(5.0)),
        );
        spec.description = Some("loop gain".into());
        let json = serde_json::to_string(&spec).unwrap();
        assert_eq!(
            json,
            r#"{"name":"gain","type":"f64","default":2.0,"min":0.0,"max":5.0,"description":"loop gain"}"#
        );
        assert_eq!(serde_json::from_str::<ParamSpec>(&json).unwrap(), spec);
        let string_spec = spec_json(r#"{"name":"frame","type":"String","default":"base_link"}"#);
        assert_eq!(string_spec.ty, ParamType::String);
        assert_eq!(string_spec.default, ParamValue::Str("base_link".into()));
        assert!(
            serde_json::from_str::<ParamSpec>(
                r#"{"name":"g","type":"f64","default":1.0,"step":0.1}"#
            )
            .is_err(),
            "an unknown spec key is a constraint the host cannot enforce; refuse it"
        );
    }

    fn spec_json(json: &str) -> ParamSpec {
        serde_json::from_str(json).unwrap()
    }

    #[test]
    fn values_display_as_a_user_would_write_them() {
        assert_eq!(ParamValue::Float(2.0).to_string(), "2.0");
        assert_eq!(ParamValue::Int(-3).to_string(), "-3");
        assert_eq!(ParamValue::UInt(7).to_string(), "7");
        assert_eq!(ParamValue::Bool(false).to_string(), "false");
        assert_eq!(ParamValue::Str("off".into()).to_string(), "\"off\"");
    }
}

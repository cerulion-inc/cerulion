// SPDX-License-Identifier: AGPL-3.0-only
//! The resume rule for params: a param is not node state.
//!
//! The `#[cerulion_node]` state fold-in leaves every `#[param]` field out of
//! the node's captured state, so an anchor never carries a param and a
//! restore never writes one. A resumed graph runs on the values IN FORCE: the
//! ones its own build resolved and handed to each node before `init`.
//!
//! That is only safe when those values are the ones the recording ran with,
//! so a resume compares them first. The values recorded with an anchor are
//! the recording's effective graph: a run embeds every node's RESOLVED params
//! (the graph's values and the defaults it left alone), and a value is fixed
//! for the whole run, so the embedded map is the value at every anchor the
//! run took. [`ParamsInForce::differences`] lists each param whose value in
//! force differs from the recorded one, including a param the recording
//! names no value for (a node that gained the param since, or a default the
//! run could not record). The caller refuses on any difference unless the
//! operator opted in to the current values.
//!
//! A change to a param's TYPE does not invalidate an anchor: no param is in
//! `STATE_SHAPE`. It shows up here instead, as a recorded value that differs
//! from the value in force.
//!
//! Cold: one call per resumed node, before the first step.

use std::fmt;

use super::params::{ParamSpec, ParamValue, ParamValues};

/// One node's params as its build resolved them: the declared specs, in
/// declaration order, and the value in force for each.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParamsInForce {
    /// What the node's type declares (`NodeInfo::params`).
    pub specs: Vec<ParamSpec>,
    /// The resolved value of every declared param, in declaration order.
    pub values: ParamValues,
}

/// One param whose value in force differs from the recorded one.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamDifference {
    /// The node id.
    pub node: String,
    /// The param name.
    pub param: String,
    /// The value the recording names, in the declared type's canonical form.
    /// `None` when the recording names no value for this param.
    pub recorded: Option<ParamValue>,
    /// The value in force. `None` when the node no longer declares the param.
    pub current: Option<ParamValue>,
}

impl fmt::Display for ParamDifference {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let show = |value: &Option<ParamValue>| match value {
            Some(v) => v.to_string(),
            None => "no value".to_string(),
        };
        write!(
            f,
            "node '{}' param '{}': recorded {}, current {}",
            self.node,
            self.param,
            show(&self.recorded),
            show(&self.current)
        )
    }
}

impl ParamsInForce {
    /// Every param of node `node` whose value in force differs from
    /// `recorded`, in declaration order, then any recorded name the node no
    /// longer declares, in recorded order. Empty when the two agree.
    ///
    /// A recorded value is compared in the declared type's canonical form,
    /// so a recording that wrote `count: 5` (read back as a signed integer)
    /// for a `u32` param agrees with the in-force unsigned `5`.
    pub fn differences(&self, node: &str, recorded: &ParamValues) -> Vec<ParamDifference> {
        let mut out = Vec::new();
        for spec in &self.specs {
            let current = self.values.get(&spec.name).cloned();
            let recorded_value = recorded
                .get(&spec.name)
                .map(|v| spec.ty.canonicalize(v).unwrap_or_else(|| v.clone()));
            if recorded_value != current {
                out.push(ParamDifference {
                    node: node.to_string(),
                    param: spec.name.clone(),
                    recorded: recorded_value,
                    current,
                });
            }
        }
        for (name, value) in recorded {
            if !self.specs.iter().any(|spec| &spec.name == name) {
                out.push(ParamDifference {
                    node: node.to_string(),
                    param: name.clone(),
                    recorded: Some(value.clone()),
                    current: None,
                });
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::params::ParamType;

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

    fn values(pairs: &[(&str, ParamValue)]) -> ParamValues {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    /// `gain: f64` in force at 3.0 and `frame_id: String` at "base_link".
    fn in_force() -> ParamsInForce {
        ParamsInForce {
            specs: vec![
                spec("gain", ParamType::F64, ParamValue::Float(1.0)),
                spec("frame_id", ParamType::String, ParamValue::Str("x".into())),
            ],
            values: values(&[
                ("gain", ParamValue::Float(3.0)),
                ("frame_id", ParamValue::Str("base_link".into())),
            ]),
        }
    }

    #[test]
    fn equal_values_have_no_difference() {
        let recorded = values(&[
            ("gain", ParamValue::Float(3.0)),
            ("frame_id", ParamValue::Str("base_link".into())),
        ]);
        assert!(in_force().differences("cam", &recorded).is_empty());
    }

    #[test]
    fn an_unsigned_value_read_back_as_a_signed_integer_compares_canonically() {
        // A YAML `5` parses as `Int(5)`; a `u32` param's value in force is the
        // canonical `UInt(5)`. They are the same value.
        let in_force = ParamsInForce {
            specs: vec![spec("count", ParamType::U32, ParamValue::UInt(1))],
            values: values(&[("count", ParamValue::UInt(5))]),
        };
        let recorded = values(&[("count", ParamValue::Int(5))]);
        assert!(in_force.differences("cam", &recorded).is_empty());
    }

    #[test]
    fn a_changed_value_is_one_line_naming_node_param_and_both_values() {
        let recorded = values(&[
            ("gain", ParamValue::Float(2.0)),
            ("frame_id", ParamValue::Str("base_link".into())),
        ]);
        let lines: Vec<String> = in_force()
            .differences("cam", &recorded)
            .iter()
            .map(ToString::to_string)
            .collect();
        assert_eq!(
            lines,
            vec!["node 'cam' param 'gain': recorded 2.0, current 3.0".to_string()]
        );
    }

    #[test]
    fn a_param_the_recording_does_not_name_and_one_the_node_dropped_both_differ() {
        let recorded = values(&[
            ("gain", ParamValue::Float(3.0)),
            ("legacy", ParamValue::Bool(true)),
        ]);
        let lines: Vec<String> = in_force()
            .differences("cam", &recorded)
            .iter()
            .map(ToString::to_string)
            .collect();
        assert_eq!(
            lines,
            vec![
                "node 'cam' param 'frame_id': recorded no value, current \"base_link\"".to_string(),
                "node 'cam' param 'legacy': recorded true, current no value".to_string(),
            ]
        );
    }

    #[test]
    fn a_node_without_params_and_an_empty_recording_agree() {
        assert!(ParamsInForce::default()
            .differences("relay", &ParamValues::new())
            .is_empty());
    }
}

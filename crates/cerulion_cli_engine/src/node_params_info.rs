// SPDX-License-Identifier: AGPL-3.0-only
//! The params table of `cerulion node info`: the `#[param]` fields a node
//! type declares, read from the node info of its BUILT library (the same
//! document `graph validate` and `graph run` check a graph's `params:`
//! against), so the table shows exactly what a graph may set.

use std::path::Path;

use cerulion_core::graph::params::ParamSpec;
use cerulion_core::NodeEntry as _;

/// What `node info` can say about a type's params.
#[derive(Debug, Clone, PartialEq)]
pub enum DeclaredParams {
    /// No built library for the type: the specs live in the library's node
    /// info, so there is nothing to read until `cerulion node build <type>`.
    NotBuilt,
    /// The library exists but its node info could not be read.
    Unreadable(String),
    /// The declared specs, in declaration order (empty: the type declares none).
    Declared(Vec<ParamSpec>),
}

/// Read the type's declared params from its built library, preferring the
/// debug build and falling back to release, the same resolution `graph run`
/// uses without `--release`.
pub fn declared_params(workspace_root: &Path, node_type: &str) -> DeclaredParams {
    let Ok(lib) = crate::graph_cmd::find_cdylib(workspace_root, node_type, false) else {
        return DeclaredParams::NotBuilt;
    };
    match cerulion_core::graph::node::DylibNodeEntry::load(&lib.path).and_then(|node| node.info()) {
        Ok(info) => DeclaredParams::Declared(info.params().to_vec()),
        Err(e) => DeclaredParams::Unreadable(e.to_string()),
    }
}

/// The `Params:` section of `node info`, one row per declared param with
/// its type, default, bounds and description. Pure: the caller prints it.
pub fn render_params(node_type: &str, declared: &DeclaredParams) -> String {
    let specs = match declared {
        DeclaredParams::NotBuilt => {
            return format!(
                "Params: not read (no built library; run `cerulion node build {node_type}` \
                 and ask again)\n"
            );
        }
        DeclaredParams::Unreadable(reason) => {
            return format!("Params: not read (the node library could not be read: {reason})\n");
        }
        DeclaredParams::Declared(specs) if specs.is_empty() => {
            return "Params: none\n".to_string();
        }
        DeclaredParams::Declared(specs) => specs,
    };
    let dash = |v: &Option<cerulion_core::graph::params::ParamValue>| match v {
        Some(v) => v.to_string(),
        None => "-".to_string(),
    };
    let rows: Vec<[String; 6]> = specs
        .iter()
        .map(|s| {
            [
                s.name.clone(),
                s.ty.to_string(),
                s.default.to_string(),
                dash(&s.min),
                dash(&s.max),
                s.description.clone().unwrap_or_default(),
            ]
        })
        .collect();
    let header = ["NAME", "TYPE", "DEFAULT", "MIN", "MAX", "DESCRIPTION"];
    let mut widths: Vec<usize> = header.iter().map(|h| h.len()).collect();
    for row in &rows {
        for (w, cell) in widths.iter_mut().zip(row.iter()) {
            *w = (*w).max(cell.len());
        }
    }
    let line = |cells: [&str; 6]| -> String {
        let mut out = String::from(" ");
        for (i, (cell, w)) in cells.iter().zip(widths.iter()).enumerate() {
            out.push(' ');
            if i + 1 == cells.len() {
                out.push_str(cell);
            } else {
                out.push_str(&format!("{cell:<w$}"));
            }
        }
        out.trim_end().to_string()
    };
    let mut text = String::from("Params:\n");
    text.push_str(&line(header));
    text.push('\n');
    for row in &rows {
        text.push_str(&line([
            &row[0], &row[1], &row[2], &row[3], &row[4], &row[5],
        ]));
        text.push('\n');
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::graph::params::{ParamType, ParamValue};

    #[test]
    fn the_table_lists_each_spec_with_its_bounds_and_description() {
        let declared = DeclaredParams::Declared(vec![
            ParamSpec {
                name: "gain".into(),
                ty: ParamType::F64,
                default: ParamValue::Float(1.0),
                min: Some(ParamValue::Float(0.0)),
                max: Some(ParamValue::Float(10.0)),
                description: Some("multiplier written to cmd.x".into()),
            },
            ParamSpec {
                name: "frame_id".into(),
                ty: ParamType::String,
                default: ParamValue::Str("base_link".into()),
                min: None,
                max: None,
                description: None,
            },
        ]);
        assert_eq!(
            render_params("cam", &declared),
            "Params:\n\
             \x20 NAME     TYPE   DEFAULT     MIN MAX  DESCRIPTION\n\
             \x20 gain     f64    1.0         0.0 10.0 multiplier written to cmd.x\n\
             \x20 frame_id String \"base_link\" -   -\n"
        );
    }

    #[test]
    fn a_type_without_params_and_an_unbuilt_type_each_say_so() {
        assert_eq!(
            render_params("cam", &DeclaredParams::Declared(vec![])),
            "Params: none\n"
        );
        assert_eq!(
            render_params("cam", &DeclaredParams::NotBuilt),
            "Params: not read (no built library; run `cerulion node build cam` and ask again)\n"
        );
        assert_eq!(
            render_params("cam", &DeclaredParams::Unreadable("boom".into())),
            "Params: not read (the node library could not be read: boom)\n"
        );
    }
}

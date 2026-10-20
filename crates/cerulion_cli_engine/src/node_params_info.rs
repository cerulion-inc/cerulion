// SPDX-License-Identifier: AGPL-3.0-only
//! The params table of `cerulion node info`: the `#[param]` fields a node
//! type declares, read from the node info of its BUILT library (the same
//! document `graph validate` and `graph run` check a graph's `params:`
//! against), so the table shows exactly what a graph may set.
//!
//! The library is read through the caller's [`NodeInspector`], the one seam
//! every reader of a node library goes through, so `node info` and
//! `graph validate` cannot drift in how they read it, and a standing process
//! that serves this table can read in a child that dies alone.

use std::path::Path;

use cerulion_core::graph::params::ParamSpec;

use crate::node_inspector::NodeInspector;

/// What `node info` can say about a type's params.
#[derive(Debug, Clone, PartialEq)]
pub enum DeclaredParams {
    /// No built library for the type: the specs live in the library's node
    /// info, so there is nothing to read until `cerulion node build <type>`.
    NotBuilt,
    /// The library exists but its node info could not be read.
    Unreadable(String),
    /// The type name is not a node type name (`[A-Za-z_][A-Za-z0-9_]*`), so
    /// no library was looked up: a path component in it (`../x`) would
    /// otherwise resolve, and LOAD, a library outside the workspace target.
    InvalidName(String),
    /// The declared specs, in declaration order (empty: the type declares
    /// none). `stale` is the library's staleness verdict: a source file under
    /// `nodes/<type>/` is newer than the built library, so the table may not
    /// show the `#[param]` lines as they are written now.
    Declared { specs: Vec<ParamSpec>, stale: bool },
}

/// Read the type's declared params from its built library, preferring the
/// debug build and falling back to release, the same resolution `graph run`
/// uses without `--release`.
pub fn declared_params(
    workspace_root: &Path,
    node_type: &str,
    inspector: &dyn NodeInspector,
) -> DeclaredParams {
    // Checked HERE, at the engine boundary that turns the name into a
    // library path and loads it, so no caller can hand it a path.
    if let Err(e) = crate::utils::validate_node_type(node_type) {
        return DeclaredParams::InvalidName(e.to_string());
    }
    let Ok(lib) = crate::graph_cmd::find_cdylib(workspace_root, node_type, false) else {
        return DeclaredParams::NotBuilt;
    };
    match inspector.inspect(&lib.path, node_type) {
        Ok(info) => DeclaredParams::Declared {
            specs: info.params().to_vec(),
            stale: lib.stale,
        },
        Err(e) => DeclaredParams::Unreadable(e.to_string()),
    }
}

/// The `Params:` section of `node info`, one row per declared param with
/// its type, default, bounds and description, and a note when the library
/// is older than its source. Pure: the caller prints it.
pub fn render_params(node_type: &str, declared: &DeclaredParams) -> String {
    let (specs, stale) = match declared {
        DeclaredParams::NotBuilt => {
            return format!(
                "Params: not read (no built library; run `cerulion node build {node_type}` \
                 and ask again)\n"
            );
        }
        DeclaredParams::Unreadable(reason) => {
            return format!("Params: not read (the node library could not be read: {reason})\n");
        }
        DeclaredParams::InvalidName(reason) => {
            return format!("Params: not read ({reason})\n");
        }
        DeclaredParams::Declared { specs, stale } => (specs, *stale),
    };
    let mut text = if specs.is_empty() {
        String::from("Params: none\n")
    } else {
        render_table(specs)
    };
    if stale {
        text.push_str(&format!(
            "  (the node library is older than its source; run `cerulion node build \
             {node_type}` and ask again for the current table)\n"
        ));
    }
    text
}

/// `text` as one table cell: every control character (a newline, a tab, an
/// escape sequence's ESC) is written as its Rust escape (`\n`, `\t`,
/// `\u{1b}`), so a library's strings can neither split a row nor reach the
/// terminal raw. Everything else is kept as written.
fn one_line(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        if c.is_control() {
            out.extend(c.escape_default());
        } else {
            out.push(c);
        }
    }
    out
}

fn render_table(specs: &[ParamSpec]) -> String {
    let dash = |v: &Option<cerulion_core::graph::params::ParamValue>| match v {
        Some(v) => v.to_string(),
        None => "-".to_string(),
    };
    let rows: Vec<[String; 6]> = specs
        .iter()
        .map(|s| {
            [
                one_line(&s.name),
                s.ty.to_string(),
                s.default.to_string(),
                dash(&s.min),
                dash(&s.max),
                one_line(s.description.as_deref().unwrap_or_default()),
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

    fn two_specs() -> Vec<ParamSpec> {
        vec![
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
        ]
    }

    const TABLE: &str = "Params:\n\
         \x20 NAME     TYPE   DEFAULT     MIN MAX  DESCRIPTION\n\
         \x20 gain     f64    1.0         0.0 10.0 multiplier written to cmd.x\n\
         \x20 frame_id String \"base_link\" -   -\n";

    #[test]
    fn the_table_lists_each_spec_with_its_bounds_and_description() {
        let declared = DeclaredParams::Declared {
            specs: two_specs(),
            stale: false,
        };
        assert_eq!(render_params("cam", &declared), TABLE);
    }

    #[test]
    fn a_stale_library_gets_the_rebuild_note_under_its_table() {
        let declared = DeclaredParams::Declared {
            specs: two_specs(),
            stale: true,
        };
        assert_eq!(
            render_params("cam", &declared),
            format!(
                "{TABLE}  (the node library is older than its source; run `cerulion node build \
                 cam` and ask again for the current table)\n"
            )
        );
        let none = DeclaredParams::Declared {
            specs: vec![],
            stale: true,
        };
        assert_eq!(
            render_params("cam", &none),
            "Params: none\n  (the node library is older than its source; run `cerulion node \
             build cam` and ask again for the current table)\n"
        );
    }

    #[test]
    fn a_description_with_control_characters_stays_on_its_row() {
        let mut specs = two_specs();
        specs[0].description = Some("line one\nline two\tend \u{1b}[31mred".into());
        let table = render_params(
            "cam",
            &DeclaredParams::Declared {
                specs,
                stale: false,
            },
        );
        assert_eq!(
            table,
            "Params:\n\
             \x20 NAME     TYPE   DEFAULT     MIN MAX  DESCRIPTION\n\
             \x20 gain     f64    1.0         0.0 10.0 line one\\nline two\\tend \\u{1b}[31mred\n\
             \x20 frame_id String \"base_link\" -   -\n"
        );
        assert_eq!(table.lines().count(), 4, "one header, two rows, no split");
    }

    /// The name becomes a library path that is then LOADED, so a name with a
    /// path in it is refused before any lookup: the inspector is never asked.
    #[test]
    fn a_node_type_with_a_path_in_it_is_refused_before_any_library_is_read() {
        struct NeverAsked;
        impl NodeInspector for NeverAsked {
            fn inspect(
                &self,
                cdylib: &Path,
                _label: &str,
            ) -> crate::error::CliResult<cerulion_core::graph::NodeInfo> {
                panic!("inspected {} for an invalid name", cdylib.display());
            }
        }
        let root = Path::new("/nonexistent-workspace");
        for name in ["../../../../tmp/pwn", "a/b", "", "1cam"] {
            let declared = declared_params(root, name, &NeverAsked);
            let DeclaredParams::InvalidName(reason) = &declared else {
                panic!("{name:?} must be refused by name, got {declared:?}");
            };
            assert_eq!(
                render_params(name, &declared),
                format!("Params: not read ({reason})\n")
            );
        }
        assert_eq!(
            declared_params(root, "../../../../tmp/pwn", &NeverAsked),
            DeclaredParams::InvalidName(
                "node type name '../../../../tmp/pwn' must contain only ASCII alphanumeric \
                 characters and underscores"
                    .into()
            )
        );
    }

    #[test]
    fn a_type_without_params_and_an_unbuilt_type_each_say_so() {
        assert_eq!(
            render_params(
                "cam",
                &DeclaredParams::Declared {
                    specs: vec![],
                    stale: false,
                }
            ),
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

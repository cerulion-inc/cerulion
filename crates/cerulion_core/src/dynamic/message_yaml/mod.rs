// SPDX-License-Identifier: AGPL-3.0-only
//! The message <-> YAML codec: one wire frame as one YAML mapping, and back.
//!
//! [`message_to_yaml`] renders any frame a [`crate::codegen::FrameWalker`] knows as YAML;
//! [`yaml_to_message`] parses that YAML (or YAML a person typed) back into a
//! byte-exact wire frame. `topic echo` prints the first; the second is the
//! reader for every command that takes a message as text.
//!
//! # The YAML form (a contract)
//!
//! - A message is a block mapping, one key per field, in the schema's
//!   DECLARATION order (the order of the `.msg` / schema YAML). A message
//!   with no fields is `{}`.
//! - Every field is required on input; an unknown key is an error.
//! - Integers are decimal. Floats use the shortest form that reads back to
//!   the same value (`1.0`, `0.1`, `1e-7`), plus `.nan`, `.inf`, `-.inf`;
//!   an integer is accepted for a float field. Every NaN is written `.nan`
//!   and reads back as the quiet NaN (`0x7FF8000000000000`, or `0x7FC00000`
//!   for `float32`): a NaN's sign and payload bits are the one part of a
//!   frame the YAML form does not keep, so a frame holding any other NaN
//!   does not round-trip byte for byte.
//! - Strings are always double quoted, with YAML escapes for control and
//!   line-break characters, so `"true"` and `"1"` stay strings.
//! - Arrays of scalars and strings are flow sequences (`[1, 2, 3]`); arrays
//!   of messages are block sequences (`- x: 1.0`). Byte arrays (`uint8[]`,
//!   `bytes`) are sequences of integers.
//! - `builtin_interfaces/Time` and `Duration` are plain messages:
//!   `stamp: {sec: 1, nanosec: 2}` in block form.
//! - Constants are not part of the form: a field that takes a constant is
//!   written as its number.
//! - An array cut short for display ends with `!truncated <n>` (the number
//!   of elements left out). Input that carries the tag is refused.
//! - Anchors, aliases and tags (other than the refusal above) are refused.
//! - Output is capped at [`MAX_YAML_BYTES`]; a message whose YAML form is
//!   longer is refused at the field being written.
//! - Output cut at a line budget ([`YamlEncodeOptions::max_lines`]) ends
//!   with a column-0 `!truncated` line, which no YAML reader accepts.
//!
//! Every input error names the full field path (`pose.position.x`,
//! `poses[2].position.x`, `(message)` for the root) and the expected type
//! or bound.
//!
//! Neither direction is on a publish path: both allocate (`String`, `Vec`),
//! and both run once per command in a CLI, never per frame inside a node.

mod decode;
mod encode;
mod scan;

pub use decode::yaml_to_message;
pub use encode::{message_to_yaml, message_to_yaml_with, YamlEncodeOptions};

use crate::codegen::FieldType;

/// The tag that marks an array cut short for display (`!truncated 57472`).
pub const YAML_TRUNCATED_TAG: &str = "!truncated";

/// How deep messages may nest, on either direction, before the codec
/// refuses. Real ROS 2 types nest well under 10 levels; the bound keeps a
/// self-referencing schema from recursing without end.
pub const MAX_YAML_DEPTH: usize = 64;

/// How many arrays of variable messages a nonempty array of strings or
/// messages may sit inside. The frame walker decodes no deeper, so
/// [`yaml_to_message`] refuses what [`message_to_yaml`] could not print.
pub const MAX_YAML_ARRAY_DEPTH: usize =
    crate::codegen::frame_walker::MAX_NESTED_ARRAY_DEPTH as usize;

/// The most YAML [`message_to_yaml`] writes for one frame before it
/// refuses (256 MiB). An unset nested field expands to its schema's zero
/// values, so a few bytes of frame can name far more output than any
/// terminal or file wants; the cap turns that into an error before the
/// output fills memory.
pub const MAX_YAML_BYTES: usize = 256 << 20;

/// The display form of a field path: `(message)` for the root.
fn shown(path: &str) -> String {
    if path.is_empty() {
        "(message)".to_string()
    } else {
        path.to_string()
    }
}

/// `path` + `.name` (no dot at the root).
fn push_field(path: &mut String, name: &str) {
    if !path.is_empty() {
        path.push('.');
    }
    path.push_str(name);
}

/// `path` + `[index]`.
fn push_index(path: &mut String, index: usize) {
    use std::fmt::Write as _;
    let _ = write!(path, "[{index}]");
}

/// The user-facing name of a field type (`float64`, `uint8[]`,
/// `float32[9]`, `string_fixed[16]`, `geometry_msgs/Point`).
fn type_name(ft: &FieldType) -> String {
    ft.canonical_str()
}

#[cfg(test)]
mod tests;

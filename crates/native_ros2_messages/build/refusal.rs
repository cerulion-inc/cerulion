// SPDX-License-Identifier: AGPL-3.0-only
//! Whether the generator refused a schema in the source it emitted.
//!
//! The build script pulls this file in by path, and so does
//! `tests/generator_refusal_test.rs`, so the check the build runs is the one
//! the test pins.

/// True when `generated` invokes `compile_error!` as an item.
///
/// The generator reports some refusals (a field name that collides with a
/// generated accessor, the reserved `__cer` prefix) as a `compile_error!` call
/// on a line of its own. A schema or field description is emitted as a `///`
/// doc comment, so a description that only mentions the macro is a comment
/// line, not a call, and does not count. A description line break that would
/// start a real call at the head of a line does count: that source would not
/// compile either.
pub fn generator_refused(generated: &str) -> bool {
    generated
        .lines()
        .any(|line| line.trim_start().starts_with("compile_error!("))
}

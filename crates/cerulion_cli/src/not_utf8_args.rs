// SPDX-License-Identifier: AGPL-3.0-only
//! The refusal `cerulion` prints for a command-line argument that is not valid
//! UTF-8 where a verb takes text.
//!
//! clap takes such an argument for a path field (`PathBuf`, `OsString`) and
//! refuses it for a text field with one line that names no argument
//! ([`clap::error::ErrorKind::InvalidUtf8`]). `main` prints [`refusal`] in
//! place of that line. clap does not say which argument it refused, so the
//! refusal names every argument that is not valid UTF-8, a path clap accepted
//! among them.

use std::ffi::OsString;
use std::fmt::Write as _;

/// The line after the per-argument lines: what the user does next.
const REMEDY: &str =
    "Only a path may hold bytes that are not UTF-8: pass every other argument as valid UTF-8 text.";

/// The refusal for `args`, the command line after the program name: one
/// `Error:` line per argument that is not valid UTF-8, then [`REMEDY`]. An
/// argument's position counts from 1, and its value is printed as `{:?}`
/// prints an `OsStr`: in double quotes, valid text as it is, every other byte
/// as `\xNN`. `None` when every argument is valid UTF-8.
pub(crate) fn refusal(args: &[OsString]) -> Option<String> {
    let mut text = String::new();
    for (index, arg) in args.iter().enumerate() {
        if arg.to_str().is_none() {
            let position = index + 1;
            // Writing into a `String` cannot fail.
            let _ = writeln!(
                text,
                "Error: argument {position} is not valid UTF-8: {arg:?}"
            );
        }
    }
    if text.is_empty() {
        return None;
    }
    text.push_str(REMEDY);
    text.push('\n');
    Some(text)
}

#[cfg(all(test, unix))]
mod tests {
    use super::refusal;
    use std::ffi::OsString;
    use std::os::unix::ffi::OsStringExt as _;

    fn args(list: &[&[u8]]) -> Vec<OsString> {
        list.iter()
            .map(|a| OsString::from_vec(a.to_vec()))
            .collect()
    }

    #[test]
    fn one_argument_that_is_not_utf8_is_named_by_its_position() {
        assert_eq!(
            refusal(&args(&[b"bag", b"play", b"caf\xE9.mcap"])).as_deref(),
            Some(
                "Error: argument 3 is not valid UTF-8: \"caf\\xE9.mcap\"\n\
                 Only a path may hold bytes that are not UTF-8: pass every other argument as \
                 valid UTF-8 text.\n"
            )
        );
    }

    #[test]
    fn every_argument_that_is_not_utf8_gets_its_own_line_in_order() {
        assert_eq!(
            refusal(&args(&[b"\xFFa", b"topic", b"b\xC3", b"ok"])).as_deref(),
            Some(
                "Error: argument 1 is not valid UTF-8: \"\\xFFa\"\n\
                 Error: argument 3 is not valid UTF-8: \"b\\xC3\"\n\
                 Only a path may hold bytes that are not UTF-8: pass every other argument as \
                 valid UTF-8 text.\n"
            )
        );
    }

    /// The valid part of an argument prints as `{:?}` prints text: a quote and
    /// a backslash are escaped, a multi-byte character is kept.
    #[test]
    fn the_valid_part_of_an_argument_is_escaped_as_debug_prints_it() {
        let mut mixed = b"\"\xE9".to_vec();
        mixed.extend_from_slice("é\\".as_bytes());
        assert_eq!(
            refusal(&args(&[&mixed])).as_deref(),
            Some(
                "Error: argument 1 is not valid UTF-8: \"\\\"\\xE9é\\\\\"\n\
                 Only a path may hold bytes that are not UTF-8: pass every other argument as \
                 valid UTF-8 text.\n"
            )
        );
    }

    #[test]
    fn a_command_line_of_valid_utf8_has_no_refusal() {
        assert_eq!(refusal(&args(&[])), None);
        assert_eq!(refusal(&args(&[b"bag", "café".as_bytes(), b""])), None);
    }

    #[test]
    fn two_runs_over_the_same_arguments_are_identical() {
        let input = args(&[b"x\xFE", b"y"]);
        assert_eq!(refusal(&input), refusal(&input));
    }
}

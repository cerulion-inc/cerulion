// SPDX-License-Identifier: AGPL-3.0-only
//! The refusal `cerulion` prints for a command-line argument that is not valid
//! UTF-8 anywhere but a path or the `ros2 run` / `ros2 launch` pass-through.
//!
//! clap takes such an argument for a path field (`PathBuf`, `OsString`). It
//! refuses one in a text field with a line that names no argument
//! ([`ErrorKind::InvalidUtf8`]). It also refuses one in the place of a verb or
//! a flag name, past the last argument a verb takes, or as a value given to a
//! flag that takes none; it reports each as an unrecognized verb, an
//! unexpected argument or an unexpected value, quoting it with U+FFFD in place
//! of each byte that is not UTF-8. `main` prints [`refusal_for_parse_error`]
//! in place of either. clap does not say the position of the argument it
//! refused, so the refusal names every argument that is not valid UTF-8, a
//! path clap accepted among them.

use std::ffi::OsString;
use std::fmt::Write as _;

use clap::error::ErrorKind;

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

/// What `main` prints in place of clap's parse error `e`, for `args`, the
/// command line after the program name: [`refusal`], when `e` is a usage
/// error that is [`ErrorKind::InvalidUtf8`] or whose text holds U+FFFD, which
/// clap's quote of an argument shows for each byte that is not UTF-8. `None`
/// keeps clap's error: `--help` and `--version` output, any other usage error,
/// and a usage error with U+FFFD in its text while an argument that is valid
/// UTF-8 holds U+FFFD as typed, since clap's quote of that argument reads the
/// same as a quote of bytes.
pub(crate) fn refusal_for_parse_error(e: &clap::Error, args: &[OsString]) -> Option<String> {
    if !e.use_stderr() {
        return None;
    }
    let typed_replacement = args
        .iter()
        .filter_map(|arg| arg.to_str())
        .any(|arg| arg.contains(char::REPLACEMENT_CHARACTER));
    let quotes_bytes =
        !typed_replacement && e.render().to_string().contains(char::REPLACEMENT_CHARACTER);
    if e.kind() != ErrorKind::InvalidUtf8 && !quotes_bytes {
        return None;
    }
    refusal(args)
}

#[cfg(all(test, unix))]
mod tests {
    use super::{refusal, refusal_for_parse_error};
    use clap::error::ErrorKind;
    use std::ffi::OsString;
    use std::os::unix::ffi::OsStringExt as _;

    fn args(list: &[&[u8]]) -> Vec<OsString> {
        list.iter()
            .map(|a| OsString::from_vec(a.to_vec()))
            .collect()
    }

    /// The error `cerulion`'s own parser returns for `list`, the command line
    /// after the program name, and that command line.
    fn parse_error(list: &[&[u8]]) -> (clap::Error, Vec<OsString>) {
        let args = args(list);
        let argv = std::iter::once(OsString::from("cerulion")).chain(args.iter().cloned());
        let error = crate::families::cli_command()
            .try_get_matches_from(argv)
            .expect_err("the command line must not parse");
        (error, args)
    }

    fn refusal_line(position: usize, value: &str) -> String {
        format!(
            "Error: argument {position} is not valid UTF-8: {value}\n\
             Only a path may hold bytes that are not UTF-8: pass every other argument as \
             valid UTF-8 text.\n"
        )
    }

    #[test]
    fn a_text_field_that_is_not_utf8_gets_the_refusal() {
        let (error, args) = parse_error(&[b"topic", b"echo", b"/caf\xE9"]);
        assert_eq!(error.kind(), ErrorKind::InvalidUtf8);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(3, "\"/caf\\xE9\""))
        );
    }

    /// A verb name, at the top and under a verb, that is not valid UTF-8 is an
    /// unrecognized verb to clap; the refusal replaces that error.
    #[test]
    fn a_verb_name_that_is_not_utf8_gets_the_refusal() {
        let (error, args) = parse_error(&[b"\xFFbag"]);
        assert_eq!(error.kind(), ErrorKind::InvalidSubcommand);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(1, "\"\\xFFbag\""))
        );
        let (error, args) = parse_error(&[b"bag", b"pl\xE4y", b"x.mcap"]);
        assert_eq!(error.kind(), ErrorKind::InvalidSubcommand);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(2, "\"pl\\xE4y\""))
        );
        let (error, args) = parse_error(&[b"help", b"\xFF"]);
        assert_eq!(error.kind(), ErrorKind::InvalidSubcommand);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(2, "\"\\xFF\""))
        );
    }

    /// A value that is not valid UTF-8 attached to a flag that takes none is
    /// an extra value to clap; the refusal replaces that error.
    #[test]
    fn a_value_for_a_flag_that_takes_none_gets_the_refusal() {
        let (error, args) = parse_error(&[b"--verbose=\xFF", b"bag"]);
        assert_eq!(error.kind(), ErrorKind::TooManyValues);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(1, "\"--verbose=\\xFF\""))
        );
    }

    /// A long or short flag name that is not valid UTF-8 is an unexpected
    /// argument to clap; the refusal replaces that error.
    #[test]
    fn a_flag_name_that_is_not_utf8_gets_the_refusal() {
        let (error, args) = parse_error(&[b"bag", b"info", b"--j\xE9son", b"x.mcap"]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(3, "\"--j\\xE9son\""))
        );
        let (error, args) = parse_error(&[b"topic", b"list", b"-\xFF"]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(3, "\"-\\xFF\""))
        );
    }

    /// A usage error about another argument keeps clap's text, though a path
    /// on the same command line is not valid UTF-8.
    #[test]
    fn another_usage_error_keeps_clap_text_beside_a_path_that_is_not_utf8() {
        let (error, args) = parse_error(&[b"bag", b"info", b"caf\xE9.mcap", b"--bogus"]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(refusal_for_parse_error(&error, &args), None);
    }

    #[test]
    fn help_output_is_kept_beside_a_path_that_is_not_utf8() {
        let (error, args) = parse_error(&[b"bag", b"info", b"caf\xE9.mcap", b"--help"]);
        assert_eq!(error.kind(), ErrorKind::DisplayHelp);
        assert_eq!(refusal_for_parse_error(&error, &args), None);
    }

    /// A flag name refused beside a path that is not valid UTF-8: both
    /// arguments are named, in order, the path clap accepted among them.
    #[test]
    fn a_flag_name_refused_beside_a_path_names_both() {
        let (error, args) = parse_error(&[b"bag", b"info", b"caf\xE9.mcap", b"--j\xE9son"]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(
            refusal_for_parse_error(&error, &args).as_deref(),
            Some(
                "Error: argument 3 is not valid UTF-8: \"caf\\xE9.mcap\"\n\
                 Error: argument 4 is not valid UTF-8: \"--j\\xE9son\"\n\
                 Only a path may hold bytes that are not UTF-8: pass every other argument as \
                 valid UTF-8 text.\n"
            )
        );
    }

    /// An argument past the last one a verb takes is an unexpected argument
    /// to clap; the refusal replaces that error.
    #[test]
    fn an_extra_argument_that_is_not_utf8_gets_the_refusal() {
        let (error, args) = parse_error(&[b"bag", b"info", b"x.mcap", b"caf\xE9"]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(4, "\"caf\\xE9\""))
        );
    }

    /// U+FFFD typed as valid UTF-8 is quoted by clap as it is. Beside a path
    /// that is not valid UTF-8, clap's text stays: the refusal would name the
    /// path in place of the argument clap refused. With every argument valid
    /// UTF-8 there is no refusal to print either.
    #[test]
    fn a_typed_replacement_character_keeps_clap_text() {
        let (error, args) = parse_error(&["\u{FFFD}bag".as_bytes()]);
        assert_eq!(error.kind(), ErrorKind::InvalidSubcommand);
        assert_eq!(refusal_for_parse_error(&error, &args), None);
        let (error, args) = parse_error(&[
            b"bag",
            b"info",
            b"caf\xE9.mcap",
            "caf\u{FFFD}.mcap".as_bytes(),
        ]);
        assert_eq!(error.kind(), ErrorKind::UnknownArgument);
        assert_eq!(refusal_for_parse_error(&error, &args), None);
    }

    /// clap's line for a text field quotes nothing, so beside a typed U+FFFD
    /// it still gets the refusal, which names only the argument that is not
    /// valid UTF-8.
    #[test]
    fn a_text_field_beside_a_typed_replacement_character_gets_the_refusal() {
        let (error, args) = parse_error(&[
            b"bag",
            b"play",
            "caf\u{FFFD}.mcap".as_bytes(),
            b"--topics",
            b"\xFF",
        ]);
        assert_eq!(error.kind(), ErrorKind::InvalidUtf8);
        assert_eq!(
            refusal_for_parse_error(&error, &args),
            Some(refusal_line(5, "\"\\xFF\""))
        );
    }

    /// Help output is kept even when its own text holds U+FFFD: the decision
    /// reads only usage errors.
    #[test]
    fn help_output_that_holds_a_replacement_character_is_kept() {
        let command = clap::Command::new("t")
            .about("\u{FFFD}")
            .arg(clap::Arg::new("path").value_parser(clap::value_parser!(std::path::PathBuf)));
        let args = args(&[b"caf\xE9", b"--help"]);
        let argv = std::iter::once(OsString::from("t")).chain(args.iter().cloned());
        let error = command
            .try_get_matches_from(argv)
            .expect_err("--help ends the parse");
        assert_eq!(error.kind(), ErrorKind::DisplayHelp);
        assert!(error
            .render()
            .to_string()
            .contains(char::REPLACEMENT_CHARACTER));
        assert_eq!(refusal_for_parse_error(&error, &args), None);
    }

    #[test]
    fn two_parses_of_the_same_command_line_give_the_same_refusal() {
        let expected = Some(refusal_line(2, "\"--\\xFF\""));
        for _ in 0..2 {
            let (error, args) = parse_error(&[b"bag", b"--\xFF"]);
            assert_eq!(refusal_for_parse_error(&error, &args), expected);
        }
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
        let expected = Some(refusal_line(1, "\"x\\xFE\""));
        assert_eq!(refusal(&input), expected);
        assert_eq!(refusal(&input), expected);
    }
}

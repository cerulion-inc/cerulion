// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-written oracles for the byte-mismatch value diff: every expected
//! string below is written out by hand from the frame bytes, never taken
//! from a run of the code under test.
//!
//! Frames are built with the YAML codec (`yaml_to_message`) or byte by byte,
//! and a byte a test patches is named with its hand-computed offset.

use std::sync::OnceLock;

use cerulion_core::dynamic::{yaml_to_message, FieldDef, FrameWalker, NamedValue, WireHeader};
use cerulion_core::wire::StepTag;

use super::*;
use crate::replay_engine::{ByteDifference, DifferenceKind, DifferenceValue};

/// The built-in set plus `extra` workspace schema YAML.
fn walker_with(extra: &str) -> FrameWalker {
    let mut schemas = crate::schema_cmd::parse_builtin_schemas();
    schemas.extend(crate::schema_cmd::parse_message_schemas(extra).expect("fixture schemas parse"));
    FrameWalker::new(schemas).0
}

/// The built-in set alone, built once for the binary.
fn builtin() -> &'static FrameWalker {
    static WALKER: OnceLock<FrameWalker> = OnceLock::new();
    WALKER.get_or_init(|| crate::topic_cmd::local_walker_from_workspace(None))
}

/// `yaml` as a wire frame of `schema` (timestamp 7, sequence 0, untagged).
fn frame(walker: &FrameWalker, schema: &str, yaml: &str) -> Vec<u8> {
    yaml_to_message(walker, schema, yaml.as_bytes(), 7).expect("fixture YAML encodes")
}

/// A frame built byte by byte: the header (sequence 0, timestamp 7,
/// untagged) and `payload`.
fn raw_frame(schema_hash: u64, fixed_size: usize, table_count: u32, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed_size) as u32,
        offset_table_count: table_count,
        sequence: 0,
        timestamp_ns: 7,
        step_tag: StepTag::UNTAGGED,
    };
    let mut out = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut out);
    out.extend_from_slice(payload);
    out
}

fn hash_of(walker: &FrameWalker, name: &str) -> u64 {
    walker
        .schema_hash_for(name)
        .expect("fixture schema is in the set")
}

/// The raw diff's first differing byte (the shorter length on a prefix).
fn first_diff(recorded: &[u8], replayed: &[u8]) -> usize {
    recorded
        .iter()
        .zip(replayed)
        .position(|(a, b)| a != b)
        .unwrap_or(recorded.len().min(replayed.len()))
}

fn locate(walker: &FrameWalker, recorded: &[u8], replayed: &[u8]) -> FrameDifference {
    locate_difference(
        || walker,
        recorded,
        replayed,
        first_diff(recorded, replayed),
        &[],
        None,
    )
}

/// The detail line for frame 0 of `/t`.
fn line(walker: &FrameWalker, recorded: &[u8], replayed: &[u8]) -> String {
    let verdict = ByteVerdict {
        index: 0,
        first_diff_byte_offset: first_diff(recorded, replayed),
        recorded_len: recorded.len(),
        replayed_len: replayed.len(),
        difference: locate(walker, recorded, replayed),
    };
    byte_mismatch_detail("/t", "", &verdict)
}

fn put_u32(frame: &mut [u8], at: usize, v: u32) {
    frame[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

fn put_u64(frame: &mut [u8], at: usize, v: u64) {
    frame[at..at + 8].copy_from_slice(&v.to_le_bytes());
}

fn shown(text: &str, json: DifferenceValue) -> Shown {
    Shown {
        text: text.to_string(),
        json,
    }
}

// ---------------------------------------------------------------------------
// Happy path
// ---------------------------------------------------------------------------

/// The quickstart case: `geometry_msgs/Vector3` with x changed from 0.3 to
/// 0.25, which the raw diff reports at byte 40 (the first payload byte).
#[test]
fn a_vector3_x_change_names_x_and_both_values() {
    let w = builtin();
    let rec = frame(w, "geometry_msgs/Vector3", "x: 0.3\ny: 0.0\nz: 0.0\n");
    let rep = frame(w, "geometry_msgs/Vector3", "x: 0.25\ny: 0.0\nz: 0.0\n");
    assert_eq!(rec.len(), 64);
    assert_eq!(
        rec[40..48],
        [0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xd3, 0x3f]
    );
    assert_eq!(rep[40..48], [0, 0, 0, 0, 0, 0, 0xd0, 0x3f]);
    assert_eq!(first_diff(&rec, &rep), 40);
    let verdict = ByteVerdict {
        index: 99,
        first_diff_byte_offset: 40,
        recorded_len: 64,
        replayed_len: 64,
        difference: locate(w, &rec, &rep),
    };
    assert_eq!(
        byte_mismatch_detail(
            "/obstacle_avoidance/safety_controller/linear_velocity",
            "",
            &verdict
        ),
        "frame 99 on '/obstacle_avoidance/safety_controller/linear_velocity' differs at x: \
         recorded 0.3, replayed 0.25 (recorded 64 B, replayed 64 B)"
    );
    assert_eq!(
        report_entry(&verdict.difference),
        ByteDifference {
            kind: DifferenceKind::Field,
            path: Some("x".to_string()),
            recorded: Some(DifferenceValue::Float64(0.3)),
            replayed: Some(DifferenceValue::Float64(0.25)),
            field_byte: None,
            field_char: None,
            byte_offset: None,
            region: None,
            reason: None,
            more: 0,
        }
    );
}

/// The engine fixture's flipped byte: 103.0 is `00 00 00 00 00 c0 59 40`;
/// byte 0 XOR 0xFF reads back as 103.00000000000362.
#[test]
fn a_flipped_low_byte_prints_the_shortest_float_that_reads_back() {
    let w = builtin();
    let mut rec = frame(w, "geometry_msgs/Vector3", "x: 103.0\ny: 0.0\nz: 0.0\n");
    let rep = rec.clone();
    rec[40] ^= 0xff;
    assert_eq!(
        line(w, &rec, &rep),
        "frame 0 on '/t' differs at x: recorded 103.00000000000362, replayed 103.0 (recorded \
         64 B, replayed 64 B)"
    );
}

/// A nested message recurses: the path is `linear.x`, and the other
/// differing value (`angular.z`) is counted, not named.
#[test]
fn a_twist_names_the_nested_path_and_counts_the_rest() {
    let w = builtin();
    let twist = |lx: &str, az: &str| {
        frame(
            w,
            "geometry_msgs/Twist",
            &format!("linear: {{x: {lx}, y: 0.0, z: 0.0}}\nangular: {{x: 0.0, y: 0.0, z: {az}}}\n"),
        )
    };
    let (rec, rep) = (twist("0.5", "0.0"), twist("0.25", "1.0"));
    assert_eq!(
        line(w, &rec, &rep),
        "frame 0 on '/t' differs at linear.x: recorded 0.5, replayed 0.25; 1 more value(s) \
         differ (recorded 88 B, replayed 88 B)"
    );
}

/// The first value named is the first in DECLARATION order. `note` is a
/// variable field, so its bytes come after the fixed `v`: byte order would
/// name `v`.
#[test]
fn declaration_order_beats_byte_order() {
    let w =
        walker_with("schemas:\n  Note:\n    fields:\n      string note: {}\n      float64 v: {}\n");
    let rec = frame(&w, "Note", "note: \"map\"\nv: 1.0\n");
    let rep = frame(&w, "Note", "note: \"odom\"\nv: 2.0\n");
    // `v` is the whole fixed section (frame bytes 40..48), ahead of note's
    // bytes at 56.
    assert_ne!(rec[40..48], rep[40..48]);
    assert_eq!(&rec[56..], b"map");
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at note: recorded \"map\", replayed \"odom\"; 1 more value(s) \
         differ (recorded 59 B, replayed 60 B)"
    );
}

// ---------------------------------------------------------------------------
// Value spelling, one width at a time
// ---------------------------------------------------------------------------

const WIDTHS: &str = "\
schemas:
  Widths:
    fields:
      int8 a: {}
      uint8 b: {}
      int16 c: {}
      uint16 d: {}
      int32 e: {}
      uint32 f: {}
      int64 g: {}
      uint64 h: {}
      bool i: {}
      float32 j: {}
      float64 k: {}
      string s: {}
";

/// The `Widths` frame with every field zero except the one `set` names.
fn widths(w: &FrameWalker, set: &str) -> Vec<u8> {
    let mut fields = vec![
        ("a", "0"),
        ("b", "0"),
        ("c", "0"),
        ("d", "0"),
        ("e", "0"),
        ("f", "0"),
        ("g", "0"),
        ("h", "0"),
        ("i", "false"),
        ("j", "0.0"),
        ("k", "0.0"),
        ("s", "\"\""),
    ];
    let mut text = String::new();
    for (name, value) in &mut fields {
        let given = set
            .split_once(": ")
            .filter(|(n, _)| n == name)
            .map(|(_, v)| v);
        text.push_str(&format!("{name}: {}\n", given.unwrap_or(value)));
    }
    frame(w, "Widths", &text)
}

/// The first value of a pair of `Widths` frames.
fn first_value(w: &FrameWalker, rec: &[u8], rep: &[u8]) -> ValueDifference {
    match locate(w, rec, rep) {
        FrameDifference::Field { value, more: 0 } => value,
        other => panic!("expected one differing field, got {other:?}"),
    }
}

/// Every integer width prints in exact decimal, the 64-bit extremes too
/// (never widened through a float), and the report carries it as a JSON
/// integer.
#[test]
fn every_integer_width_prints_exact() {
    let w = walker_with(WIDTHS);
    let cases = [
        ("a: -1", "a", "-1", DifferenceValue::Int(-1)),
        ("b: 255", "b", "255", DifferenceValue::UInt(255)),
        ("c: -32768", "c", "-32768", DifferenceValue::Int(-32768)),
        ("d: 65535", "d", "65535", DifferenceValue::UInt(65535)),
        (
            "e: -2147483648",
            "e",
            "-2147483648",
            DifferenceValue::Int(-2_147_483_648),
        ),
        (
            "f: 4294967295",
            "f",
            "4294967295",
            DifferenceValue::UInt(4_294_967_295),
        ),
        (
            "g: -9223372036854775808",
            "g",
            "-9223372036854775808",
            DifferenceValue::Int(i64::MIN),
        ),
        (
            "h: 18446744073709551615",
            "h",
            "18446744073709551615",
            DifferenceValue::UInt(u64::MAX),
        ),
    ];
    let zero = widths(&w, "");
    for (set, path, text, json) in cases {
        let value = first_value(&w, &widths(&w, set), &zero);
        assert_eq!(value.path, path, "{set}");
        assert_eq!(value.recorded, shown(text, json), "{set}");
        assert_eq!(value.replayed.text, "0", "{set}");
    }
}

/// A bool prints `true` / `false`; a bool byte 2 reads as true, so a pair
/// of byte 1 and byte 2 carries each side's byte.
#[test]
fn bools_print_their_word_and_a_byte_past_one_shows_its_bits() {
    let w = walker_with(WIDTHS);
    let value = first_value(&w, &widths(&w, ""), &widths(&w, "i: true"));
    assert_eq!(value.path, "i");
    assert_eq!(value.recorded, shown("false", DifferenceValue::Bool(false)));
    assert_eq!(value.replayed, shown("true", DifferenceValue::Bool(true)));

    // Widths' fixed section: a@0 b@1 c@2 d@4 e@8 f@12 g@16 h@24 i@32 j@36
    // k@40 (48 B), then s's entry: `i` is frame byte 72.
    let one = widths(&w, "i: true");
    assert_eq!(one[72], 1);
    let mut two = one.clone();
    two[72] = 2;
    let value = first_value(&w, &one, &two);
    assert_eq!(value.path, "i");
    assert_eq!(
        value.recorded,
        shown(
            "true (0x01)",
            DifferenceValue::Text("true (0x01)".to_string())
        )
    );
    assert_eq!(
        value.replayed,
        shown(
            "true (0x02)",
            DifferenceValue::Text("true (0x02)".to_string())
        )
    );

    // Against a false the byte 2 still carries its byte, so the report keeps
    // the 2 a plain `true` would lose.
    let zero = widths(&w, "");
    assert_eq!(zero[72], 0);
    let value = first_value(&w, &zero, &two);
    assert_eq!(value.recorded, shown("false", DifferenceValue::Bool(false)));
    assert_eq!(
        value.replayed,
        shown(
            "true (0x02)",
            DifferenceValue::Text("true (0x02)".to_string())
        )
    );
}

/// Floats print their own shortest form (`0.1` for a float32 0.1), the
/// YAML spellings for the non-finite values, and their bits when two NaNs
/// print the same.
#[test]
fn floats_print_their_shortest_form_and_nan_payloads_show_bits() {
    let w = walker_with(WIDTHS);
    let zero = widths(&w, "");
    let cases = [
        ("j: 0.1", "j", "0.1", DifferenceValue::Float32(0.1)),
        ("k: 1e-7", "k", "1e-7", DifferenceValue::Float64(1e-7)),
        ("k: 5e-324", "k", "5e-324", DifferenceValue::Float64(5e-324)),
        ("k: -0.0", "k", "-0.0", DifferenceValue::Float64(-0.0)),
        (
            "k: .inf",
            "k",
            ".inf",
            DifferenceValue::Text(".inf".to_string()),
        ),
        (
            "k: -.inf",
            "k",
            "-.inf",
            DifferenceValue::Text("-.inf".to_string()),
        ),
        (
            "j: -.inf",
            "j",
            "-.inf",
            DifferenceValue::Text("-.inf".to_string()),
        ),
    ];
    for (set, path, text, json) in cases {
        let value = first_value(&w, &widths(&w, set), &zero);
        assert_eq!(value.path, path, "{set}");
        assert_eq!(value.recorded, shown(text, json), "{set}");
        assert_eq!(value.replayed.text, "0.0", "{set}");
    }
    // -0.0 against 0.0 prints differently, so no bits are added.
    let value = first_value(&w, &widths(&w, "k: -0.0"), &zero);
    assert_eq!(
        (value.recorded.text.as_str(), value.replayed.text.as_str()),
        ("-0.0", "0.0")
    );

    // `j` (float32) is payload 36: frame byte 76. The codec writes every NaN
    // quiet: 0x7fc00000.
    let quiet = widths(&w, "j: .nan");
    assert_eq!(quiet[76..80], [0x00, 0x00, 0xc0, 0x7f]);
    let mut payload = quiet.clone();
    payload[76] = 0x01;
    let value = first_value(&w, &quiet, &payload);
    assert_eq!(value.path, "j");
    assert_eq!(value.recorded.text, ".nan (0x7fc00000)");
    assert_eq!(value.replayed.text, ".nan (0x7fc00001)");

    // `k` (float64) at payload 40: frame byte 80; quiet NaN 0x7ff8000000000000.
    let quiet = widths(&w, "k: .nan");
    assert_eq!(quiet[80..88], [0, 0, 0, 0, 0, 0, 0xf8, 0x7f]);
    let mut payload = quiet.clone();
    payload[80] = 0x01;
    let value = first_value(&w, &quiet, &payload);
    assert_eq!(value.recorded.text, ".nan (0x7ff8000000000000)");
    assert_eq!(
        value.replayed,
        shown(
            ".nan (0x7ff8000000000001)",
            DifferenceValue::Text(".nan (0x7ff8000000000001)".to_string())
        )
    );
}

/// Strings print double quoted with the YAML escapes, and the report
/// carries the string itself.
#[test]
fn strings_print_quoted_with_escapes() {
    let w = walker_with(WIDTHS);
    let value = first_value(&w, &widths(&w, "s: \"map\""), &widths(&w, "s: \"odom\""));
    assert_eq!(value.path, "s");
    assert_eq!(
        value.recorded,
        shown("\"map\"", DifferenceValue::Text("map".to_string()))
    );
    assert_eq!(
        value.replayed,
        shown("\"odom\"", DifferenceValue::Text("odom".to_string()))
    );
    let value = first_value(&w, &widths(&w, "s: \"a\\nb\""), &widths(&w, "s: \"a\\tb\""));
    assert_eq!(value.recorded.text, "\"a\\nb\"");
    assert_eq!(value.replayed.text, "\"a\\tb\"");
    assert_eq!(
        value.recorded.json,
        DifferenceValue::Text("a\nb".to_string())
    );
}

/// Two 300-character strings differing at character 120 print a 64-character
/// window of each, from character 104, with `...` on each cut side.
#[test]
fn long_strings_print_a_window_around_the_first_difference() {
    let w = walker_with(WIDTHS);
    let long = |c: char| format!("{}{c}{}", "a".repeat(120), "b".repeat(179));
    let rec = widths(&w, &format!("s: \"{}\"", long('X')));
    let rep = widths(&w, &format!("s: \"{}\"", long('Y')));
    let window = |c: char| {
        format!(
            "...\"{}{c}{}\"... (300 chars)",
            "a".repeat(16),
            "b".repeat(47)
        )
    };
    assert_eq!(
        line(&w, &rec, &rep),
        format!(
            "frame 0 on '/t' differs at s from char 120: recorded {}, replayed {} (recorded 396 \
             B, replayed 396 B)",
            window('X'),
            window('Y')
        )
    );
    let entry = report_entry(&locate(&w, &rec, &rep));
    assert_eq!(
        (entry.recorded, entry.field_byte, entry.field_char),
        (Some(DifferenceValue::Text(window('X'))), None, Some(120))
    );
}

/// A string side that is not UTF-8 says so with its length.
#[test]
fn a_string_that_is_not_utf8_prints_its_length() {
    let w = walker_with(WIDTHS);
    let rec = widths(&w, "s: \"abc\"");
    let mut rep = rec.clone();
    // The string's 3 bytes are the last 3 of the frame.
    let at = rep.len() - 2;
    assert_eq!(rep[at], b'b');
    rep[at] = 0xff;
    let value = first_value(&w, &rec, &rep);
    assert_eq!(value.path, "s");
    assert_eq!(value.recorded.text, "\"abc\"");
    assert_eq!(
        value.replayed,
        shown(
            "not valid UTF-8 (3 B)",
            DifferenceValue::Text("not valid UTF-8 (3 B)".to_string())
        )
    );
}

/// A `string_fixed[8]` whose text is equal but whose bytes past the NUL
/// differ names the field and the byte.
#[test]
fn a_fixed_string_differing_past_its_nul_names_the_byte() {
    let w = walker_with("schemas:\n  Tag:\n    fields:\n      string_fixed[8] name: {}\n");
    let rec = frame(&w, "Tag", "name: \"abc\"\n");
    let mut rep = rec.clone();
    // name is payload 0..8: "abc" then NULs; field byte 5 is frame byte 45.
    assert_eq!(rep[40..48], [b'a', b'b', b'c', 0, 0, 0, 0, 0]);
    rep[45] = 0x7a;
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at name: recorded \"abc\", replayed \"abc\" (bytes differ past \
         the NUL at field byte 5) (recorded 48 B, replayed 48 B)"
    );
    // The report carries the byte too: its two values are the same text.
    let entry = report_entry(&locate(&w, &rec, &rep));
    assert_eq!(
        (
            entry.recorded,
            entry.replayed,
            entry.field_byte,
            entry.field_char
        ),
        (
            Some(DifferenceValue::Text("abc".to_string())),
            Some(DifferenceValue::Text("abc".to_string())),
            Some(5),
            None
        )
    );
}

// ---------------------------------------------------------------------------
// Arrays
// ---------------------------------------------------------------------------

const ARRAYS: &str = "\
schemas:
  Arrays:
    fields:
      float64[4] covariance: {}
      float32[] ranges: {}
      uint8[] data: {}
      int8[] signed: {}
      bool[] flags: {}
      string[] names: {}
";

fn arrays(w: &FrameWalker, ranges: &str, data: &str, signed: &str, names: &str) -> Vec<u8> {
    frame(
        w,
        "Arrays",
        &format!(
            "covariance: [0.0, 0.0, 0.0, 0.0]\nranges: {ranges}\ndata: {data}\nsigned: \
             {signed}\nflags: [true, false]\nnames: {names}\n"
        ),
    )
}

/// An element of a numeric array is named by its index, in its own type.
#[test]
fn an_array_element_is_named_by_its_index() {
    let w = walker_with(ARRAYS);
    let base = arrays(&w, "[1.0, 2.0, 3.0]", "[7, 8]", "[-1, 2]", "[\"a\", \"b\"]");
    let ranges = arrays(&w, "[1.0, 2.5, 3.0]", "[7, 8]", "[-1, 2]", "[\"a\", \"b\"]");
    assert_eq!(
        first_value(&w, &base, &ranges),
        ValueDifference {
            path: "ranges[1]".to_string(),
            within: None,
            recorded: shown("2.0", DifferenceValue::Float32(2.0)),
            replayed: shown("2.5", DifferenceValue::Float32(2.5)),
        }
    );
    let data = arrays(&w, "[1.0, 2.0, 3.0]", "[7, 9]", "[-1, 2]", "[\"a\", \"b\"]");
    let value = first_value(&w, &base, &data);
    assert_eq!(value.path, "data[1]");
    assert_eq!(value.recorded, shown("8", DifferenceValue::UInt(8)));
    let signed = arrays(&w, "[1.0, 2.0, 3.0]", "[7, 8]", "[1, 2]", "[\"a\", \"b\"]");
    let value = first_value(&w, &base, &signed);
    assert_eq!(value.path, "signed[0]");
    assert_eq!(value.recorded, shown("-1", DifferenceValue::Int(-1)));
    let names = arrays(&w, "[1.0, 2.0, 3.0]", "[7, 8]", "[-1, 2]", "[\"a\", \"c\"]");
    let value = first_value(&w, &base, &names);
    assert_eq!(value.path, "names[1]");
    assert_eq!(value.recorded.text, "\"b\"");
    assert_eq!(value.replayed.text, "\"c\"");
}

/// A fixed array's last element is named by its index.
#[test]
fn a_fixed_array_element_is_named_by_its_index() {
    let w = walker_with(ARRAYS);
    let base = arrays(&w, "[]", "[]", "[]", "[]");
    let rep = frame(
        &w,
        "Arrays",
        "covariance: [0.0, 0.0, 0.0, 0.5]\nranges: []\ndata: []\nsigned: []\nflags: [true, \
         false]\nnames: []\n",
    );
    let value = first_value(&w, &base, &rep);
    assert_eq!(value.path, "covariance[3]");
    assert_eq!(
        (value.recorded.text.as_str(), value.replayed.text.as_str()),
        ("0.0", "0.5")
    );
}

/// Arrays of different lengths name the array with both lengths first, then
/// count every differing element of the common prefix.
#[test]
fn arrays_of_different_lengths_name_the_lengths_then_count_the_prefix() {
    let w = walker_with(ARRAYS);
    let rec = arrays(&w, "[1.0, 2.0, 3.0]", "[]", "[]", "[]");
    let rep = arrays(&w, "[1.0, 9.0, 9.0, 4.0]", "[]", "[]", "[]");
    match locate(&w, &rec, &rep) {
        FrameDifference::Field { value, more } => {
            assert_eq!(value.path, "ranges");
            assert_eq!(value.recorded.text, "3 element(s)");
            assert_eq!(value.replayed.text, "4 element(s)");
            // ranges[1] and ranges[2] differ in the common prefix.
            assert_eq!(more, 2);
        }
        other => panic!("expected a field difference, got {other:?}"),
    }
}

/// A `bool[]` element compares by its byte: byte 2 reads as true, so a
/// byte 1 against a byte 2 carries each side's byte.
#[test]
fn a_bool_array_element_byte_past_one_shows_its_bits() {
    let w = walker_with(ARRAYS);
    let rec = arrays(&w, "[]", "[]", "[]", "[]");
    // `flags: [true, false]` are the last variable field but one: find its
    // two bytes [1, 0] by its offset-table entry (#3, payload 32 + 24).
    let entry = 40 + 32 + 3 * 8;
    let at = 40 + u32::from_le_bytes(rec[entry..entry + 4].try_into().unwrap()) as usize;
    assert_eq!(rec[at..at + 2], [1, 0]);
    let mut rep = rec.clone();
    rep[at] = 2;
    let value = first_value(&w, &rec, &rep);
    assert_eq!(value.path, "flags[0]");
    assert_eq!(value.recorded.text, "true (0x01)");
    assert_eq!(value.replayed.text, "true (0x02)");
    // `flags[1]` is a false; a byte 2 there reads `true (0x02)` against it.
    let mut rep = rec.clone();
    rep[at + 1] = 2;
    let value = first_value(&w, &rec, &rep);
    assert_eq!(value.path, "flags[1]");
    assert_eq!(
        (value.recorded.text.as_str(), value.replayed.text.as_str()),
        ("false", "true (0x02)")
    );
}

/// Arrays of messages recurse per element: `poses[2].position.x` in a
/// `PoseArray`, and a variable element's `poses[1].header.frame_id` in a
/// `nav_msgs/Path`.
#[test]
fn arrays_of_messages_recurse_per_element() {
    let w = builtin();
    let pose = |x: &str| {
        format!(
            "  - position: {{x: {x}, y: 0.0, z: 0.0}}\n    orientation: {{x: 0.0, y: 0.0, z: 0.0, \
             w: 1.0}}\n"
        )
    };
    let header = "header:\n  stamp: {sec: 0, nanosec: 0}\n  frame_id: \"map\"\n";
    let poses = |third: &str| {
        frame(
            w,
            "geometry_msgs/PoseArray",
            &format!(
                "{header}poses:\n{}{}{}",
                pose("1.0"),
                pose("2.0"),
                pose(third)
            ),
        )
    };
    let value = first_value(w, &poses("3.0"), &poses("3.5"));
    assert_eq!(value.path, "poses[2].position.x");
    assert_eq!(
        (value.recorded.text.as_str(), value.replayed.text.as_str()),
        ("3.0", "3.5")
    );

    let stamped = |frame_id: &str| {
        format!(
            "  - header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: \"{frame_id}\"}}\n    pose:\n      \
             position: {{x: 0.0, y: 0.0, z: 0.0}}\n      orientation: {{x: 0.0, y: 0.0, z: 0.0, \
             w: 1.0}}\n"
        )
    };
    let path = |second: &str| {
        frame(
            w,
            "nav_msgs/Path",
            &format!("{header}poses:\n{}{}", stamped("map"), stamped(second)),
        )
    };
    let value = first_value(w, &path("map"), &path("odom"));
    assert_eq!(value.path, "poses[1].header.frame_id");
    assert_eq!(
        (value.recorded.text.as_str(), value.replayed.text.as_str()),
        ("\"map\"", "\"odom\"")
    );
}

/// An array whose bytes are not in the canonical element framing compares
/// as bytes: a `geometry_msgs/Point[]` blob of 25 bytes is no whole number
/// of 24-byte points.
#[test]
fn an_opaque_array_compares_as_bytes() {
    let w =
        walker_with("schemas:\n  Cloud:\n    fields:\n      geometry_msgs/Point[] points: {}\n");
    let hash = hash_of(&w, "Cloud");
    // payload: entry (offset 8, length 25), then the 25-byte blob.
    let blob = |byte12: u8, len: usize| {
        let mut payload = Vec::new();
        payload.extend_from_slice(&8u32.to_le_bytes());
        payload.extend_from_slice(&(len as u32).to_le_bytes());
        let mut body = vec![0u8; len];
        body[12] = byte12;
        payload.extend_from_slice(&body);
        raw_frame(hash, 0, 1, &payload)
    };
    let (rec, rep) = (blob(0x4c, 25), blob(0x4d, 25));
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at points, byte 12 of its opaque bytes: recorded 0x4c, replayed \
         0x4d (recorded 73 B, replayed 73 B)"
    );
    assert_eq!(
        serde_json::to_value(report_entry(&locate(&w, &rec, &rep))).expect("serializes"),
        serde_json::json!({"kind": "field", "path": "points", "recorded": "0x4c",
            "replayed": "0x4d", "field_byte": 12, "field_char": null, "byte_offset": null,
            "region": null, "reason": null, "more": 0})
    );
    // One side canonical (one whole point), the other not: the raw bytes.
    let (rec, rep) = (blob(0, 24), blob(0, 25));
    let value = first_value(&w, &rec, &rep);
    assert_eq!(value.path, "points");
    assert_eq!(value.recorded.text, "24 opaque byte(s)");
    assert_eq!(value.replayed.text, "25 opaque byte(s)");
}

/// An empty variable nested blob (the `set_<f>_bytes(&[])` idiom) compares
/// as the schema's zero values, as `topic echo` prints it.
#[test]
fn an_empty_nested_blob_compares_as_zero_values() {
    let w = builtin();
    let rep = frame(
        w,
        "geometry_msgs/PoseStamped",
        "header: {stamp: {sec: 5, nanosec: 0}, frame_id: \"\"}\npose:\n  position: {x: 0.0, y: \
         0.0, z: 0.0}\n  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}\n",
    );
    // PoseStamped: fixed `pose` (56 B), then one entry for `header`.
    let layout = w
        .layout_for_name("geometry_msgs/PoseStamped")
        .expect("built-in");
    assert_eq!((layout.fixed_size, layout.variable_fields.len()), (56, 1));
    let mut payload = vec![0u8; 56];
    payload.extend_from_slice(&[0u8; 8]); // entry (0, 0): the empty blob
    let rec = raw_frame(hash_of(w, "geometry_msgs/PoseStamped"), 56, 1, &payload);
    let value = first_value(w, &rec, &rep);
    assert_eq!(value.path, "header.stamp.sec");
    assert_eq!(value.recorded, shown("0", DifferenceValue::Int(0)));
    assert_eq!(value.replayed, shown("5", DifferenceValue::Int(5)));
}

/// A fixed array of fixed strings (`string_fixed[4]`, three of them; the
/// YAML type grammar cannot spell it, so the schema is built here): an
/// element is named by its index, and an element whose text is equal but
/// whose bytes differ past its NUL is named with that byte.
#[test]
fn a_fixed_array_of_fixed_strings_names_its_element() {
    let mut tags = MessageSchema::new("Tags");
    tags.add_field(FieldDef::new(
        "tags",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::StringFixed(4)),
            length: 3,
        },
    ));
    let mut schemas = crate::schema_cmd::parse_builtin_schemas();
    schemas.push(tags);
    let w = FrameWalker::new(schemas).0;
    let hash = hash_of(&w, "Tags");
    // The 12 fixed bytes: "ab", then `middle`, then `last`, 4 bytes each.
    let tagged = |middle: &[u8; 4], last: &[u8; 4]| {
        let mut payload = b"ab\0\0".to_vec();
        payload.extend_from_slice(middle);
        payload.extend_from_slice(last);
        raw_frame(hash, 12, 0, &payload)
    };
    let rec = tagged(b"cd\0\0", b"ef\0\0");
    assert_eq!(
        line(&w, &rec, &tagged(b"cx\0\0", b"ef\0\0")),
        "frame 0 on '/t' differs at tags[1]: recorded \"cd\", replayed \"cx\" (recorded 52 B, \
         replayed 52 B)"
    );
    // tags[2]'s byte 3 is payload 11, frame byte 51.
    let rep = tagged(b"cd\0\0", b"ef\0z");
    assert_eq!(first_diff(&rec, &rep), 51);
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at tags[2]: recorded \"ef\", replayed \"ef\" (bytes differ past \
         the NUL at field byte 3) (recorded 52 B, replayed 52 B)"
    );
}

/// An array of messages whose two sides differ in length names the array
/// with both lengths, then counts what differs in the common prefix.
#[test]
fn an_array_of_messages_of_different_lengths_names_both_lengths() {
    let w = builtin();
    let poses = |xs: &[&str]| {
        let mut yaml =
            String::from("header:\n  stamp: {sec: 0, nanosec: 0}\n  frame_id: \"map\"\nposes:\n");
        for x in xs {
            yaml.push_str(&format!(
                "  - position: {{x: {x}, y: 0.0, z: 0.0}}\n    orientation: {{x: 0.0, y: 0.0, \
                 z: 0.0, w: 1.0}}\n"
            ));
        }
        frame(w, "geometry_msgs/PoseArray", &yaml)
    };
    let (rec, rep) = (poses(&["1.0", "2.0"]), poses(&["1.5", "2.0", "3.0"]));
    match locate(w, &rec, &rep) {
        FrameDifference::Field { value, more } => {
            assert_eq!(value.path, "poses");
            assert_eq!(
                value.recorded,
                shown(
                    "2 element(s)",
                    DifferenceValue::Text("2 element(s)".to_string())
                )
            );
            assert_eq!(value.replayed.text, "3 element(s)");
            // poses[0].position.x differs in the common prefix.
            assert_eq!(more, 1);
        }
        other => panic!("expected a field difference, got {other:?}"),
    }
}

/// Two strings that are not UTF-8 on either side, of one length, name the
/// first byte that differs.
#[test]
fn two_strings_that_are_not_utf8_name_the_differing_byte() {
    let w = walker_with(WIDTHS);
    let base = widths(&w, "s: \"abc\"");
    // The 40-byte header, the 48-byte fixed section, s's 8-byte entry, then
    // "abc": 99 bytes, `b` at frame byte 97.
    assert_eq!(base.len(), 99);
    assert_eq!(base[97], b'b');
    let (mut rec, mut rep) = (base.clone(), base);
    rec[97] = 0xff;
    rep[97] = 0xfe;
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at s, byte 1 of its bytes: recorded 0xff, replayed 0xfe \
         (recorded 99 B, replayed 99 B)"
    );
    let entry = report_entry(&locate(&w, &rec, &rep));
    assert_eq!(
        (entry.recorded, entry.field_byte),
        (Some(DifferenceValue::Text("0xff".to_string())), Some(1))
    );
}

const POINTS_IN_A_MESSAGE: &str = "\
schemas:
  PtsInner:
    fields:
      geometry_msgs/Point[] pts: {}
  PtsOuter:
    fields:
      PtsInner h: {}
";

/// A zero-default value against one of another shape: an empty `h` blob on
/// one side, and on the other an `h` whose `pts` is a 25-byte opaque blob.
/// Neither side has bytes to compare against the other's.
#[test]
fn a_zero_value_against_a_value_of_another_shape_names_both() {
    let w = walker_with(POINTS_IN_A_MESSAGE);
    let hash = hash_of(&w, "PtsOuter");
    // `h`'s entry (0, 0): the empty blob.
    let empty = raw_frame(hash, 0, 1, &[0u8; 8]);
    // `h`'s entry (8, 33), then h: its own entry (8, 25) and 25 zero bytes.
    let mut payload = Vec::new();
    payload.extend_from_slice(&8u32.to_le_bytes());
    payload.extend_from_slice(&33u32.to_le_bytes());
    payload.extend_from_slice(&8u32.to_le_bytes());
    payload.extend_from_slice(&25u32.to_le_bytes());
    payload.extend_from_slice(&[0u8; 25]);
    let opaque = raw_frame(hash, 0, 1, &payload);
    assert_eq!(
        line(&w, &empty, &opaque),
        "frame 0 on '/t' differs at h.pts: recorded the zero value, replayed a value of another \
         shape (recorded 48 B, replayed 81 B)"
    );
}

/// A zero-default message's fixed array reads as zeros: an empty `h` blob
/// against an `h` whose `v` is `[0.0, 0.0, 1.5]` names `h.v[2]`.
#[test]
fn a_zero_default_fixed_array_compares_as_zeros() {
    let w = walker_with(
        "schemas:\n  Vec3s:\n    fields:\n      float64[3] v: {}\n      string s: {}\n  \
         VecHolder:\n    fields:\n      Vec3s h: {}\n",
    );
    let rec = raw_frame(hash_of(&w, "VecHolder"), 0, 1, &[0u8; 8]);
    let rep = frame(&w, "VecHolder", "h: {v: [0.0, 0.0, 1.5], s: \"\"}\n");
    assert_eq!(
        first_value(&w, &rec, &rep),
        ValueDifference {
            path: "h.v[2]".to_string(),
            within: None,
            recorded: shown("0.0", DifferenceValue::Float64(0.0)),
            replayed: shown("1.5", DifferenceValue::Float64(1.5)),
        }
    );
}

/// An array of a message the schema set cannot resolve is opaque to the
/// walker, and compares as its bytes: counted behind a differing value, and
/// named by its byte when it is the only difference.
#[test]
fn an_array_of_an_unresolvable_message_compares_as_bytes() {
    let w = walker_with(
        "schemas:\n  Crate:\n    fields:\n      float64 v: {}\n      Unknown[] xs: {}\n",
    );
    let hash = hash_of(&w, "Crate");
    // v (8 B), xs's entry (16, 4), then xs's 4 bytes: 60-byte frames.
    let crate_frame = |v: f64, xs: [u8; 4]| {
        let mut payload = v.to_le_bytes().to_vec();
        payload.extend_from_slice(&16u32.to_le_bytes());
        payload.extend_from_slice(&4u32.to_le_bytes());
        payload.extend_from_slice(&xs);
        raw_frame(hash, 8, 1, &payload)
    };
    let rec = crate_frame(1.0, [1, 2, 3, 4]);
    assert_eq!(
        line(&w, &rec, &crate_frame(2.0, [1, 2, 9, 4])),
        "frame 0 on '/t' differs at v: recorded 1.0, replayed 2.0; 1 more value(s) differ \
         (recorded 60 B, replayed 60 B)"
    );
    assert_eq!(
        line(&w, &rec, &crate_frame(1.0, [1, 2, 9, 4])),
        "frame 0 on '/t' differs at xs, byte 2 of its opaque bytes: recorded 0x03, replayed 0x09 \
         (recorded 60 B, replayed 60 B)"
    );
}

/// A nested message of a schema the set cannot resolve: the walker refuses
/// a frame holding one, so only hand-built values reach this arm, and they
/// compare as values of no known shape.
#[test]
fn a_nested_message_of_an_unresolvable_schema_compares_by_shape() {
    let w = walker_with("schemas:\n  Wrapper:\n    fields:\n      Unknown u: {}\n");
    let schema = w.schema_for_name("Wrapper").expect("fixture");
    let wrapper = |x: f64| FrameValue {
        schema_name: "Wrapper".to_string(),
        fields: vec![NamedValue {
            name: "u".to_string(),
            value: FrameValueKind::Nested(Box::new(FrameValue {
                schema_name: "Unknown".to_string(),
                fields: vec![NamedValue {
                    name: "x".to_string(),
                    value: FrameValueKind::F64(x),
                }],
            })),
        }],
    };
    assert_eq!(
        diff_frame_values(&w, schema, &wrapper(1.0), &wrapper(1.0), &[]),
        FrameValueDiff::default()
    );
    let found = diff_frame_values(&w, schema, &wrapper(1.0), &wrapper(2.0), &[]);
    let value = found.first.expect("u differs");
    assert_eq!(
        (
            value.path.as_str(),
            value.recorded.text.as_str(),
            value.replayed.text.as_str()
        ),
        ("u", "a value of another shape", "a value of another shape")
    );
}

// ---------------------------------------------------------------------------
// Counting and the tolerance skip list
// ---------------------------------------------------------------------------

/// Equal values name nothing; six differing values name one and count
/// five; a million differing bytes count 999999.
#[test]
fn the_rest_of_the_differing_values_are_counted() {
    let w = walker_with(WIDTHS);
    let a = widths(&w, "");
    let schema = w.schema_for_name("Widths").expect("fixture");
    let rv = w.walk_by_hash(&a).expect("decodes");
    let pv = w.walk_by_hash(&a).expect("decodes");
    assert_eq!(
        diff_frame_values(&w, schema, &rv, &pv, &[]),
        FrameValueDiff::default()
    );

    let six = frame(
        &w,
        "Widths",
        "a: 1\nb: 1\nc: 1\nd: 1\ne: 1\nf: 1\ng: 0\nh: 0\ni: false\nj: 0.0\nk: 0.0\ns: \"\"\n",
    );
    match locate(&w, &a, &six) {
        FrameDifference::Field { value, more } => {
            assert_eq!(value.path, "a");
            assert_eq!(more, 5);
        }
        other => panic!("expected a field difference, got {other:?}"),
    }

    let bytes = walker_with("schemas:\n  Blob:\n    fields:\n      uint8[] data: {}\n");
    let hash = hash_of(&bytes, "Blob");
    let blob = |fill: u8| {
        let mut payload = Vec::new();
        payload.extend_from_slice(&8u32.to_le_bytes());
        payload.extend_from_slice(&1_000_000u32.to_le_bytes());
        payload.extend(std::iter::repeat_n(fill, 1_000_000));
        raw_frame(hash, 0, 1, &payload)
    };
    match locate(&bytes, &blob(1), &blob(2)) {
        FrameDifference::Field { value, more } => {
            assert_eq!(value.path, "data[0]");
            assert_eq!(more, 999_999);
        }
        other => panic!("expected a field difference, got {other:?}"),
    }
}

/// A field a tolerance metric judges is never compared: the next differing
/// field is named, and with only tolerated fields differing nothing is.
#[test]
fn a_tolerated_field_is_skipped() {
    let w = builtin();
    let vector = |x: &str, y: &str| {
        frame(
            w,
            "geometry_msgs/Vector3",
            &format!("x: {x}\ny: {y}\nz: 0.0\n"),
        )
    };
    let (rec, rep) = (vector("1.0", "1.0"), vector("2.0", "3.0"));
    let schema = w
        .schema_for_name("geometry_msgs/Vector3")
        .expect("built-in");
    let (rv, pv) = (
        w.walk_by_hash(&rec).expect("decodes"),
        w.walk_by_hash(&rep).expect("decodes"),
    );
    let found = diff_frame_values(w, schema, &rv, &pv, &["x"]);
    assert_eq!(found.first.map(|v| v.path), Some("y".to_string()));
    assert_eq!(found.more, 0);
    let found = diff_frame_values(w, schema, &rv, &pv, &["x", "y"]);
    assert_eq!(found, FrameValueDiff::default());
}

// ---------------------------------------------------------------------------
// The precedence rule
// ---------------------------------------------------------------------------

/// A differing sequence is named ahead of the payload, which is counted.
#[test]
fn a_header_field_is_named_ahead_of_the_payload() {
    let w = builtin();
    let mut rec = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut rep = frame(w, "geometry_msgs/Vector3", "x: 2.0\ny: 0.0\nz: 0.0\n");
    put_u32(&mut rec, 20, 44);
    put_u32(&mut rep, 20, 43);
    let verdict = ByteVerdict {
        index: 43,
        first_diff_byte_offset: 20,
        recorded_len: 64,
        replayed_len: 64,
        difference: locate(w, &rec, &rep),
    };
    assert_eq!(
        byte_mismatch_detail("/xr/ticker/cmd", "", &verdict),
        "frame 43 on '/xr/ticker/cmd' differs in the wire header at sequence: recorded 44, \
         replayed 43; 1 more value(s) differ (recorded 64 B, replayed 64 B)"
    );
    assert_eq!(
        serde_json::to_value(report_entry(&verdict.difference)).expect("serializes"),
        serde_json::json!({
            "kind": "header",
            "path": "sequence",
            "recorded": 44,
            "replayed": 43,
            "field_byte": null,
            "field_char": null,
            "byte_offset": null,
            "region": null,
            "reason": null,
            "more": 1,
        })
    );
}

/// `timestamp_ns` carries ` ns`; a step tag prints as rank and step.
#[test]
fn timestamps_and_step_tags_print_their_meaning() {
    let w = builtin();
    let base = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut later = base.clone();
    put_u64(&mut later, 24, 9);
    assert_eq!(
        line(w, &base, &later),
        "frame 0 on '/t' differs in the wire header at timestamp_ns: recorded 7 ns, replayed 9 \
         ns (recorded 64 B, replayed 64 B)"
    );
    let mut tagged = base.clone();
    put_u64(&mut tagged, 32, StepTag::new(0, 5).expect("in range").raw());
    assert_eq!(
        line(w, &base, &tagged),
        "frame 0 on '/t' differs in the wire header at step_tag: recorded untagged, replayed \
         rank 0 step 5 (recorded 64 B, replayed 64 B)"
    );
    let mut rank_only = base.clone();
    put_u64(&mut rank_only, 32, 2u64 << 48);
    assert_eq!(
        line(w, &tagged, &rank_only),
        "frame 0 on '/t' differs in the wire header at step_tag: recorded rank 0 step 5, \
         replayed rank 2, no step (recorded 64 B, replayed 64 B)"
    );
}

/// A variable field whose length changed moves `total_size`; the field is
/// named, and `total_size` is neither named nor counted.
#[test]
fn total_size_is_never_named_while_a_value_differs() {
    let w = walker_with("schemas:\n  Note:\n    fields:\n      string note: {}\n");
    let rec = frame(&w, "Note", "note: \"map\"\n");
    let rep = frame(&w, "Note", "note: \"odometry\"\n");
    assert_eq!(first_diff(&rec, &rep), 8);
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at note: recorded \"map\", replayed \"odometry\" (recorded 51 \
         B, replayed 56 B)"
    );
}

/// With only `total_size` different, it is what the line names.
#[test]
fn total_size_alone_is_named() {
    let w = builtin();
    let rec = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut rep = rec.clone();
    put_u32(&mut rep, 8, 99);
    assert_eq!(
        line(w, &rec, &rep),
        "frame 0 on '/t' differs in the wire header at total_size: recorded 64, replayed 99 \
         (recorded 64 B, replayed 64 B)"
    );
}

/// The offset table's two header fields are named like the others:
/// `offset_table_offset` (frame bytes 12..16) is 40 + 24 = 64 on a
/// `Vector3`, and `offset_table_count` (16..20) is 0.
#[test]
fn the_offset_table_header_fields_are_named() {
    let w = builtin();
    let rec = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    assert_eq!(rec[12..20], [64, 0, 0, 0, 0, 0, 0, 0]);
    let mut moved = rec.clone();
    put_u32(&mut moved, 12, 99);
    assert_eq!(
        line(w, &rec, &moved),
        "frame 0 on '/t' differs in the wire header at offset_table_offset: recorded 64, \
         replayed 99 (recorded 64 B, replayed 64 B)"
    );
    let mut counted = rec.clone();
    put_u32(&mut counted, 16, 5);
    assert_eq!(
        line(w, &rec, &counted),
        "frame 0 on '/t' differs in the wire header at offset_table_count: recorded 0, replayed \
         5 (recorded 64 B, replayed 64 B)"
    );
    assert_eq!(
        serde_json::to_value(report_entry(&locate(w, &rec, &counted))).expect("serializes"),
        serde_json::json!({"kind": "header", "path": "offset_table_count", "recorded": 0,
            "replayed": 5, "field_byte": null, "field_char": null, "byte_offset": null,
            "region": null, "reason": null, "more": 0})
    );
}

/// Different schema hashes are named and nothing is decoded: the schema set
/// is never even built.
#[test]
fn different_schema_hashes_compare_no_field() {
    let rec = raw_frame(0x1111, 0, 0, &[]);
    let rep = raw_frame(0x2222, 0, 0, &[]);
    let difference = locate_difference(
        || -> &'static FrameWalker { panic!("the schema set is not consulted") },
        &rec,
        &rep,
        0,
        &[],
        None,
    );
    let verdict = ByteVerdict {
        index: 1,
        first_diff_byte_offset: 0,
        recorded_len: 40,
        replayed_len: 40,
        difference,
    };
    assert_eq!(
        byte_mismatch_detail("/t", "", &verdict),
        "frame 1 on '/t' differs in the wire header at schema_hash: recorded \
         0x0000000000001111, replayed 0x0000000000002222; fields not compared, the frames carry \
         different schemas (recorded 40 B, replayed 40 B)"
    );
}

// ---------------------------------------------------------------------------
// Every value equal: where the byte sits
// ---------------------------------------------------------------------------

/// A replayed frame four bytes shorter than its recorded twin, values equal:
/// the bytes past the last field.
#[test]
fn bytes_past_the_last_field() {
    let w = builtin();
    let rep = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut rec = rep.clone();
    rec.extend_from_slice(&[1, 2, 3, 4]);
    put_u32(&mut rec, 8, 68);
    let verdict = ByteVerdict {
        index: 3,
        first_diff_byte_offset: 8,
        recorded_len: 68,
        replayed_len: 64,
        difference: locate_difference(|| w, &rec, &rep, 8, &[], None),
    };
    assert_eq!(
        byte_mismatch_detail("/sr/source/out", "", &verdict),
        "frame 3 on '/sr/source/out' differs at byte 64, in bytes past the last field; every \
         field value is equal (recorded 68 B, replayed 64 B)"
    );
    assert_eq!(
        report_entry(&verdict.difference),
        ByteDifference {
            kind: DifferenceKind::Layout,
            path: None,
            recorded: None,
            replayed: None,
            field_byte: None,
            field_char: None,
            byte_offset: Some(64),
            region: Some("trailing_bytes"),
            reason: None,
            more: 0,
        }
    );
}

/// A tolerated `x` drifted within its bound, and the recorded frame carries
/// four trailing bytes with its `total_size` patched to 68: the diff
/// reports byte 8 (`total_size`), and the byte the line names is past the
/// last field, never one of x's (frame bytes 40..48).
#[test]
fn a_tolerated_field_is_never_named_as_framing() {
    let w = builtin();
    let rep = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut rec = frame(w, "geometry_msgs/Vector3", "x: 1.05\ny: 0.0\nz: 0.0\n");
    rec.extend_from_slice(&[1, 2, 3, 4]);
    put_u32(&mut rec, 8, 68);
    let verdict = ByteVerdict {
        index: 3,
        first_diff_byte_offset: 8,
        recorded_len: 68,
        replayed_len: 64,
        difference: locate_difference(|| w, &rec, &rep, 8, &["x"], None),
    };
    assert_eq!(
        byte_mismatch_detail("/sr/source/out", "", &verdict),
        "frame 3 on '/sr/source/out' differs at byte 64, in bytes past the last field; every \
         field value is equal (recorded 68 B, replayed 64 B)"
    );
    assert_eq!(
        serde_json::to_value(report_entry(&verdict.difference)).expect("serializes"),
        serde_json::json!({"kind": "layout", "path": null, "recorded": null, "replayed": null,
            "field_byte": null, "field_char": null, "byte_offset": 64,
            "region": "trailing_bytes", "reason": null, "more": 0})
    );
}

/// The same tolerated drift between frames of one length whose replayed
/// `total_size` reads 99: no other byte differs outside x, so the line
/// names `total_size`.
#[test]
fn a_tolerated_field_leaves_total_size_to_be_named() {
    let w = builtin();
    let rec = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let mut rep = frame(w, "geometry_msgs/Vector3", "x: 1.05\ny: 0.0\nz: 0.0\n");
    put_u32(&mut rep, 8, 99);
    let verdict = ByteVerdict {
        index: 0,
        first_diff_byte_offset: 8,
        recorded_len: 64,
        replayed_len: 64,
        difference: locate_difference(|| w, &rec, &rep, 8, &["x"], None),
    };
    assert_eq!(
        byte_mismatch_detail("/t", "", &verdict),
        "frame 0 on '/t' differs in the wire header at total_size: recorded 64, replayed 99 \
         (recorded 64 B, replayed 64 B)"
    );
}

const PADDED: &str = "\
schemas:
  Inner:
    fields:
      uint8 a: {}
      float64 b: {}
  Outer:
    fields:
      Inner inner: {}
      Inner[2] arr: {}
";

/// Padding inside an inlined nested message, and inside an element of a
/// fixed array of them, is fixed-section padding; a value there is a value.
/// Inner is `a` @0, 7 padding bytes, `b` @8 (16 B); Outer is `inner` @0 and
/// `arr` @16 (48 B).
#[test]
fn padding_inside_an_inlined_message_is_padding() {
    let w = walker_with(PADDED);
    let rec = frame(
        &w,
        "Outer",
        "inner: {a: 1, b: 2.0}\narr:\n  - {a: 3, b: 4.0}\n  - {a: 5, b: 6.0}\n",
    );
    assert_eq!(rec.len(), 88);
    let region = |at: usize| {
        let mut rep = rec.clone();
        assert_eq!(rep[at], 0, "byte {at} is padding");
        rep[at] = 0x55;
        locate(&w, &rec, &rep)
    };
    // inner's padding byte 3: payload 3, frame byte 43.
    assert_eq!(
        region(43),
        FrameDifference::Layout {
            byte_offset: 43,
            region: LayoutRegion::FixedSectionPadding,
        }
    );
    // arr[1] starts at payload 32; its padding byte 3 is frame byte 75.
    assert_eq!(
        region(75),
        FrameDifference::Layout {
            byte_offset: 75,
            region: LayoutRegion::FixedSectionPadding,
        }
    );
    // arr[1].a is frame byte 72.
    let mut rep = rec.clone();
    assert_eq!(rep[72], 5);
    rep[72] = 6;
    assert_eq!(first_value(&w, &rec, &rep).path, "arr[1].a");
}

/// Padding in the fixed section, the offset table, the gap between two
/// variable fields, and slack inside a nested message's bytes.
#[test]
fn every_region_of_the_frame_is_named() {
    // Pad: uint8 a @0, 7 B padding, float64 b @8 (fixed_size 16).
    let w = walker_with(
        "schemas:\n  Pad:\n    fields:\n      uint8 a: {}\n      float64 b: {}\n  Two:\n    \
         fields:\n      string s: {}\n      float64[] d: {}\n  Holder:\n    fields:\n      \
         std_msgs/Header h: {}\n",
    );
    let rec = frame(&w, "Pad", "a: 1\nb: 2.0\n");
    let mut rep = rec.clone();
    rep[43] = 0x55;
    let region = |rec: &[u8], rep: &[u8]| match locate(&w, rec, rep) {
        FrameDifference::Layout {
            byte_offset,
            region,
        } => (byte_offset, region),
        other => panic!("expected a layout difference, got {other:?}"),
    };
    assert_eq!(region(&rec, &rep), (43, LayoutRegion::FixedSectionPadding));

    // Two: table #0 s, #1 d at payload [0, 16); s "ab" at 16, d aligned to 8
    // at 24, so payload bytes 18..24 are the gap.
    let rec = frame(&w, "Two", "s: \"ab\"\nd: [1.0]\n");
    assert_eq!(rec[40..48], [16, 0, 0, 0, 2, 0, 0, 0]);
    assert_eq!(rec[48..52], [24, 0, 0, 0]);
    let mut gap = rec.clone();
    gap[40 + 20] = 0x55;
    assert_eq!(region(&rec, &gap), (60, LayoutRegion::UnusedVariableBytes));

    // The same "ab" moved 8 bytes later: the entry differs, every value does
    // not.
    let mut moved = Vec::new();
    moved.extend_from_slice(&24u32.to_le_bytes());
    moved.extend_from_slice(&2u32.to_le_bytes());
    moved.extend_from_slice(&32u32.to_le_bytes());
    moved.extend_from_slice(&8u32.to_le_bytes());
    moved.extend_from_slice(&[0; 8]);
    moved.extend_from_slice(b"ab");
    moved.extend_from_slice(&[0; 6]);
    moved.extend_from_slice(&1.0f64.to_le_bytes());
    let moved = raw_frame(hash_of(&w, "Two"), 0, 2, &moved);
    assert_eq!(region(&rec, &moved), (40, LayoutRegion::OffsetTable));

    // Holder: entry (8, 20) for h; h's sub-frame is stamp (8 B), its own
    // entry (16, 3), "map", then 1 byte of slack inside h's 20 bytes.
    let holder = |slack: u8| {
        let mut payload = Vec::new();
        payload.extend_from_slice(&8u32.to_le_bytes());
        payload.extend_from_slice(&20u32.to_le_bytes());
        payload.extend_from_slice(&[0; 8]);
        payload.extend_from_slice(&16u32.to_le_bytes());
        payload.extend_from_slice(&3u32.to_le_bytes());
        payload.extend_from_slice(b"map");
        payload.push(slack);
        raw_frame(hash_of(&w, "Holder"), 0, 1, &payload)
    };
    assert_eq!(
        region(&holder(0), &holder(1)),
        (
            67,
            LayoutRegion::FieldFraming {
                path: "h".to_string()
            }
        )
    );
}

// ---------------------------------------------------------------------------
// Not decoded: every reason
// ---------------------------------------------------------------------------

#[test]
fn a_frame_shorter_than_the_header_is_not_decoded() {
    let short = vec![0u8; 12];
    let whole = raw_frame(1, 0, 0, &[]);
    let never = || -> &'static FrameWalker { panic!("a short frame decodes nothing") };
    assert_eq!(
        locate_difference(never, &short, &whole, 0, &[], None),
        FrameDifference::Undecoded {
            byte_offset: 0,
            reason: UndecodedReason::FrameTooShort {
                side: Side::Recorded,
                len: 12,
            },
        }
    );
    let never = || -> &'static FrameWalker { panic!("a short frame decodes nothing") };
    let difference = locate_difference(never, &whole, &short, 12, &[], None);
    let verdict = ByteVerdict {
        index: 0,
        first_diff_byte_offset: 12,
        recorded_len: 40,
        replayed_len: 12,
        difference,
    };
    assert_eq!(
        byte_mismatch_detail("/t", "", &verdict),
        "frame 0 on '/t' differs at byte 12, not decoded because the replayed frame is 12 B, \
         shorter than the 40-byte wire header (recorded 40 B, replayed 12 B)"
    );
    assert_eq!(
        report_entry(&verdict.difference).reason,
        Some("replayed_frame_too_short")
    );
}

#[test]
fn a_schema_hash_no_schema_has_is_not_decoded() {
    let w = builtin();
    let rec = raw_frame(0, 0, 0, &[1]);
    let rep = raw_frame(0, 0, 0, &[2]);
    assert_eq!(
        line(w, &rec, &rep),
        "frame 0 on '/t' differs at byte 40, not decoded because no schema in this workspace or \
         the built-in set has the frame's schema hash 0x0000000000000000 (recorded 41 B, \
         replayed 41 B)"
    );
}

#[test]
fn a_frame_the_walker_refuses_is_not_decoded_with_its_reason() {
    let w = walker_with("schemas:\n  Note:\n    fields:\n      string note: {}\n");
    let rec = frame(&w, "Note", "note: \"abc\"\n");
    let mut rep = rec.clone();
    // The entry's length (payload 4..8, frame 44..48): 100 bytes, past the end.
    put_u32(&mut rep, 44, 100);
    assert_eq!(
        line(&w, &rec, &rep),
        "frame 0 on '/t' differs at byte 44, not decoded because the replayed frame does not \
         decode as Note: variable field 'note' out of bounds: offset 8 len 100 vs payload len \
         11 (recorded 51 B, replayed 51 B)"
    );
    match locate(&w, &rep, &rec) {
        FrameDifference::Undecoded { reason, .. } => {
            assert_eq!(
                (
                    reason.tag(),
                    reason.clause().starts_with("the recorded frame")
                ),
                ("recorded_frame_undecodable", true)
            );
        }
        other => panic!("expected an undecoded difference, got {other:?}"),
    }
}

/// The tolerance decoder refused a frame and nothing decoded here differs:
/// the refusal is the reason, named as that decoder's, at a byte that
/// differs outside the tolerated fields.
#[test]
fn a_tolerance_decoder_refusal_is_the_reason_when_no_value_differs() {
    let w = builtin();
    let rec = frame(w, "geometry_msgs/Vector3", "x: 1.0\ny: 0.0\nz: 0.0\n");
    let refusal = |side| RemainderRefusal {
        side,
        error: "corrupt".to_string(),
    };
    // Identical frames: no byte differs, so the remainder's own offset (the
    // end of the header) stands.
    assert_eq!(
        locate_difference(|| w, &rec, &rec, 40, &[], Some(&refusal(Side::Recorded))),
        FrameDifference::Undecoded {
            byte_offset: 40,
            reason: UndecodedReason::ToleranceRefused {
                side: Side::Recorded,
                error: "corrupt".to_string(),
            },
        }
    );
    // x (frame bytes 40..48) is tolerated and drifted; four trailing bytes
    // follow the replayed frame's last field. The byte named is 64, never one
    // of x's.
    let mut rep = frame(w, "geometry_msgs/Vector3", "x: 1.5\ny: 0.0\nz: 0.0\n");
    rep.extend_from_slice(&[9, 9, 9, 9]);
    let verdict = ByteVerdict {
        index: 0,
        first_diff_byte_offset: 40,
        recorded_len: 64,
        replayed_len: 68,
        difference: locate_difference(|| w, &rec, &rep, 40, &["x"], Some(&refusal(Side::Replayed))),
    };
    assert_eq!(
        byte_mismatch_detail("/t", "", &verdict),
        "frame 0 on '/t' differs at byte 64, not decoded because the --tolerance decoder \
         refused the replayed frame: corrupt (recorded 64 B, replayed 68 B)"
    );
    assert_eq!(
        report_entry(&verdict.difference).reason,
        Some("replayed_frame_undecodable")
    );
}

// ---------------------------------------------------------------------------
// The line and the report for every arm
// ---------------------------------------------------------------------------

/// Every arm of the detail and its report entry, plain and with a writer
/// scope.
#[test]
fn every_arm_renders_its_line_and_report_entry() {
    let text = |s: &str| DifferenceValue::Text(s.to_string());
    let field = |path: &str, within, recorded, replayed, more| FrameDifference::Field {
        value: ValueDifference {
            path: path.to_string(),
            within,
            recorded,
            replayed,
        },
        more,
    };
    let cases: Vec<(FrameDifference, &str, serde_json::Value)> = vec![
        (
            field(
                "x",
                None,
                shown("5.0", DifferenceValue::Float64(5.0)),
                shown("6.0", DifferenceValue::Float64(6.0)),
                0,
            ),
            "differs at x: recorded 5.0, replayed 6.0",
            serde_json::json!({"kind": "field", "path": "x", "recorded": 5.0, "replayed": 6.0,
                "field_byte": null, "field_char": null, "byte_offset": null, "region": null,
                "reason": null, "more": 0}),
        ),
        (
            field(
                "ranges",
                None,
                shown("360 element(s)", text("360 element(s)")),
                shown("361 element(s)", text("361 element(s)")),
                4,
            ),
            "differs at ranges: recorded 360 element(s), replayed 361 element(s); 4 more \
             value(s) differ",
            serde_json::json!({"kind": "field", "path": "ranges", "recorded": "360 element(s)",
                "replayed": "361 element(s)", "field_byte": null, "field_char": null,
                "byte_offset": null, "region": null, "reason": null, "more": 4}),
        ),
        (
            field(
                "note",
                Some(Within::Char(120)),
                shown("...\"ab\"... (300 chars)", text("...\"ab\"... (300 chars)")),
                shown("...\"ac\"... (300 chars)", text("...\"ac\"... (300 chars)")),
                0,
            ),
            "differs at note from char 120: recorded ...\"ab\"... (300 chars), replayed \
             ...\"ac\"... (300 chars)",
            serde_json::json!({"kind": "field", "path": "note",
                "recorded": "...\"ab\"... (300 chars)", "replayed": "...\"ac\"... (300 chars)",
                "field_byte": null, "field_char": 120, "byte_offset": null, "region": null,
                "reason": null, "more": 0}),
        ),
        (
            field(
                "points",
                Some(Within::OpaqueByte(12)),
                shown("0x4c", text("0x4c")),
                shown("0x4d", text("0x4d")),
                0,
            ),
            "differs at points, byte 12 of its opaque bytes: recorded 0x4c, replayed 0x4d",
            serde_json::json!({"kind": "field", "path": "points", "recorded": "0x4c",
                "replayed": "0x4d", "field_byte": 12, "field_char": null, "byte_offset": null,
                "region": null, "reason": null, "more": 0}),
        ),
        (
            field(
                "s",
                Some(Within::Byte(1)),
                shown("0xff", text("0xff")),
                shown("0xfe", text("0xfe")),
                2,
            ),
            "differs at s, byte 1 of its bytes: recorded 0xff, replayed 0xfe; 2 more value(s) \
             differ",
            serde_json::json!({"kind": "field", "path": "s", "recorded": "0xff",
                "replayed": "0xfe", "field_byte": 1, "field_char": null, "byte_offset": null,
                "region": null, "reason": null, "more": 2}),
        ),
        (
            field(
                "name",
                Some(Within::PastNul(5)),
                shown("\"abc\"", text("abc")),
                shown("\"abc\"", text("abc")),
                0,
            ),
            "differs at name: recorded \"abc\", replayed \"abc\" (bytes differ past the NUL at \
             field byte 5)",
            serde_json::json!({"kind": "field", "path": "name", "recorded": "abc",
                "replayed": "abc", "field_byte": 5, "field_char": null, "byte_offset": null,
                "region": null, "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Header {
                field: HeaderField::StepTag,
                recorded: shown("rank 1 step 2", text("rank 1 step 2")),
                replayed: shown("untagged", text("untagged")),
                more: 0,
            },
            "differs in the wire header at step_tag: recorded rank 1 step 2, replayed untagged",
            serde_json::json!({"kind": "header", "path": "step_tag", "recorded": "rank 1 step 2",
                "replayed": "untagged", "field_byte": null, "field_char": null,
                "byte_offset": null, "region": null, "reason": null, "more": 0}),
        ),
        (
            FrameDifference::SchemaHash {
                recorded: 0xabc,
                replayed: 0xdef,
            },
            "differs in the wire header at schema_hash: recorded 0x0000000000000abc, replayed \
             0x0000000000000def; fields not compared, the frames carry different schemas",
            serde_json::json!({"kind": "schema_hash", "path": "schema_hash",
                "recorded": "0x0000000000000abc", "replayed": "0x0000000000000def",
                "field_byte": null, "field_char": null, "byte_offset": null, "region": null,
                "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Layout {
                byte_offset: 43,
                region: LayoutRegion::FixedSectionPadding,
            },
            "differs at byte 43, in the fixed-section padding; every field value is equal",
            serde_json::json!({"kind": "layout", "path": null, "recorded": null, "replayed": null,
                "field_byte": null, "field_char": null, "byte_offset": 43,
                "region": "fixed_section_padding", "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Layout {
                byte_offset: 40,
                region: LayoutRegion::OffsetTable,
            },
            "differs at byte 40, in the offset table; every field value is equal",
            serde_json::json!({"kind": "layout", "path": null, "recorded": null, "replayed": null,
                "field_byte": null, "field_char": null, "byte_offset": 40,
                "region": "offset_table", "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Layout {
                byte_offset: 67,
                region: LayoutRegion::FieldFraming {
                    path: "h".to_string(),
                },
            },
            "differs at byte 67, in the framing of h; every field value is equal",
            serde_json::json!({"kind": "layout", "path": "h", "recorded": null, "replayed": null,
                "field_byte": null, "field_char": null, "byte_offset": 67,
                "region": "field_framing", "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Layout {
                byte_offset: 60,
                region: LayoutRegion::UnusedVariableBytes,
            },
            "differs at byte 60, in unused bytes between variable fields; every field value is \
             equal",
            serde_json::json!({"kind": "layout", "path": null, "recorded": null, "replayed": null,
                "field_byte": null, "field_char": null, "byte_offset": 60,
                "region": "unused_variable_bytes", "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Layout {
                byte_offset: 64,
                region: LayoutRegion::TrailingBytes,
            },
            "differs at byte 64, in bytes past the last field; every field value is equal",
            serde_json::json!({"kind": "layout", "path": null, "recorded": null, "replayed": null,
                "field_byte": null, "field_char": null, "byte_offset": 64,
                "region": "trailing_bytes", "reason": null, "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 12,
                reason: UndecodedReason::FrameTooShort {
                    side: Side::Recorded,
                    len: 12,
                },
            },
            "differs at byte 12, not decoded because the recorded frame is 12 B, shorter than \
             the 40-byte wire header",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 12,
                "region": null, "reason": "recorded_frame_too_short", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 40,
                reason: UndecodedReason::FrameTooShort {
                    side: Side::Replayed,
                    len: 3,
                },
            },
            "differs at byte 40, not decoded because the replayed frame is 3 B, shorter than \
             the 40-byte wire header",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 40,
                "region": null, "reason": "replayed_frame_too_short", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 40,
                reason: UndecodedReason::UnknownSchemaHash { schema_hash: 0x5 },
            },
            "differs at byte 40, not decoded because no schema in this workspace or the \
             built-in set has the frame's schema hash 0x0000000000000005",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 40,
                "region": null, "reason": "unknown_schema_hash", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 44,
                reason: UndecodedReason::Undecodable {
                    side: Side::Recorded,
                    schema: "sensor_msgs/LaserScan".to_string(),
                    error: "bad entry".to_string(),
                },
            },
            "differs at byte 44, not decoded because the recorded frame does not decode as \
             sensor_msgs/LaserScan: bad entry",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 44,
                "region": null, "reason": "recorded_frame_undecodable", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 44,
                reason: UndecodedReason::Undecodable {
                    side: Side::Replayed,
                    schema: "sensor_msgs/LaserScan".to_string(),
                    error: "bad entry".to_string(),
                },
            },
            "differs at byte 44, not decoded because the replayed frame does not decode as \
             sensor_msgs/LaserScan: bad entry",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 44,
                "region": null, "reason": "replayed_frame_undecodable", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 52,
                reason: UndecodedReason::ToleranceRefused {
                    side: Side::Recorded,
                    error: "corrupt".to_string(),
                },
            },
            "differs at byte 52, not decoded because the --tolerance decoder refused the \
             recorded frame: corrupt",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 52,
                "region": null, "reason": "recorded_frame_undecodable", "more": 0}),
        ),
        (
            FrameDifference::Undecoded {
                byte_offset: 52,
                reason: UndecodedReason::ToleranceRefused {
                    side: Side::Replayed,
                    error: "corrupt".to_string(),
                },
            },
            "differs at byte 52, not decoded because the --tolerance decoder refused the \
             replayed frame: corrupt",
            serde_json::json!({"kind": "undecoded", "path": null, "recorded": null,
                "replayed": null, "field_byte": null, "field_char": null, "byte_offset": 52,
                "region": null, "reason": "replayed_frame_undecodable", "more": 0}),
        ),
    ];
    for (difference, middle, json) in cases {
        let verdict = ByteVerdict {
            index: 4,
            first_diff_byte_offset: 40,
            recorded_len: 64,
            replayed_len: 64,
            difference,
        };
        assert_eq!(
            byte_mismatch_detail("/mp/bus", "", &verdict),
            format!("frame 4 on '/mp/bus' {middle} (recorded 64 B, replayed 64 B)")
        );
        assert_eq!(
            byte_mismatch_detail("/mp/bus", " from writer beta/out", &verdict),
            format!(
                "frame 4 on '/mp/bus' from writer beta/out {middle} (recorded 64 B, replayed 64 \
                 B)"
            )
        );
        assert_eq!(
            serde_json::to_value(report_entry(&verdict.difference)).expect("serializes"),
            json
        );
    }
}

/// A float32 reaches the report in its own shortest form, not widened.
#[test]
fn a_float32_value_reaches_the_report_in_its_own_shortest_form() {
    let entry = ByteDifference {
        kind: DifferenceKind::Field,
        path: Some("j".to_string()),
        recorded: Some(DifferenceValue::Float32(0.1)),
        replayed: Some(DifferenceValue::Float32(-0.0)),
        field_byte: None,
        field_char: None,
        byte_offset: None,
        region: None,
        reason: None,
        more: 0,
    };
    assert_eq!(
        serde_json::to_string(&entry).expect("serializes"),
        "{\"kind\":\"field\",\"path\":\"j\",\"recorded\":0.1,\"replayed\":-0.0,\
         \"field_byte\":null,\"field_char\":null,\"byte_offset\":null,\"region\":null,\
         \"reason\":null,\"more\":0}"
    );
}

// ---------------------------------------------------------------------------
// Determinism and bounds
// ---------------------------------------------------------------------------

/// Two calls over the same frames give the same answer, and the first value
/// across nested arrays is the lowest index in declaration order.
#[test]
fn the_same_frames_always_name_the_same_value() {
    let w = builtin();
    let pose = |x: &str, y: &str| {
        format!(
            "  - position: {{x: {x}, y: {y}, z: 0.0}}\n    orientation: {{x: 0.0, y: 0.0, z: 0.0, \
             w: 1.0}}\n"
        )
    };
    let header = "header:\n  stamp: {sec: 0, nanosec: 0}\n  frame_id: \"map\"\n";
    let rec = frame(
        w,
        "geometry_msgs/PoseArray",
        &format!(
            "{header}poses:\n{}{}",
            pose("1.0", "1.0"),
            pose("2.0", "2.0")
        ),
    );
    let rep = frame(
        w,
        "geometry_msgs/PoseArray",
        &format!(
            "{header}poses:\n{}{}",
            pose("1.0", "9.0"),
            pose("8.0", "2.0")
        ),
    );
    let first = locate(w, &rec, &rep);
    assert_eq!(first, locate(w, &rec, &rep));
    assert_eq!(
        line(w, &rec, &rep),
        format!(
            "frame 0 on '/t' differs at poses[0].position.y: recorded 1.0, replayed 9.0; 1 more \
             value(s) differ (recorded {} B, replayed {} B)",
            rec.len(),
            rep.len()
        )
    );
}

/// A 1 MiB string differing at its end prints a bounded window, and a path
/// past 256 bytes keeps its last 256 behind `...`.
#[test]
fn values_and_paths_stay_bounded() {
    let w = walker_with(WIDTHS);
    let big = |c: char| format!("{}{c}", "a".repeat(1 << 20));
    let value = first_value(
        &w,
        &widths(&w, &format!("s: \"{}\"", big('x'))),
        &widths(&w, &format!("s: \"{}\"", big('y'))),
    );
    assert_eq!(value.within, Some(Within::Char(1 << 20)));
    assert!(
        value.recorded.text.len() < 300,
        "{}",
        value.recorded.text.len()
    );
    assert!(
        value.replayed.text.len() < 300,
        "{}",
        value.replayed.text.len()
    );

    // 30 nested levels of 10-character names: a 329-byte path.
    let mut yaml = String::from("schemas:\n");
    for level in 0..30 {
        let field = if level == 29 {
            "float64 leaf_value: {}".to_string()
        } else {
            format!("L{:02} level_{:04}: {{}}", level + 1, level)
        };
        yaml.push_str(&format!("  L{level:02}:\n    fields:\n      {field}\n"));
    }
    let deep = walker_with(&yaml);
    let mut text = String::new();
    for level in 0..29 {
        text.push_str(&format!(
            "{:indent$}level_{level:04}:\n",
            "",
            indent = level * 2
        ));
    }
    let leaf = |v: &str| format!("{text}{:indent$}leaf_value: {v}\n", "", indent = 29 * 2);
    let rec = frame(&deep, "L00", &leaf("1.0"));
    let rep = frame(&deep, "L00", &leaf("2.0"));
    let value = first_value(&deep, &rec, &rep);
    let full: String = (0..29)
        .map(|l| format!("level_{l:04}."))
        .chain(std::iter::once("leaf_value".to_string()))
        .collect();
    assert_eq!(full.len(), 29 * 11 + 10);
    assert_eq!(value.path, format!("...{}", &full[full.len() - 256..]));
}

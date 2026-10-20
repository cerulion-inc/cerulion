"""Message <-> dict / YAML through the shared Rust codec.

Every expected frame below is hand-packed from the wire rules (32-byte
header, fixed section, offset table, variable data) and every expected
YAML string is written out by hand: no assertion compares the codec with
itself.
"""

import math
import struct

import pytest

import cerulion

SCHEMAS = """
schemas:
  Point:
    fields:
      float64 x: {}
      float64 y: {}
  Named:
    fields:
      string name: {}
      float64[] samples: {}
  Mixed:
    fields:
      uint8 small: {}
      int64 big: {}
      uint64 huge: {}
      float32 ratio: {}
      bool flag: {}
      int32[3] triple: {}
  Empty:
    fields: {}
  Tag:
    fields:
      string_fixed[4] tag: {}
"""


@pytest.fixture(scope="module")
def schemas():
    s = cerulion.SchemaSet.builtins()
    s.add_yaml(SCHEMAS)
    return s


def header(schemas, name, total, fixed, count, timestamp_ns=0):
    return struct.pack(
        "<QIIIIQ",
        schemas.schema_hash(name),
        total,
        32 + fixed,
        count,
        0,
        timestamp_ns,
    )


def point_frame(schemas, x, y, timestamp_ns=0):
    return header(schemas, "Point", 48, 16, 0, timestamp_ns) + struct.pack(
        "<dd", x, y
    )


def named_frame(schemas, name, samples):
    # Offset table of two entries (16 bytes), then the string bytes at
    # payload offset 16, then the float64 array at the next 8-byte boundary.
    raw = name.encode()
    data_start = 16 + len(raw)
    pad = (-data_start) % 8
    samples_off = data_start + pad
    body = struct.pack("<IIII", 16, len(raw), samples_off, 8 * len(samples))
    body += raw + b"\0" * pad + struct.pack(f"<{len(samples)}d", *samples)
    return header(schemas, "Named", 32 + len(body), 0, 2) + body


# ---- happy path -----------------------------------------------------------


def test_yaml_builds_the_hand_packed_frame(schemas):
    frame = schemas.yaml_to_message("Point", "x: 1.5\ny: -2.0\n")
    assert frame == point_frame(schemas, 1.5, -2.0)


def test_yaml_carries_the_timestamp_into_the_header(schemas):
    frame = schemas.yaml_to_message("Point", "x: 1\ny: 2", timestamp_ns=77)
    assert frame == point_frame(schemas, 1.0, 2.0, timestamp_ns=77)


def test_dict_builds_the_hand_packed_frame(schemas):
    frame = schemas.dict_to_message("Named", {"name": "arm", "samples": [0.5, 2.0]})
    assert frame == named_frame(schemas, "arm", [0.5, 2.0])


def test_frame_renders_as_the_hand_written_yaml(schemas):
    text = schemas.message_to_yaml(named_frame(schemas, "arm", [0.5, 2.0]))
    assert text == 'name: "arm"\nsamples: [0.5, 2.0]\n'


def test_frame_renders_as_a_plain_dict(schemas):
    values = schemas.message_to_dict(point_frame(schemas, 1.5, -2.0))
    assert values == {"x": 1.5, "y": -2.0}
    assert list(values) == ["x", "y"]
    assert all(type(v) is float for v in values.values())


def test_nested_builtin_message_round_trips_through_dict(schemas):
    values = {
        "header": {"stamp": {"sec": 3, "nanosec": 4}, "frame_id": "map"},
        "pose": {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }
    frame = schemas.dict_to_message("geometry_msgs/PoseStamped", values)
    assert schemas.message_to_dict(frame) == values
    assert schemas.message_to_yaml(frame) == (
        "header:\n"
        "  stamp:\n"
        "    sec: 3\n"
        "    nanosec: 4\n"
        '  frame_id: "map"\n'
        "pose:\n"
        "  position:\n"
        "    x: 1.0\n"
        "    y: 2.0\n"
        "    z: 3.0\n"
        "  orientation:\n"
        "    x: 0.0\n"
        "    y: 0.0\n"
        "    z: 0.0\n"
        "    w: 1.0\n"
    )


def test_round_trips_are_byte_identical(schemas):
    frame = named_frame(schemas, "wrist", [1.0, -0.25, 1e-7])
    assert schemas.yaml_to_message("Named", schemas.message_to_yaml(frame)) == frame
    assert schemas.dict_to_message("Named", schemas.message_to_dict(frame)) == frame


def test_received_frame_and_bag_record_shapes_are_accepted(schemas):
    frame = point_frame(schemas, 1.0, 2.0)
    assert schemas.message_to_dict(memoryview(frame)) == {"x": 1.0, "y": 2.0}
    assert schemas.message_to_dict(bytearray(frame)) == {"x": 1.0, "y": 2.0}

    class RecordLike:
        raw = frame

    assert schemas.message_to_dict(RecordLike()) == {"x": 1.0, "y": 2.0}


def test_a_strided_or_wide_item_buffer_is_a_type_error(schemas):
    frame = point_frame(schemas, 1.0, 2.0)
    strided = memoryview(bytes(b for byte in frame for b in (byte, 0)))[::2]
    assert strided.tobytes() == frame
    wide = memoryview(frame).cast("I")
    for buffer in (strided, wide):
        for convert in (schemas.message_to_dict, schemas.message_to_yaml):
            with pytest.raises(TypeError) as err:
                convert(buffer)
            assert str(err.value) == (
                "frame must be a contiguous bytes-like object of single-byte "
                "items (bytes, bytearray, memoryview of bytes, np.uint8 array)"
            )


def test_a_bag_record_view_converts_as_its_whole_frame(schemas):
    from cerulion._bag import Record

    record = Record(point_frame(schemas, 1.0, 2.0))
    view = record.view("Point", schemas=schemas)
    assert view.to_dict() == {"x": 1.0, "y": 2.0}
    assert view.to_yaml() == "x: 1.0\ny: 2.0\n"
    values = {
        "header": {"stamp": {"sec": 3, "nanosec": 4}, "frame_id": "map"},
        "pose": {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }
    name = "geometry_msgs/PoseStamped"
    nested = Record(schemas.dict_to_message(name, values)).view(name, schemas=schemas)
    assert nested.to_dict() == values
    # A nested view is still not the frame.
    with pytest.raises(TypeError):
        nested.pose.to_dict()


# ---- every error arm: the text is the codec's, the same for YAML and dict --

POINT_OK = {"x": 1.0, "y": 2.0}
MIXED_OK = {
    "small": 1,
    "big": 2,
    "huge": 3,
    "ratio": 0.5,
    "flag": True,
    "triple": [1, 2, 3],
}


@pytest.mark.parametrize(
    "name, values, yaml_text, kind, message",
    [
        (
            "Point",
            {"x": 1.0, "y": 2.0, "z": 3.0},
            "x: 1.0\ny: 2.0\nz: 3.0\n",
            "YamlUnknownField",
            "z: unknown field; Point has fields: x, y",
        ),
        (
            "Point",
            {"x": 1.0},
            "x: 1.0\n",
            "YamlMissingField",
            "y: missing field required by Point",
        ),
        (
            "Point",
            {"x": "1.0", "y": 2.0},
            'x: "1.0"\ny: 2.0\n',
            "YamlWrongType",
            "x: expected float64, got string",
        ),
        (
            "Mixed",
            {**MIXED_OK, "small": 256},
            "small: 256\nbig: 2\nhuge: 3\nratio: 0.5\nflag: true\ntriple: [1, 2, 3]\n",
            "YamlOutOfRange",
            "small: 256 is out of range for uint8 (0 to 255)",
        ),
        (
            "Mixed",
            {**MIXED_OK, "triple": [1, 2]},
            "small: 1\nbig: 2\nhuge: 3\nratio: 0.5\nflag: true\ntriple: [1, 2]\n",
            "YamlWrongLength",
            "triple: expected exactly 3 elements, got 2",
        ),
        (
            "Point",
            {1: 2.0},
            "1: 2.0\n",
            "YamlWrongType",
            "(message): expected a field name, got integer",
        ),
    ],
)
def test_bad_input_raises_the_codec_error_from_both_yaml_and_dict(
    schemas, name, values, yaml_text, kind, message
):
    with pytest.raises(cerulion.EncodeError) as from_yaml:
        schemas.yaml_to_message(name, yaml_text)
    with pytest.raises(cerulion.EncodeError) as from_dict:
        schemas.dict_to_message(name, values)
    assert str(from_yaml.value) == str(from_dict.value) == message
    assert from_yaml.value.kind == from_dict.value.kind == kind


MIXED_TEXT = "small: 1\nbig: {big}\nhuge: 3\nratio: 0.5\nflag: true\ntriple: [1, 2, 3]\n"


def test_an_integer_past_64_bits_raises_the_yaml_error_from_both_paths(schemas):
    # A dict writes a wide int as its digits, so both paths get the codec's
    # error for the field: out of range up to 128 bits, and past that the
    # YAML parser reads the digits as a float.
    for big, kind, message in [
        (
            -(2**63) - 1,
            "YamlOutOfRange",
            "big: -9223372036854775809 is out of range for int64 "
            "(-9223372036854775808 to 9223372036854775807)",
        ),
        (
            2**64,
            "YamlOutOfRange",
            "big: 18446744073709551616 is out of range for int64 "
            "(-9223372036854775808 to 9223372036854775807)",
        ),
        (2**200, "YamlWrongType", "big: expected int64, got float"),
        (2**1024, "YamlWrongType", "big: expected int64, got string"),
    ]:
        with pytest.raises(cerulion.EncodeError) as from_yaml:
            schemas.yaml_to_message("Mixed", MIXED_TEXT.format(big=big))
        with pytest.raises(cerulion.EncodeError) as from_dict:
            schemas.dict_to_message("Mixed", {**MIXED_OK, "big": big})
        assert str(from_yaml.value) == str(from_dict.value) == message
        assert from_yaml.value.kind == from_dict.value.kind == kind
    with pytest.raises(cerulion.EncodeError) as nested:
        schemas.dict_to_message("Mixed", {**MIXED_OK, "triple": [1, 2**70, 3]})
    assert nested.value.kind == "YamlOutOfRange"
    assert str(nested.value) == (
        "triple[1]: 1180591620717411303424 is out of range for int32 "
        "(-2147483648 to 2147483647)"
    )


def test_an_integer_past_64_bits_fills_a_float_field_from_both_paths(schemas):
    # 2**100 and 2**200 are exact float64 values; 2**130 + 1 rounds to 2**130.
    for big, value in [(2**100, 2.0**100), (2**200, 2.0**200), (2**130 + 1, 2.0**130)]:
        expected = point_frame(schemas, value, 1.0)
        assert schemas.dict_to_message("Point", {"x": big, "y": 1.0}) == expected
        assert schemas.yaml_to_message("Point", f"x: {big}\ny: 1.0\n") == expected
    # Past the float64 range the YAML parser reads the digits as a string.
    for build in (
        lambda: schemas.dict_to_message("Point", {"x": 2**1024, "y": 1.0}),
        lambda: schemas.yaml_to_message("Point", f"x: {2**1024}\ny: 1.0\n"),
    ):
        with pytest.raises(cerulion.EncodeError) as err:
            build()
        assert err.value.kind == "YamlWrongType"
        assert str(err.value) == "x: expected float64, got string"


def test_an_int_too_long_for_decimal_text_names_its_field(schemas):
    # Python refuses decimal text past 4300 digits; the error shows the bit
    # count instead (10**5000 is 16610 bits).
    for raw in (10**5000, -(10**5000)):
        for name, values, path in [
            ("Mixed", {**MIXED_OK, "big": raw}, "big"),
            ("Point", {"x": raw, "y": 1.0}, "x"),
        ]:
            with pytest.raises(cerulion.EncodeError) as err:
                schemas.dict_to_message(name, values)
            assert err.value.kind == "YamlOutOfRange"
            assert str(err.value) == (
                f"{path}: a 16610-bit integer is out of range for every field type"
            )


def test_malformed_yaml_names_the_parse_failure(schemas):
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.yaml_to_message("Point", "x: [1.0,\ny: 2.0\n")
    assert err.value.kind == "YamlSyntax"
    assert str(err.value).startswith("message YAML does not parse: ")


def test_yaml_that_is_not_utf8_is_refused(schemas):
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.yaml_to_message("Point", b"x: \xff\n")
    assert err.value.kind == "YamlInputNotUtf8"
    assert str(err.value) == "message YAML is not valid UTF-8 (invalid byte at offset 3)"
    # A str with a lone surrogate has no UTF-8 form either: the same error,
    # at the surrogate's byte offset, not a UnicodeEncodeError.
    with pytest.raises(cerulion.EncodeError) as lone:
        schemas.yaml_to_message("Point", "x: '\ud800'\n")
    assert lone.value.kind == "YamlInputNotUtf8"
    assert str(lone.value) == "message YAML is not valid UTF-8 (invalid byte at offset 4)"


@pytest.mark.parametrize("text", [5, None, 1.5, ["x: 1"]])
def test_yaml_that_is_not_text_is_a_type_error(schemas, text):
    # `bytes(5)` would be five NUL bytes; anything not str or bytes-like
    # is refused before the codec.
    with pytest.raises(TypeError) as err:
        schemas.yaml_to_message("Point", text)
    assert str(err.value) == (
        f"message YAML must be str or bytes-like, not {type(text).__name__}"
    )


def test_yaml_text_may_be_any_bytes_like(schemas):
    for raw in (b"x: 1\ny: 2\n", bytearray(b"x: 1\ny: 2\n"), memoryview(b"x: 1\ny: 2\n")):
        assert schemas.yaml_to_message("Point", raw) == point_frame(schemas, 1.0, 2.0)


@pytest.mark.parametrize(
    "name, values, yaml_text, kind, message",
    [
        (
            "shape_msgs/SolidPrimitive",
            {"type": 1, "dimensions": [1.0, 2.0, 3.0, 4.0], "polygon": {"points": []}},
            "type: 1\ndimensions: [1.0, 2.0, 3.0, 4.0]\npolygon: {points: []}\n",
            "YamlTooManyElements",
            "dimensions: expected at most 3 elements, got 4",
        ),
        (
            "Tag",
            {"tag": "abcde"},
            'tag: "abcde"\n',
            "YamlStringTooLong",
            "tag: string is 5 bytes, over the bound of 4 bytes",
        ),
        (
            "Tag",
            {"tag": "a\0"},
            'tag: "a\\0"\n',
            "YamlNulInFixedString",
            "tag: a string_fixed value cannot contain a NUL character",
        ),
    ],
)
def test_bound_errors_match_on_both_paths(schemas, name, values, yaml_text, kind, message):
    with pytest.raises(cerulion.EncodeError) as from_yaml:
        schemas.yaml_to_message(name, yaml_text)
    with pytest.raises(cerulion.EncodeError) as from_dict:
        schemas.dict_to_message(name, values)
    assert str(from_yaml.value) == str(from_dict.value) == message
    assert from_yaml.value.kind == from_dict.value.kind == kind


def test_a_yaml_tag_is_refused(schemas):
    # A dict has no way to write a tag; only YAML text reaches this arm.
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.yaml_to_message("Tag", "tag: !foo x\n")
    assert err.value.kind == "YamlTag"
    assert str(err.value) == "tag: YAML tag !foo is not supported"


def test_anchor_and_truncated_display_output_are_refused(schemas):
    with pytest.raises(cerulion.EncodeError) as anchor:
        schemas.yaml_to_message("Point", "x: &a 1.0\ny: *a\n")
    assert anchor.value.kind == "YamlAnchorOrAlias"
    with pytest.raises(cerulion.EncodeError) as cut:
        schemas.yaml_to_message("Named", 'name: "a"\nsamples: [1.0, !truncated 3]\n')
    assert cut.value.kind == "YamlTruncated"


def test_unknown_schema_is_a_schema_error(schemas):
    with pytest.raises(cerulion.SchemaError):
        schemas.dict_to_message("Nope", {})


def test_a_python_value_with_no_message_form_names_its_field(schemas):
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Point", {"x": object(), "y": 1.0})
    assert err.value.kind == "DictUnsupportedValue"
    assert str(err.value) == (
        "x: a object value has no message form; use dict, list, str, int, float, "
        "bool or bytes"
    )


def test_a_tolist_attribute_that_is_not_callable_has_no_message_form(schemas):
    class NotNumpy:
        tolist = None

    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Point", {"x": NotNumpy(), "y": 1.0})
    assert err.value.kind == "DictUnsupportedValue"
    assert str(err.value) == (
        "x: a NotNumpy value has no message form; use dict, list, str, int, float, "
        "bool or bytes"
    )


def test_a_str_with_a_lone_surrogate_names_its_field(schemas):
    for values, path in (
        ({"name": "a\ud800", "samples": []}, "name"),
        ({"name": "a", "samples": [], "\udfff": 1}, "(message)"),
    ):
        with pytest.raises(cerulion.EncodeError) as err:
            schemas.dict_to_message("Named", values)
        assert err.value.kind == "DictNotUtf8"
        assert str(err.value) == (
            f"{path}: the str holds a lone surrogate, which UTF-8 cannot encode"
        )


def test_a_dict_that_contains_itself_is_refused(schemas):
    values = {"x": 1.0}
    values["y"] = values
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Point", values)
    assert err.value.kind == "DictTooDeep"
    # The path is cut to its first 60 characters.
    assert str(err.value) == "y." * 30 + "...: the value nests deeper than 129 levels"


def test_the_deepest_dict_the_yaml_reader_takes_encodes(schemas):
    # The YAML reader takes 128 nested collections: the root dict, 63
    # levels of a one-element array holding one message, and the bottom
    # `float64[1]`. Every message is fixed size: arrays of variable messages
    # stop at a shallower bound of their own. With a NumPy array at the
    # bottom, its `tolist()` is one level more.
    np = pytest.importorskip("numpy")
    deep = cerulion.SchemaSet()
    lines = ["schemas:", "  L0:", "    fields:", "      float64[1] v: {}"]
    for level in range(1, 64):
        lines += [f"  L{level}:", "    fields:", f"      L{level - 1}[1] inner: {{}}"]
    deep.add_yaml("\n".join(lines) + "\n")
    values = {"v": np.array([2.5])}
    plain = {"v": [2.5]}
    text = "v: [2.5]"
    for _ in range(63):
        values = {"inner": [values]}
        plain = {"inner": [plain]}
        text = f"inner: [{{{text}}}]"
    frame = deep.dict_to_message("L63", values)
    assert frame == deep.dict_to_message("L63", plain)
    assert frame == deep.yaml_to_message("L63", text + "\n")
    # One level more is past the reader, as YAML text or as a dict.
    with pytest.raises(cerulion.EncodeError) as err:
        deep.dict_to_message("L63", {"inner": [values]})
    assert err.value.kind == "DictTooDeep"
    with pytest.raises(cerulion.EncodeError) as err:
        deep.yaml_to_message("L63", f"inner: [{{{text}}}]\n")
    assert err.value.kind == "YamlSyntax"


def test_a_dict_whose_yaml_passes_the_cap_is_refused(schemas):
    # One shared 1 MiB `bytes` is 5 MiB of YAML (`255, ` per byte), so
    # sixty references name 300 MiB. The 26 bytes before the list and 51
    # items with their separators come to 267387008 bytes; item 51 passes
    # the 268435456-byte cap.
    chunk = b"\xff" * (1 << 20)
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Named", {"name": "n", "samples": [chunk] * 60})
    assert err.value.kind == "DictTooLarge"
    assert str(err.value) == "samples[51]: the YAML form passes the 268435456-byte cap"
    # A `str` of 70 Mi control characters is 280 MiB of escapes (`\x01`).
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Named", {"name": "\x01" * (70 << 20), "samples": []})
    assert err.value.kind == "DictTooLarge"
    assert str(err.value) == "name: the YAML form passes the 268435456-byte cap"


def test_a_frame_with_an_unknown_hash_is_a_decode_error(schemas):
    frame = bytearray(point_frame(schemas, 1.0, 2.0))
    frame[0:8] = (1).to_bytes(8, "little")
    for convert in (schemas.message_to_dict, schemas.message_to_yaml):
        with pytest.raises(cerulion.DecodeError) as err:
            convert(bytes(frame))
        assert err.value.kind == "UnknownSchemaHash"
        assert str(err.value) == (
            "unknown schema_hash 0x0000000000000001 (no schema in the set matches)"
        )


def test_a_frame_value_with_no_yaml_form_is_a_decode_error(schemas):
    # A `bool[]` element byte that is neither 0 nor 1: the frame is well
    # formed, but the value has no YAML text.
    name = "moveit_msgs/AllowedCollisionEntry"
    frame = schemas.dict_to_message(name, {"enabled": [True]})
    assert frame[32:] == bytes.fromhex("080000000100000001")
    bad = frame[:-1] + b"\x02"
    for convert in (schemas.message_to_dict, schemas.message_to_yaml):
        with pytest.raises(cerulion.DecodeError) as err:
            convert(bad)
        assert err.value.kind == "YamlUnencodable"
        assert str(err.value) == (
            "enabled: bool byte 2 is neither 0 nor 1; the frame has no YAML form"
        )


def unset_fan_out(count):
    """A 40-byte `Outer` frame whose one nested field is unset, and the set
    it belongs to. The unset `Mid` renders as its zero values: `count`
    leaves of 1000 zeros each, none of them in the frame."""
    s = cerulion.SchemaSet()
    s.add_yaml(
        "schemas:\n"
        "  Leaf:\n    fields:\n      int8[1000] v: {}\n"
        "  Mid:\n    fields:\n      string tag: {}\n"
        f"      Leaf[{count}] leaves: {{}}\n"
        "  Outer:\n    fields:\n      Mid m: {}\n"
    )
    # No fixed section, one offset entry: `m` at payload offset 8, length
    # 0 (an empty nested field is the unset idiom).
    frame = header(s, "Outer", 40, 0, 1) + struct.pack("<II", 8, 0)
    return s, frame


def test_the_dict_of_a_frame_is_bounded_by_the_frame_in_values(schemas):
    # A 40-byte frame may become 4 * 40 + 1048576 = 1048736 Python values.
    # Its dict holds 7 values above the leaves (two dicts, a list, three
    # keys and the empty tag) and 1003 per leaf (a dict, its key, a list
    # and 1000 ints): 1045 leaves are 1048142 values, 1046 are 1049145.
    s, frame = unset_fan_out(1045)
    assert s.message_to_dict(frame) == {
        "m": {"tag": "", "leaves": [{"v": [0] * 1000}] * 1045}
    }
    s, frame = unset_fan_out(1046)
    with pytest.raises(cerulion.DecodeError) as err:
        s.message_to_dict(frame)
    assert err.value.kind == "DictTooLarge"
    assert str(err.value) == (
        "the dict of this 40-byte frame passes its 1048736-value budget; "
        "read it with to_yaml()"
    )
    # The YAML form of the same frame is not bounded by the dict budget.
    assert s.message_to_yaml(frame).startswith(
        'm:\n  tag: ""\n  leaves:\n    - v: [0, 0, '
    )


def test_the_dict_of_a_frame_is_bounded_by_the_frame_in_yaml_text(schemas):
    # A 40-byte frame may render 64 * 40 + 16777216 = 16779776 bytes of
    # YAML on its way to a dict. The text opens with 23 bytes (`m:`, the
    # tag line, `leaves:`) and each leaf is one 3010-byte line (`    - v:`,
    # a space, `[0, ` ... `0]` of 3000 bytes, a newline). Leaf 5574 would
    # end at 23 + 5575 * 3010 = 16780773 bytes, past the cap, so it is
    # refused there; the codec's own 268435456-byte cap takes the frame.
    s, frame = unset_fan_out(6000)
    with pytest.raises(cerulion.DecodeError) as err:
        s.message_to_dict(frame)
    assert err.value.kind == "YamlOutputTooLarge"
    assert str(err.value) == (
        "m.leaves[5574].v: the YAML form passes the 16779776-byte output cap"
    )
    assert len(s.message_to_yaml(frame)) == 23 + 6000 * 3010


def test_a_nested_type_missing_from_the_set_is_a_schema_error():
    partial = cerulion.SchemaSet()
    partial.add_yaml("schemas:\n  Holder:\n    fields:\n      Missing m: {}\n")
    for build in (
        lambda: partial.dict_to_message("Holder", {"m": {}}),
        lambda: partial.yaml_to_message("Holder", "m: {}\n"),
    ):
        with pytest.raises(cerulion.SchemaError) as err:
            build()
        assert err.value.kind == "YamlUnresolvedNested"
        assert str(err.value) == "m: nested type Missing is not in the schema set"


def test_messages_nested_past_the_codec_bound_are_a_schema_error():
    deep = cerulion.SchemaSet()
    lines = ["schemas:", "  L0:", "    fields:", "      float64 v: {}"]
    for level in range(1, 66):
        lines += [f"  L{level}:", "    fields:", f"      L{level - 1} inner: {{}}"]
    deep.add_yaml("\n".join(lines) + "\n")
    values = {"v": 2.5}
    for _ in range(65):
        values = {"inner": values}
    with pytest.raises(cerulion.SchemaError) as err:
        deep.dict_to_message("L65", values)
    assert err.value.kind == "YamlTooDeep"
    assert str(err.value).endswith(": messages nest deeper than 64 levels")


def test_arrays_nested_past_the_walker_depth_are_an_encode_error():
    # M0 holds eight levels of message arrays above M8's string[]; the
    # walker decodes no deeper, so a nonempty ninth array is refused.
    chain = cerulion.SchemaSet()
    lines = ["schemas:", "  M8:", "    fields:", "      string[] s: {}"]
    for level in range(7, -1, -1):
        lines += [f"  M{level}:", "    fields:", f"      M{level + 1}[] a: {{}}"]
    chain.add_yaml("\n".join(lines) + "\n")
    values = {"s": ["x"]}
    for _ in range(8):
        values = {"a": [values]}
    message = (
        "a[0].a[0].a[0].a[0].a[0].a[0].a[0].a[0].s: "
        "arrays of strings and messages nest more than 8 deep here"
    )
    with pytest.raises(cerulion.EncodeError) as from_dict:
        chain.dict_to_message("M0", values)
    assert from_dict.value.kind == "YamlArraysTooDeep"
    assert str(from_dict.value) == message
    flow = "a: [{" * 7 + 'a: [{s: ["x"]}]' + "}]" * 7 + "\n"
    with pytest.raises(cerulion.EncodeError) as from_yaml:
        chain.yaml_to_message("M0", flow)
    assert from_yaml.value.kind == "YamlArraysTooDeep"
    assert str(from_yaml.value) == message


def test_a_loan_or_nested_view_does_not_convert(schemas):
    from cerulion._typed import Message

    payload = bytearray(16)
    view = Message(payload, schemas.layout("Point"), schemas)
    with pytest.raises(TypeError):
        view.to_dict()


# ---- edge and adversarial values -----------------------------------------


def test_empty_message(schemas):
    frame = header(schemas, "Empty", 32, 0, 0)
    assert schemas.dict_to_message("Empty", {}) == frame
    assert schemas.yaml_to_message("Empty", "") == frame
    assert schemas.message_to_yaml(frame) == "{}\n"
    assert schemas.message_to_dict(frame) == {}


def test_empty_sequences(schemas):
    frame = named_frame(schemas, "", [])
    assert schemas.dict_to_message("Named", {"name": "", "samples": []}) == frame
    assert schemas.message_to_dict(frame) == {"name": "", "samples": []}


def mixed_frame(schemas, small, big, huge, ratio_bits, flag, triple):
    # Fixed section in declaration order with natural alignment:
    # u8 @0, i64 @8, u64 @16, f32 @24, bool @28, i32[3] @32..44, size 48.
    body = struct.pack("<B7xqQIB3x3i4x", small, big, huge, ratio_bits, flag, *triple)
    return header(schemas, "Mixed", 32 + len(body), len(body), 0) + body


def test_integer_extremes_round_trip(schemas):
    values = {
        "small": 255,
        "big": -(2**63),
        "huge": 2**64 - 1,
        "ratio": 0.5,
        "flag": False,
        "triple": [-(2**31), 0, 2**31 - 1],
    }
    frame = schemas.dict_to_message("Mixed", values)
    assert frame == mixed_frame(
        schemas, 255, -(2**63), 2**64 - 1, 0x3F000000, 0, [-(2**31), 0, 2**31 - 1]
    )
    assert schemas.message_to_dict(frame) == values


def test_canonical_nan_and_infinities_round_trip_bit_exact(schemas):
    for bits, text in ((0x7FC00000, ".nan"), (0x7F800000, ".inf"), (0xFF800000, "-.inf")):
        frame = mixed_frame(schemas, 0, 0, 0, bits, 1, [0, 0, 0])
        assert f"ratio: {text}\n" in schemas.message_to_yaml(frame)
        values = schemas.message_to_dict(frame)
        assert math.isnan(values["ratio"]) or math.isinf(values["ratio"])
        assert schemas.dict_to_message("Mixed", values) == frame
        assert schemas.yaml_to_message("Mixed", schemas.message_to_yaml(frame)) == frame


def test_any_other_nan_reads_back_as_the_canonical_nan(schemas):
    # YAML has one `.nan`: a NaN's sign and payload are not kept.
    canonical = mixed_frame(schemas, 0, 0, 0, 0x7FC00000, 1, [0, 0, 0])
    for bits in (0xFFC00000, 0x7FC00001, 0x7F800001):
        frame = mixed_frame(schemas, 0, 0, 0, bits, 1, [0, 0, 0])
        assert schemas.dict_to_message("Mixed", schemas.message_to_dict(frame)) == canonical
        assert schemas.yaml_to_message("Mixed", schemas.message_to_yaml(frame)) == canonical


def test_float32_reads_back_to_the_same_bits(schemas):
    # 0.1 as float32 is 0x3DCCCCCD; the dict holds its shortest form.
    frame = mixed_frame(schemas, 0, 0, 0, 0x3DCCCCCD, 0, [0, 0, 0])
    values = schemas.message_to_dict(frame)
    assert values["ratio"] == 0.1
    assert schemas.dict_to_message("Mixed", values) == frame


def test_non_ascii_and_escaped_strings_round_trip(schemas):
    name = 'café 雪 \U0001F600 "q" \\ \n\t \u0085\x00 \x7f \u2028 \ufeff \ufffe \uffff'
    frame = schemas.dict_to_message("Named", {"name": name, "samples": [1.0]})
    assert frame == named_frame(schemas, name, [1.0])
    assert schemas.message_to_dict(frame)["name"] == name
    assert schemas.yaml_to_message("Named", schemas.message_to_yaml(frame)) == frame


def test_strings_that_look_like_other_values_stay_strings(schemas):
    for name in ("true", "1", "null", "~", ".nan", "- a", "{}", "#x"):
        frame = schemas.dict_to_message("Named", {"name": name, "samples": []})
        assert schemas.message_to_dict(frame)["name"] == name


def test_numpy_and_bytes_values_are_accepted(schemas):
    np = pytest.importorskip("numpy")
    frame = schemas.dict_to_message(
        "Named", {"name": "n", "samples": np.array([0.5, 2.0])}
    )
    assert frame == named_frame(schemas, "n", [0.5, 2.0])
    image = schemas.dict_to_message(
        "sensor_msgs/CompressedImage",
        {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "format": "png",
            "data": b"\x00\x01\xff",
        },
    )
    assert schemas.message_to_dict(image)["data"] == [0, 1, 255]


def test_deeply_nested_types_round_trip(schemas):
    deep = cerulion.SchemaSet()
    lines = ["schemas:", "  L0:", "    fields:", "      float64 v: {}"]
    for level in range(1, 12):
        lines += [f"  L{level}:", "    fields:", f"      L{level - 1} inner: {{}}"]
    deep.add_yaml("\n".join(lines) + "\n")
    values = {"v": 2.5}
    for _ in range(11):
        values = {"inner": values}
    frame = deep.dict_to_message("L11", values)
    assert frame == header(deep, "L11", 40, 8, 0) + struct.pack("<d", 2.5)
    assert deep.message_to_dict(frame) == values


def test_every_builtin_type_round_trips(schemas):
    # One message of every type: the YAML of its smallest frame (every
    # variable field empty) read back is a canonical frame, and dict and
    # YAML both carry that frame back byte for byte.
    count = 0
    refused = {}
    for name in schemas.names():
        try:
            zero = schemas._native.begin_frame(
                name, [0] * len(schemas.layout(name).variable_fields), 0
            )
            text = schemas.message_to_yaml(zero)
            frame = schemas.yaml_to_message(name, text)
            again = schemas.message_to_yaml(frame)
        except cerulion.SchemaError as err:
            assert err.kind == "YamlUnsupportedField", name
            refused[name] = str(err)
            continue
        assert again == text, name
        assert schemas.yaml_to_message(name, schemas.message_to_yaml(frame)) == frame, name
        assert schemas.dict_to_message(name, schemas.message_to_dict(frame)) == frame, name
        count += 1
    # A gap fails here instead of being skipped quietly.
    assert refused == {}
    assert count >= 250


STAMP = {"sec": 7, "nanosec": 8}
POSE = {
    "position": {"x": 1.0, "y": -2.0, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}


@pytest.mark.parametrize(
    "name, values",
    [
        (
            "geometry_msgs/PoseArray",
            {"header": {"stamp": STAMP, "frame_id": "map"}, "poses": [POSE, POSE]},
        ),
        (
            "sensor_msgs/JointState",
            {
                "header": {"stamp": STAMP, "frame_id": ""},
                "name": ["shoulder", "elbow"],
                "position": [0.25, -1.5],
                "velocity": [],
                "effort": [3.0],
            },
        ),
        (
            "trajectory_msgs/JointTrajectory",
            {
                "header": {"stamp": STAMP, "frame_id": "base"},
                "joint_names": ["a"],
                "points": [
                    {
                        "positions": [1.0],
                        "velocities": [],
                        "accelerations": [],
                        "effort": [],
                        "time_from_start": {"sec": 1, "nanosec": 0},
                    },
                    {
                        "positions": [2.0, 3.0],
                        "velocities": [0.5],
                        "accelerations": [],
                        "effort": [],
                        "time_from_start": {"sec": 2, "nanosec": 5},
                    },
                ],
            },
        ),
        (
            "moveit_msgs/AllowedCollisionEntry",
            {"enabled": [True, False, True]},
        ),
        (
            "shape_msgs/SolidPrimitive",
            {"type": 1, "dimensions": [1.0, 2.0, 3.0], "polygon": {"points": []}},
        ),
    ],
)
def test_nonempty_sequences_of_messages_round_trip(schemas, name, values):
    # Sequences of nested messages, of strings and of nested messages that
    # hold sequences themselves: dict and YAML carry each frame back byte
    # for byte, and the dict reads back as written.
    frame = schemas.dict_to_message(name, values)
    assert schemas.message_to_dict(frame) == values
    assert schemas.dict_to_message(name, schemas.message_to_dict(frame)) == frame
    assert schemas.yaml_to_message(name, schemas.message_to_yaml(frame)) == frame


# ---- determinism ----------------------------------------------------------


def test_two_processes_give_bit_identical_output(schemas, tmp_path):
    script = tmp_path / "run.py"
    script.write_text(
        "import sys, cerulion\n"
        f"s = cerulion.SchemaSet.builtins(); s.add_yaml({SCHEMAS!r})\n"
        "f = s.dict_to_message('Named', {'name': 'd\\u00e9t', 'samples': [0.1, -2.5, 1e-300]})\n"
        "sys.stdout.buffer.write(f + s.message_to_yaml(f).encode())\n"
    )
    import subprocess
    import sys

    runs = [
        subprocess.run([sys.executable, str(script)], check=True, capture_output=True).stdout
        for _ in range(2)
    ]
    assert runs[0] == runs[1]
    frame = named_frame(schemas, "dét", [0.1, -2.5, 1e-300])
    assert runs[0] == frame + b'name: "d\xc3\xa9t"\nsamples: [0.1, -2.5, 1e-300]\n'

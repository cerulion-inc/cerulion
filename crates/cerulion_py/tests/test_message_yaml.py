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


def test_an_integer_past_64_bits_is_refused_by_the_parser(schemas):
    # The YAML reader holds integers in 64 bits, so a longer one is a parse
    # failure on both paths, not a value the codec sees.
    for raw in ("-9223372036854775809", "18446744073709551616"):
        with pytest.raises(cerulion.EncodeError) as from_yaml:
            schemas.yaml_to_message("Mixed", f"small: 1\nbig: {raw}\n")
        with pytest.raises(cerulion.EncodeError) as from_dict:
            schemas.dict_to_message("Mixed", {**MIXED_OK, "big": int(raw)})
        assert from_yaml.value.kind == from_dict.value.kind == "YamlSyntax"


def test_malformed_yaml_names_the_parse_failure(schemas):
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.yaml_to_message("Point", "x: [1.0,\ny: 2.0\n")
    assert err.value.kind == "YamlSyntax"
    assert str(err.value).startswith("message YAML does not parse: ")


def test_yaml_that_is_not_utf8_is_refused(schemas):
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.yaml_to_message("Point", b"x: \xff\n")
    assert str(err.value) == "message YAML is not valid UTF-8 (invalid byte at offset 3)"


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
    assert str(err.value) == (
        "x: a object value has no message form; use dict, list, str, int, float, "
        "bool or bytes"
    )


def test_a_dict_that_contains_itself_is_refused(schemas):
    values = {"x": 1.0}
    values["y"] = values
    with pytest.raises(cerulion.EncodeError) as err:
        schemas.dict_to_message("Point", values)
    assert "nests deeper than 128 levels" in str(err.value)


def test_a_frame_with_an_unknown_hash_is_a_schema_mismatch(schemas):
    frame = bytearray(point_frame(schemas, 1.0, 2.0))
    frame[0:8] = (1).to_bytes(8, "little")
    with pytest.raises(cerulion.CerulionError):
        schemas.message_to_dict(bytes(frame))


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


def test_nan_and_infinities_round_trip_bit_exact(schemas):
    for bits, text in ((0x7FC00000, ".nan"), (0x7F800000, ".inf"), (0xFF800000, "-.inf")):
        frame = mixed_frame(schemas, 0, 0, 0, bits, 1, [0, 0, 0])
        assert f"ratio: {text}\n" in schemas.message_to_yaml(frame)
        values = schemas.message_to_dict(frame)
        assert math.isnan(values["ratio"]) or math.isinf(values["ratio"])
        assert schemas.dict_to_message("Mixed", values) == frame
        assert schemas.yaml_to_message("Mixed", schemas.message_to_yaml(frame)) == frame


def test_float32_reads_back_to_the_same_bits(schemas):
    # 0.1 as float32 is 0x3DCCCCCD; the dict holds its shortest form.
    frame = mixed_frame(schemas, 0, 0, 0, 0x3DCCCCCD, 0, [0, 0, 0])
    values = schemas.message_to_dict(frame)
    assert values["ratio"] == 0.1
    assert schemas.dict_to_message("Mixed", values) == frame


def test_non_ascii_and_escaped_strings_round_trip(schemas):
    name = 'café 雪 \U0001F600 "q" \\ \n\t \u0085\x00'
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
    for name in schemas.names():
        try:
            zero = schemas._native.begin_frame(
                name, [0] * len(schemas.layout(name).variable_fields), 0
            )
            text = schemas.message_to_yaml(zero)
            frame = schemas.yaml_to_message(name, text)
            again = schemas.message_to_yaml(frame)
        except cerulion.SchemaError:
            # A field type the YAML form does not cover (`bool[]` inside a
            # nested message): the codec refuses it by name.
            continue
        assert again == text, name
        assert schemas.yaml_to_message(name, schemas.message_to_yaml(frame)) == frame, name
        assert schemas.dict_to_message(name, schemas.message_to_dict(frame)) == frame, name
        count += 1
    assert count >= 250


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

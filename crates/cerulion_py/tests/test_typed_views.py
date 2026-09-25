import sys

import numpy as np
import pytest

import cerulion

from conftest import macos_shared_mapping, shm_mappings, unique_topic


SCHEMA = """\
schemas:
  Vector3:
    fields:
      float32 x: {}
      float32 y: {}
      float32 z: {}
  Probe:
    fields:
      bool boolean: {}
      int8 i8: {}
      uint8 u8: {}
      int16 i16: {}
      uint16 u16: {}
      int32 i32: {}
      uint32 u32: {}
      int64 i64: {}
      uint64 u64: {}
      float32 f32: {}
      float64 f64: {}
      string_fixed[8] label: {}
      float32[3] vector: {}
      Vector3 nested: {}
      Vector3[2] nested_array: {}
      float32[] floats: {}
      float64[] doubles: {}
      uint16[] words: {}
      int64[] longs: {}
      string name: {}
      uint8[] values: {}
"""


def test_materialized_publish_and_readonly_view(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-view")
    publisher = session.publisher(topic, schema="Probe", schemas=schemas)
    subscriber = session.subscriber(topic, schema="Probe", schemas=schemas)

    payload = {
        "boolean": True,
        "i8": -8,
        "u8": 8,
        "i16": -1600,
        "u16": 1600,
        "i32": -32000,
        "u32": 32000,
        "i64": -64000,
        "u64": 64000,
        "f32": 1.25,
        "f64": -2.5,
        "label": "tag",
        "vector": [1.0, 2.0, 3.0],
        "nested": {"x": 4.0, "y": 5.0, "z": 6.0},
        "nested_array": [[7.0, 8.0, 9.0], [10.0, 11.0, 12.0]],
        "floats": [1.5, 2.5],
        "doubles": [-1.5, 3.5],
        "words": [7, 11, 13],
        "longs": [-17, 19],
        "name": "abc",
        "values": [3, 5, 8],
    }
    publisher.publish(payload)
    frame = subscriber.receive(1000)
    assert frame is not None
    message = frame.view()
    assert message is frame.view()
    for field, value in payload.items():
        actual = getattr(message, field)
        if isinstance(value, list):
            if field == "nested_array":
                np.testing.assert_array_equal(actual["x"], [7.0, 10.0])
                np.testing.assert_array_equal(actual["y"], [8.0, 11.0])
                np.testing.assert_array_equal(actual["z"], [9.0, 12.0])
            else:
                np.testing.assert_array_equal(actual, value)
        elif isinstance(value, dict):
            assert actual.x == value["x"]
            assert actual.y == value["y"]
            assert actual.z == value["z"]
        else:
            assert actual == value
    assert not message.values.flags.writeable
    assert not message.vector.flags.writeable
    nested_record = np.asarray(message.nested._record())
    assert not nested_record.flags.writeable
    ranges = shm_mappings() if sys.platform.startswith("linux") else None
    for array in (message.values, message.vector, message.floats, nested_record):
        ptr = np.asarray(array).__array_interface__["data"][0]
        if ranges is not None:
            assert any(start <= ptr < end for start, end in ranges)
        else:
            assert macos_shared_mapping(ptr), f"typed view at {ptr:#x} is not in shared memory"
    copied = message.copy()
    assert copied["boolean"] is True
    assert copied["i8"] == -8
    assert copied["u64"] == 64000
    np.testing.assert_array_equal(copied["vector"], [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(copied["values"], [3, 5, 8])
    assert copied["name"] == "abc"
    assert copied["values"].flags.writeable
    assert copied["values"].base is None
    frame.release()


def test_raw_frame_requires_explicit_typed_view(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Probe:\n    fields:\n      uint32 id: {}\n")
    topic = unique_topic("typed-explicit-view")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    raw_sub = session.subscriber(topic)
    pub.publish({"id": 7})
    frame = raw_sub.receive(1000)
    assert frame is not None
    with np.testing.assert_raises(ValueError):
        frame.view()
    assert frame.view(schemas, "Probe").id == 7
    frame.release()


def test_typed_bool_dynamic_array_roundtrip_and_raw_bytes(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Flags:\n    fields:\n      bool[] flags: {}\n      string tag: {}\n"
    )
    topic = unique_topic("typed-bool-array")
    pub = session.publisher(topic, schema="Flags", schemas=schemas)
    sub = session.subscriber(topic, schema="Flags", schemas=schemas)
    with pub.loan(flags=3, tag=2) as message:
        message.flags[:] = [True, False, True]
        message.tag = "go"
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    np.testing.assert_array_equal(message.flags, [True, False, True])
    assert message.flags.dtype == np.dtype("?")
    # flags is the first variable field: data_floor is the 16-byte offset
    # table (no fixed section), so the payload bytes are \x01\x00\x01.
    assert bytes(frame.payload[16:19]) == b"\x01\x00\x01"
    assert message.tag == "go"
    frame.release()


def test_nested_dynamic_array_decodes_and_copy_deep_copies(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Vec3:\n    fields:\n      float32 x: {}\n      float32 y: {}\n      float32 z: {}\n"
        "  Arr:\n    fields:\n      Vec3[] pts: {}\n"
    )
    topic = unique_topic("typed-nested-array")
    pub = session.publisher(topic, schema="Arr", schemas=schemas)
    sub = session.subscriber(topic, schema="Arr", schemas=schemas)
    import struct

    # A fixed-element nested array is back-to-back fixed sections with no
    # count prefix (canonical element framing).
    body = struct.pack("<fff", 1.0, 2.0, 3.0) + struct.pack("<fff", 4.0, 5.0, 6.0)
    with pub.loan(pts=len(body)) as message:
        message.pts = body
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    pts = message.pts
    assert [p.x for p in pts] == [1.0, 4.0]
    assert [p.y for p in pts] == [2.0, 5.0]
    assert [p.z for p in pts] == [3.0, 6.0]
    copied = message.copy()
    frame.release()
    # The copy must survive the frame's release AND own its data.
    assert [p["x"] for p in copied["pts"]] == [1.0, 4.0]
    assert [p["z"] for p in copied["pts"]] == [3.0, 6.0]


def test_string_dynamic_array_decodes_from_counted_framing(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Names:\n    fields:\n      string[] names: {}\n")
    topic = unique_topic("typed-string-array")
    pub = session.publisher(topic, schema="Names", schemas=schemas)
    sub = session.subscriber(topic, schema="Names", schemas=schemas)
    import struct

    # Canonical counted framing: u32 count + per element (u32 len, bytes).
    blob = struct.pack("<I", 2) + struct.pack("<I", 3) + b"bob" + struct.pack("<I", 4) + b"alix"
    with pub.loan(names=len(blob)) as message:
        message.names = blob
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view().names == ["bob", "alix"]
    frame.release()


def test_string_dynamic_array_rejects_element_wise_publish(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Names2:\n    fields:\n      string[] names: {}\n")
    pub = session.publisher(
        unique_topic("typed-string-array-elements"), schema="Names2", schemas=schemas
    )
    with pytest.raises(cerulion.EncodeError, match="pre-framed bytes"):
        pub.publish({"names": ["a", "b"]})


def test_fixed_string_invalid_utf8_is_decode_error(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  S:\n    fields:\n      string_fixed[4] label: {}\n")
    topic = unique_topic("typed-fixed-utf8")
    pub = session.publisher(topic, schema="S", schemas=schemas)
    sub = session.subscriber(topic, schema="S", schemas=schemas)
    with pub.loan() as message:
        message._payload[0] = 0xFF
    frame = sub.receive(1000)
    assert frame is not None
    with pytest.raises(cerulion.DecodeError, match="invalid UTF-8"):
        _ = frame.view().label
    frame.release()


def test_loan_variable_nested_field_returns_writable_raw_view(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Inner:\n    fields:\n      uint32 a: {}\n      uint8[] b: {}\n"
        "  Outer:\n    fields:\n      uint32 id: {}\n      Inner n: {}\n"
    )
    topic = unique_topic("typed-loan-nested-var")
    pub = session.publisher(topic, schema="Outer", schemas=schemas)
    sub = session.subscriber(topic, schema="Outer", schemas=schemas)
    # Inner is variable: framed body = 4 (fixed a) + 8 (offset table) + 3 (b).
    import struct

    with pub.loan(n=15) as message:
        inner = message.n
        assert isinstance(inner, memoryview)
        assert len(inner) == 15 and not inner.readonly
        inner[0:4] = struct.pack("<I", 1)
        inner[4:12] = struct.pack("<II", 12, 3)
        inner[12:15] = b"\x07\x08\x09"
        message.id = 5
        del inner  # live export: releasing it before block exit lets commit proceed
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.id == 5
    assert view.n.a == 1
    desc = view.n._resolved_fields["b"]
    assert bytes(view._payload[desc[1] : desc[1] + desc[2]]) == b"\x07\x08\x09"
    frame.release()


def test_variable_fields_inside_resolved_nested_message_decode(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Inner:\n    fields:\n      uint32 a: {}\n      uint8[] b: {}\n"
        "      string s: {}\n"
        "  Outer2:\n    fields:\n      uint32 id: {}\n      Inner n: {}\n"
    )
    topic = unique_topic("typed-nested-var-decode")
    pub = session.publisher(topic, schema="Outer2", schemas=schemas)
    sub = session.subscriber(topic, schema="Outer2", schemas=schemas)
    pub.publish({"id": 1, "n": {"a": 5, "b": [9, 8, 7], "s": "hi"}})
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.id == 1
    assert view.n.a == 5
    assert bytes(view.n.b) == b"\x09\x08\x07"
    assert np.asarray(view.n.b).dtype == np.uint8
    assert view.n.s == "hi"
    frame.release()

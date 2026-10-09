import gc
import struct

import numpy as np
import pytest

import cerulion
from cerulion._typed import _encode_message

from conftest import unique_topic


def _strings(*items):
    """Canonical string[] body: u32 count, then per element u32 len + bytes."""
    out = struct.pack("<I", len(items))
    for item in items:
        out += struct.pack("<I", len(item)) + item
    return out


def _pair(session, schemas, schema, name):
    topic = unique_topic(name)
    pub = session.publisher(topic, schema=schema, schemas=schemas)
    sub = session.subscriber(topic, schema=schema, schemas=schemas)
    return pub, sub


def test_released_typed_views_return_their_slot_without_cyclic_gc(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  GcProbe:\n    fields:\n      uint32 id: {}\n      uint8[] values: {}\n")
    pub, sub = _pair(session, schemas, "GcProbe", "typed-gc")
    was_enabled = gc.isenabled()
    gc.disable()
    held = []
    try:
        for i in range(3 * sub.max_borrowed_samples + 1):
            pub.publish({"id": i, "values": [i, i + 1]})
            frame = sub.receive(1000)
            assert frame is not None, f"iteration {i}"
            message = frame.view()
            assert message.id == i
            assert bytes(message.values) == bytes([i, i + 1])
            frame.release()
            held.append((frame, message))
            if len(held) > sub.max_borrowed_samples:
                held.pop(0)
    finally:
        if was_enabled:
            gc.enable()


def test_release_detaches_views_from_every_schema_binding(session):
    yaml = "schemas:\n  TwoSets:\n    fields:\n      uint32 id: {}\n"
    first = cerulion.SchemaSet()
    first.add_yaml(yaml)
    second = cerulion.SchemaSet()
    second.add_yaml(yaml)
    pub, sub = _pair(session, first, "TwoSets", "typed-two-sets")
    kept = []
    for i in range(3 * sub.max_borrowed_samples + 1):
        pub.publish({"id": i})
        frame = sub.receive(1000)
        assert frame is not None, f"iteration {i}"
        views = (frame.view(first, "TwoSets"), frame.view(second, "TwoSets"))
        assert [view.id for view in views] == [i, i]
        frame.release()
        kept.append(views)
    for views in kept:
        for view in views:
            with pytest.raises(cerulion.ReleasedFrame):
                view.id


def test_view_on_a_released_frame_raises_even_when_cached(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Cached:\n    fields:\n      uint32 id: {}\n")
    pub, sub = _pair(session, schemas, "Cached", "typed-cached")
    pub.publish({"id": 9})
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    assert message.id == 9
    frame.release()
    with pytest.raises(cerulion.ReleasedFrame):
        frame.view()
    with pytest.raises(cerulion.ReleasedFrame):
        message.id


def test_bare_nested_names_resolve_like_the_core_walker(session):
    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg("int32 sec\nuint32 nanosec\n", "builtin_interfaces/Time")
    schemas.add_rosmsg("Time stamp\nstring frame_id\n", "std_msgs/Header")
    schemas.add_rosmsg("Header header\nuint32 id\n", "demo/Stamped")
    pub, sub = _pair(session, schemas, "demo/Stamped", "typed-bare-nested")
    pub.publish(
        {"header": {"stamp": {"sec": -3, "nanosec": 250}, "frame_id": "map"}, "id": 7}
    )
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.id == 7
    assert view.header.stamp.sec == -3
    assert view.header.stamp.nanosec == 250
    assert view.header.frame_id == "map"
    frame.release()


def test_explicit_package_never_falls_back_to_another_package():
    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg("int32 sec\n", "builtin_interfaces/Time")
    schemas.add_yaml(
        "schemas:\n  Holder:\n    fields:\n      other/Time t: {}\n      string s: {}\n"
    )
    with pytest.raises(cerulion.SchemaError, match="unknown nested schema other/Time"):
        _encode_message(schemas, "Holder", {"t": {"sec": 1}, "s": "x"}, 0)


def test_string_array_with_invalid_utf8_raises_decode_error(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Names:\n    fields:\n      string[] names: {}\n")
    pub, sub = _pair(session, schemas, "Names", "typed-names")
    pub.publish({"names": _strings(b"ok", b"hi")})
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view().names == ["ok", "hi"]
    frame.release()
    pub.publish({"names": _strings(b"ok", b"\xff")})
    frame = sub.receive(1000)
    assert frame is not None
    with pytest.raises(cerulion.DecodeError, match="invalid UTF-8"):
        frame.view().names
    frame.release()


def test_fixed_array_of_strings_publishes_and_decodes(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Pair:\n    fields:\n      string[2] names: {}\n      uint8 tag: {}\n")
    pub, sub = _pair(session, schemas, "Pair", "typed-fixed-strings")
    pub.publish({"names": _strings(b"left", b"right"), "tag": 4})
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.names == ["left", "right"]
    assert view.tag == 4
    frame.release()
    with pytest.raises(cerulion.EncodeError, match="pre-framed bytes"):
        pub.publish({"names": ["left", "right"], "tag": 4})


def test_fixed_array_inside_a_variable_nested_message_decodes(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Inner3:\n    fields:\n      float32[3] v: {}\n      string s: {}\n"
        "  Outer3:\n    fields:\n      Inner3 n: {}\n"
    )
    pub, sub = _pair(session, schemas, "Outer3", "typed-nested-fixed-array")
    pub.publish({"n": {"v": [1.5, -2.0, 4.25], "s": "x"}})
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    np.testing.assert_array_equal(view.n.v, np.array([1.5, -2.0, 4.25], dtype="<f4"))
    assert view.n.s == "x"
    frame.release()


def test_publish_frame_rejects_a_foreign_schema_hash(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  PointA:\n    fields:\n      float64 x: {}\n"
        "  PoseA:\n    fields:\n      float64 y: {}\n"
    )
    topic = unique_topic("typed-foreign-frame")
    pub = session.publisher(topic, schema="PointA", schemas=schemas)
    foreign = _encode_message(schemas, "PoseA", {"y": 1.0}, 0)
    with pytest.raises(cerulion.SchemaMismatch):
        pub.publish_frame(bytes(foreign))
    assert pub.sequence == 0


def test_fixed_nested_dict_must_be_complete_and_known():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  V3:\n    fields:\n      float32 x: {}\n      float32 y: {}\n"
        "  Holds:\n    fields:\n      V3 v: {}\n"
    )
    with pytest.raises(cerulion.EncodeError, match="missing field\\(s\\): y"):
        _encode_message(schemas, "Holds", {"v": {"x": 1.0}}, 0)
    with pytest.raises(cerulion.EncodeError, match="extra field\\(s\\): w"):
        _encode_message(schemas, "Holds", {"v": {"x": 1.0, "y": 2.0, "w": 3.0}}, 0)
    frame = _encode_message(schemas, "Holds", {"v": {"x": 1.0, "y": 2.0}}, 0)
    assert bytes(frame[cerulion.WIRE_HEADER_SIZE:]) == struct.pack("<ff", 1.0, 2.0)


NESTED_ARRAY_SCHEMA = """\
schemas:
  Cell:
    fields:
      uint8 a: {}
      uint32 b: {}
  Grid:
    fields:
      Cell[2] cells: {}
      uint8 tag: {}
"""


def test_fixed_nested_array_values_are_validated_like_scalars(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(NESTED_ARRAY_SCHEMA)
    pub, sub = _pair(session, schemas, "Grid", "typed-nested-array-validate")
    for bad in ([[1.5, 1], [2, 2]], [[256, 1], [0, 0]], [[1, -1], [0, 0]]):
        with pytest.raises(cerulion.EncodeError, match="cells"):
            pub.publish({"cells": bad, "tag": 1})
    with pytest.raises(cerulion.EncodeError, match="shape"):
        pub.publish({"cells": [[1, 2]], "tag": 1})
    with pytest.raises(cerulion.EncodeError, match="cells"):
        with pub.loan() as message:
            message.cells = [[1.5, 1], [2, 2]]
    assert pub.sequence == 0
    pub.publish({"cells": [[255, 1], [2, 4_000_000_000]], "tag": 1})
    frame = sub.receive(1000)
    assert frame is not None
    cells = frame.view().cells
    np.testing.assert_array_equal(cells["a"], [255, 2])
    np.testing.assert_array_equal(cells["b"], [1, 4_000_000_000])
    frame.release()


def test_fixed_nested_array_accepts_copy_and_dict_forms(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(NESTED_ARRAY_SCHEMA)
    pub, sub = _pair(session, schemas, "Grid", "typed-nested-array-forms")
    pub.publish({"cells": [[1, 10], [2, 20]], "tag": 3})
    frame = sub.receive(1000)
    assert frame is not None
    received = frame.view()
    copied = received.copy()
    assert copied["cells"].dtype.names == ("a", "b")
    forwarded_frame = bytes(frame.payload)
    frame.release()
    # The structured array `copy()` returns and a list of dicts republish
    # the same bytes (one receive per publish: the subscriber depth is 1).
    for payload in (copied, {"cells": [{"a": 1, "b": 10}, {"a": 2, "b": 20}], "tag": 3}):
        pub.publish(payload)
        frame = sub.receive(1000)
        assert frame is not None
        assert bytes(frame.payload) == forwarded_frame
        frame.release()
    with pytest.raises(cerulion.EncodeError, match="missing field\\(s\\): b"):
        pub.publish({"cells": [{"a": 1}, {"a": 2, "b": 20}], "tag": 3})
    with pytest.raises(cerulion.EncodeError, match="cells.b"):
        pub.publish({"cells": [{"a": 1, "b": 1.5}, {"a": 2, "b": 20}], "tag": 3})


def test_received_message_forwards_byte_identically(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Vec3:\n    fields:\n      float32 x: {}\n      float32 y: {}\n"
        "      float32 z: {}\n"
        "  Inner:\n    fields:\n      uint32 a: {}\n      string s: {}\n"
        "  Bundle:\n    fields:\n      uint32 id: {}\n      Vec3[2] fixed: {}\n"
        "      string[] names: {}\n      Vec3[] pts: {}\n      Inner inner: {}\n"
        "      uint16[] words: {}\n      string label: {}\n"
    )
    pub, sub = _pair(session, schemas, "Bundle", "typed-forward-message")
    pts = struct.pack("<fff", 1.0, 2.0, 3.0) + struct.pack("<fff", 4.0, 5.0, 6.0)
    pub.publish(
        {
            "id": 11,
            "fixed": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            "names": _strings(b"left", b"right"),
            "pts": pts,
            "inner": {"a": 9, "s": "in"},
            "words": [7, 8],
            "label": "L",
        }
    )
    first = sub.receive(1000)
    assert first is not None
    original = bytes(first.payload)
    view = first.view()
    assert view.names == ["left", "right"]
    assert [p.y for p in view.pts] == [2.0, 5.0]
    # Forward the received Message as-is: string[], Vec3[] and the variable
    # nested message travel as the bytes they arrived as.
    pub.publish(view)
    first.release()
    second = sub.receive(1000)
    assert second is not None
    assert bytes(second.payload) == original
    forwarded = second.view()
    assert forwarded.id == 11
    assert forwarded.inner.s == "in"
    assert forwarded.names == ["left", "right"]
    second.release()
    with pytest.raises(cerulion.ReleasedFrame):
        pub.publish(view)


def test_loan_pre_framed_field_rejects_a_non_buffer(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Names3:\n    fields:\n      string[] names: {}\n")
    pub, _ = _pair(session, schemas, "Names3", "typed-loan-prefamed-int")
    with pytest.raises(cerulion.EncodeError, match="bytes-like object, not int"):
        with pub.loan(names=3) as message:
            message.names = 3
    with pytest.raises(cerulion.EncodeError, match="bytes-like object, not str"):
        with pub.loan(names=3) as message:
            message.names = "abc"
    assert pub.sequence == 0


def test_empty_nested_body_fields_are_absent_not_the_parents_bytes(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Inner4:\n    fields:\n      uint32 a: {}\n      string s: {}\n"
        "  Outer4:\n    fields:\n      uint32 id: {}\n      Inner4 inner: {}\n"
    )
    pub, sub = _pair(session, schemas, "Outer4", "typed-empty-nested")
    # `inner` left at its default zero length: nothing of Inner4 was sent.
    with pub.loan() as message:
        message.id = 7
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.id == 7
    with pytest.raises(cerulion.DecodeError, match="'a' is absent"):
        view.inner.a
    with pytest.raises(cerulion.DecodeError, match="absent"):
        view.inner.copy()
    with pytest.raises(AttributeError):
        view.inner.nope
    frame.release()

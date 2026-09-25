import array
import struct

import pytest

import cerulion

from conftest import unique_topic


SCHEMA = """\
schemas:
  Probe:
    fields:
      uint32 id: {}
      uint8[] values: {}
      string name: {}
"""

DECODE_SCHEMA = """\
schemas:
  DecodeProbe:
    fields:
      uint32 id: {}
      uint16[] words: {}
      string name: {}
"""


def _wire_frame(schema_hash, body, count=2, offset=36):
    total = 32 + len(body)
    header = struct.pack("<QIIIIQ", schema_hash, total, offset, count, 0, 0)
    return header + body


def _decode_probe(session, schemas, body, *, count=2, offset=36):
    topic = unique_topic("typed-decode")
    publisher = session.publisher(
        topic, schema_hash=schemas.schema_hash("DecodeProbe")
    )
    subscriber = session.subscriber(topic, schema="DecodeProbe", schemas=schemas)
    publisher.publish_frame(_wire_frame(schemas.schema_hash("DecodeProbe"), body, count, offset))
    frame = subscriber.receive(1000)
    assert frame is not None
    return frame


def test_typed_publisher_requires_schema_set():
    with pytest.raises(TypeError, match="requires schemas"):
        cerulion.connect().publisher("typed-error-no-set", schema="Probe")


def test_typed_loan_rejects_unknown_and_missing_lengths():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    publisher = cerulion.connect().publisher(
        "typed-error-lengths", schema="Probe", schemas=schemas
    )
    with publisher.loan() as message:
        message.id = 1
    with pytest.raises(TypeError, match="unknown variable"):
        publisher.loan(values=1, other=2)


def test_materialized_publish_rejects_missing_and_extra_fields():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    publisher = cerulion.connect().publisher(
        "typed-error-fields", schema="Probe", schemas=schemas
    )
    with pytest.raises(cerulion.EncodeError, match="missing"):
        publisher.publish({"id": 1})
    with pytest.raises(cerulion.EncodeError, match="extra"):
        publisher.publish({"id": 1, "values": [2], "name": "x", "extra": 3})


def test_schema_mismatch_is_typed(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        SCHEMA
        + """\
  Other:
    fields:
      uint32 value: {}
"""
    )
    topic = unique_topic("typed-schema-mismatch")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schemas=schemas, schema="Probe")
    pub.publish({"id": 1, "values": [2], "name": "x"})
    frame = sub.receive(1000)
    assert frame is not None
    with pytest.raises(cerulion.SchemaMismatch) as exc:
        frame.view(schemas, "Other")
    assert exc.value.kind == "SchemaMismatch"
    frame.release()


def test_typed_subscriber_rejects_foreign_hash(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-foreign-hash")
    raw = session.publisher(topic, schema_hash=0xDEADBEEF)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    raw.publish(struct.pack("<I", 7) + b"\0" * 8)
    frame = sub.receive(1000)
    assert frame is not None
    with pytest.raises(cerulion.SchemaMismatch) as exc:
        frame.view()
    assert exc.value.kind == "SchemaMismatch"
    frame.release()


def test_typed_encode_length_and_attribute_errors():
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    pub = cerulion.connect().publisher(
        unique_topic("typed-encode-errors"), schema="Probe", schemas=schemas
    )
    with pytest.raises(cerulion.EncodeError):
        pub.publish({"id": 1, "values": 3, "name": "too"})
    with pytest.raises(TypeError, match="unknown variable"):
        pub.loan(values=1, other=2)


def test_readonly_view_and_unknown_attribute_errors(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-attribute-errors")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    pub.publish({"id": 1, "values": [2], "name": "x"})
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    with pytest.raises(TypeError):
        message.id = 4
    with pytest.raises(AttributeError):
        _ = message.missing
    frame.release()


@pytest.mark.parametrize(
    ("body", "count", "offset", "kind"),
    [
            (b"\0" * 20, 2, 35, "OffsetTableMismatch"),
            (b"\0" * 20, 1, 36, "OffsetTableMismatch"),
            (
            b"\0" * 4 + struct.pack("<II", 4, 1) + b"\0" * 8,
            2,
                36,
            "OffsetBelowDataFloor",
        ),
        (
                b"\0" * 4
                + struct.pack("<II", 20, 200)
                + struct.pack("<II", 20, 0)
                + b"\0" * 100,
            2,
                36,
            "VariableFieldOutOfBounds",
        ),
        (
            b"\0" * 4
            + struct.pack("<II", 20, 4)
            + struct.pack("<II", 22, 2)
            + b"\0" * 4,
            2,
            36,
            "OverlappingEntries",
        ),
        (
            b"\0" * 4
            + struct.pack("<II", 21, 2)
            + struct.pack("<II", 32, 0)
            + b"\0" * 4,
            2,
            36,
            "MisalignedElements",
        ),
        (
                b"\0" * 4
                + struct.pack("<II", 20, 0)
                + struct.pack("<II", 20, 1)
                + b"\xff",
            2,
            36,
            "InvalidUtf8",
        ),
    ],
)
def test_handcrafted_decode_errors(session, body, count, offset, kind):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(DECODE_SCHEMA)
    frame = _decode_probe(session, schemas, body, count=count, offset=offset)
    with pytest.raises(cerulion.DecodeError) as exc:
        frame.view()
    assert type(exc.value) is cerulion.DecodeError
    assert exc.value.kind == kind
    frame.release()


# SHM copies exactly the advertised frame range, so FrameTooShort,
# TotalSizeExceedsBuffer, and TotalSizeBelowPrefix cannot be injected through
# the complete-frame publisher without weakening its safety validation.


def test_typed_publisher_binding_detects_nested_schema_mutation(session):
    # Outer nests Inner; added first while Inner is still unresolved, Outer
    # resolves it as an OPAQUE variable field (a resolution warning). Adding
    # Inner afterwards flips the field to fixed-nested, changing Outer's
    # layout and therefore its hash - a bound publisher must refuse.
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Outer:\n    fields:\n      Inner n: {}\n")
    topic = unique_topic("typed-binding-mutation")
    pub = session.publisher(topic, schema="Outer", schemas=schemas)
    bound = schemas.schema_hash("Outer")
    schemas.add_yaml("schemas:\n  Inner:\n    fields:\n      uint32 x: {}\n")
    assert schemas.schema_hash("Outer") != bound
    with pytest.raises(cerulion.SchemaMismatch, match="changed after") as exc:
        pub.publish({"n": b"\0" * 8})
    assert exc.value.kind == "SchemaMismatch"
    with pytest.raises(cerulion.SchemaMismatch, match="changed after"):
        pub.loan(n=8)
    with pytest.raises(cerulion.SchemaMismatch, match="changed after"):
        pub.publish_frame(_wire_frame(bound, b"\0" * 8, count=0, offset=0))
    assert pub.sequence == 0


def test_typed_publisher_binding_tolerates_unrelated_mutation(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-binding-unrelated")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    # Adding an UNRELATED schema bumps the generation but leaves the bound
    # hash alone - publish and loan must keep working.
    schemas.add_yaml("schemas:\n  Later:\n    fields:\n      uint16 v: {}\n")
    pub.publish({"id": 3, "values": [1], "name": "a"})
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view().id == 3
    frame.release()
    with pub.loan(values=1) as message:
        message.id = 4


def test_view_cache_is_keyed_on_schema_and_generation(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  A:\n    fields:\n      uint32 v: {}\n"
        "  B:\n    fields:\n      uint64 w: {}\n"
    )
    topic = unique_topic("typed-view-cache-key")
    pub = session.publisher(topic, schema="A", schemas=schemas)
    sub = session.subscriber(topic, schema="A", schemas=schemas)
    pub.publish({"v": 7})
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    assert message is frame.view()
    # An unrelated schema bumps the generation: the cache key must notice
    # and re-resolve (same hash, so the re-resolve still decodes as "A").
    schemas.add_yaml("schemas:\n  Later:\n    fields:\n      uint16 u: {}\n")
    newer = frame.view()
    assert newer is not message
    assert newer.v == 7
    assert frame.view() is newer
    with pytest.raises(cerulion.SchemaMismatch):
        frame.view(schemas, "B")
    frame.release()




def test_string_field_rejects_a_non_string_value(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-string-type")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    for bad in (1, 2.5, None):
        with pytest.raises(cerulion.EncodeError, match="expects str or bytes"):
            pub.publish({"id": 1, "values": [2], "name": bad})
    assert pub.sequence == 0
    pub.publish({"id": 1, "values": [2], "name": b"raw"})
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view(schemas, "Probe").name == "raw"
    frame.release()


SCALAR_SCHEMA = """\
schemas:
  Scalars:
    fields:
      uint8 small: {}
      bool flag: {}
      float32 ratio: {}
      int16[3] triple: {}
      uint8[] values: {}
      string_fixed[8] tag: {}
"""


def _scalar_pair(session, name):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCALAR_SCHEMA)
    topic = unique_topic(name)
    pub = session.publisher(topic, schema="Scalars", schemas=schemas)
    sub = session.subscriber(topic, schema="Scalars", schemas=schemas)
    return pub, sub


GOOD = {
    "small": 255,
    "flag": True,
    "ratio": 0.5,
    "triple": [-1, 0, 1],
    "values": [0, 255],
    "tag": "ok",
}


@pytest.mark.parametrize(
    "field, bad",
    [
        ("small", 256),
        ("small", -1),
        ("small", 1.5),
        ("small", "1"),
        ("small", True),
        ("flag", "false"),
        ("flag", 1),
        ("ratio", "0.5"),
        ("ratio", True),
        ("triple", [0, 0, 40000]),
        ("triple", [0.5, 0, 0]),
        ("triple", 7),
        ("triple", [7]),
        ("triple", [1, 2, 3, 4]),
        ("values", [0, 256]),
        ("values", [1.5]),
        ("tag", b"\xff"),
    ],
)
def test_typed_publish_refuses_values_numpy_would_coerce(session, field, bad):
    pub, sub = _scalar_pair(session, f"typed-coerce-{field}")
    with pytest.raises(cerulion.EncodeError):
        pub.publish({**GOOD, field: bad})
    assert pub.sequence == 0
    pub.publish(GOOD)
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    assert (message.small, message.flag, message.ratio) == (255, True, 0.5)
    assert list(message.triple) == [-1, 0, 1]
    assert bytes(message.values) == bytes([0, 255])
    assert message.tag == "ok"
    frame.release()


def test_typed_loan_refuses_non_integral_lengths(session):
    pub, _ = _scalar_pair(session, "typed-loan-length")
    for bad in (3.9, "3", True, None):
        with pytest.raises(TypeError, match="must be an int"):
            pub.loan(values=bad)
    with pytest.raises(ValueError, match="non-negative"):
        pub.loan(values=-1)
    with pub.loan(values=2) as message:
        with pytest.raises(cerulion.EncodeError):
            message.small = 300
        with pytest.raises(cerulion.EncodeError, match="invalid UTF-8"):
            message.tag = b"\xff"
        message.values = [7, 8]
    assert pub.sequence == 1


def test_typed_publish_accepts_empty_scalar_arrays(session):
    pub, sub = _scalar_pair(session, "typed-empty-array")
    pub.publish({**GOOD, "values": []})
    frame = sub.receive(1000)
    assert frame is not None
    assert bytes(frame.view().values) == b""
    frame.release()


def test_typed_publish_frame_rejects_a_non_byte_buffer_as_type_error(session):
    pub, _ = _scalar_pair(session, "typed-frame-buffer")
    with pytest.raises(TypeError, match="single-byte"):
        pub.publish_frame(array.array("I", [0] * 16))
    assert pub.sequence == 0


def test_typed_loans_reclaim_slots_released_after_an_escaped_view(session):
    pub, sub = _scalar_pair(session, "typed-loan-reap")
    for _ in range(64):
        with pytest.raises(cerulion.EncodeError, match="escaped the `with` block"):
            with pub.loan(values=2) as message:
                escaped = message.values
        del escaped
    with pub.loan(values=2) as message:
        for field, value in GOOD.items():
            if field != "values":
                setattr(message, field, value)
        message.values = [4, 5]
    frame = sub.receive(1000)
    assert frame is not None
    assert bytes(frame.view().values) == bytes([4, 5])
    frame.release()

import array
import struct

import numpy as np
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
        message.values[:] = [6]
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view().id == 4
    assert bytes(frame.view().values) == b"\x06"
    frame.release()


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
        ("ratio", 1e100),
        ("ratio", -1e100),
        ("ratio", np.float64(np.finfo(np.float32).max) * 2),
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


def test_typed_f32_keeps_its_full_range_and_refuses_what_narrows_to_infinity(session):
    pub, sub = _scalar_pair(session, "typed-f32-range")
    f32_max = float(np.finfo(np.float32).max)
    # The edges of the float32 range, an explicit infinity and nan are
    # all representable and must round-trip exactly, not be refused.
    for good in (f32_max, -f32_max, float("inf"), float("-inf")):
        pub.publish({**GOOD, "ratio": good})
        frame = sub.receive(1000)
        assert frame is not None
        assert frame.view().ratio == np.float32(good)
        frame.release()
    pub.publish({**GOOD, "ratio": float("nan")})
    frame = sub.receive(1000)
    assert frame is not None
    assert np.isnan(frame.view().ratio)
    frame.release()
    # A finite value beyond that range is refused on every write path
    # instead of landing as +-inf: publish, a loan field, and an array
    # element (the fixture's float32 LaserScan is the real-world case).
    with pub.loan(values=0) as message:
        with pytest.raises(cerulion.EncodeError, match="expects F32"):
            message.ratio = 1e100
        message.ratio = f32_max
    assert pub.sequence == 6
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Ranges:\n    fields:\n      float32[2] pair: {}\n      float32[] many: {}\n"
    )
    topic = unique_topic("typed-f32-array-range")
    arrays = session.publisher(topic, schema="Ranges", schemas=schemas)
    with pytest.raises(cerulion.EncodeError, match="expects F32"):
        arrays.publish({"pair": [0.0, 1e100], "many": []})
    with pytest.raises(cerulion.EncodeError, match="expects F32"):
        arrays.publish({"pair": [0.0, 1.0], "many": np.array([1e39])})
    with arrays.loan(many=1) as message:
        with pytest.raises(cerulion.EncodeError, match="expects F32"):
            message.many = [-1e39]
        with pytest.raises(cerulion.EncodeError, match="expects F32"):
            message.pair = [1e39, 0.0]
        message.many = [f32_max]
        message.pair = [-f32_max, 0.0]
    assert arrays.sequence == 1


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


def test_typed_loan_rejects_a_positional_payload_len(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    pub = session.publisher(
        unique_topic("typed-loan-positional"), schema="Probe", schemas=schemas
    )
    with pytest.raises(TypeError, match="by keyword"):
        pub.loan(64)
    assert pub.sequence == 0


def test_typed_loan_reaches_a_variable_field_named_payload_len(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Named:\n    fields:\n      uint32 id: {}\n      uint8[] payload_len: {}\n"
    )
    topic = unique_topic("typed-loan-payload-len-field")
    pub = session.publisher(topic, schema="Named", schemas=schemas)
    sub = session.subscriber(topic, schema="Named", schemas=schemas)
    with pub.loan(payload_len=2) as message:
        message.id = 3
        message.payload_len[:] = [4, 5]
    frame = sub.receive(1000)
    assert frame is not None
    view = frame.view()
    assert view.id == 3
    assert bytes(view.payload_len) == b"\x04\x05"
    frame.release()


def test_raw_loan_takes_payload_len_only(session):
    pub = session.publisher(unique_topic("raw-loan-keywords"), schema_hash=0x51)
    with pub.loan(payload_len=4) as loan:
        loan.payload[:] = b"abcd"
    assert pub.sequence == 1
    with pytest.raises(TypeError, match="payload_len only"):
        pub.loan(values=3)
    with pytest.raises(TypeError, match="payload_len is required"):
        pub.loan()


def test_publisher_rejects_schemas_without_schema(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    with pytest.raises(TypeError, match="schemas requires schema"):
        session.publisher(
            unique_topic("typed-schemas-only"),
            schema_hash=schemas.schema_hash("Probe"),
            schemas=schemas,
        )


def test_nested_message_assignment_is_a_type_error(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  V2:\n    fields:\n      float32 x: {}\n      float32 y: {}\n"
        "  Holds2:\n    fields:\n      V2 v: {}\n"
    )
    pub = session.publisher(unique_topic("typed-nested-assign"), schema="Holds2", schemas=schemas)
    with pytest.raises(TypeError, match="through their fields"):
        with pub.loan() as message:
            message.v = {"x": 1.0, "y": 2.0}
    assert pub.sequence == 0


def test_typed_publish_frame_validates_the_frame_against_its_schema(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-publish-frame-validate")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    bound = schemas.schema_hash("Probe")
    # Two variable fields, so an offset table announcing five entries is
    # malformed: refused at the publisher, not at a subscriber's view().
    body = struct.pack("<I", 1) + b"\0" * 16
    with pytest.raises(cerulion.EncodeError, match="not a valid Probe frame"):
        pub.publish_frame(_wire_frame(bound, body, count=5, offset=4))
    assert pub.sequence == 0
    good = pub._schemas  # the same set encodes a valid frame to forward
    from cerulion._typed import _encode_message

    pub.publish_frame(bytes(_encode_message(good, "Probe", {"id": 9, "values": [1], "name": "ok"}, 0)))
    frame = sub.receive(1000)
    assert frame is not None
    assert frame.view().id == 9
    assert frame.view().name == "ok"
    frame.release()


def test_typed_publish_frame_validates_nested_bodies(session):
    from cerulion._typed import _descriptors_from_body, _encode_message, _offset

    schemas = cerulion.SchemaSet()
    schemas.add_yaml(
        "schemas:\n  Inner:\n    fields:\n      string s: {}\n"
        "  Outer:\n    fields:\n      uint32 id: {}\n      Inner inner: {}\n"
    )
    topic = unique_topic("typed-publish-frame-nested")
    pub = session.publisher(topic, schema="Outer", schemas=schemas)
    sub = session.subscriber(topic, schema="Outer", schemas=schemas)
    good = bytes(_encode_message(schemas, "Outer", {"id": 7, "inner": {"s": "ok"}}, 0))
    body = memoryview(good)[cerulion.WIRE_HEADER_SIZE :]
    inner = _descriptors_from_body(schemas.layout("Outer"), body)["inner"]
    # The outer offset table is intact; only the nested body's own table
    # (Inner has no fixed fields, so it opens the body: u32 offset, u32
    # len of `s`) points past the frame. Refused here, not at a
    # subscriber's view().
    table = cerulion.WIRE_HEADER_SIZE + _offset(inner) + schemas.layout("Inner").fixed_size
    bad = bytearray(good)
    bad[table : table + 4] = struct.pack("<I", 0xFFFF_0000)
    with pytest.raises(cerulion.EncodeError, match="not a valid Outer frame"):
        pub.publish_frame(bytes(bad))
    assert pub.sequence == 0
    pub.publish_frame(good)
    frame = sub.receive(1000)
    assert frame is not None
    assert (frame.view().id, frame.view().inner.s) == (7, "ok")
    frame.release()


def test_typed_publish_frame_forwards_what_the_reader_accepts(session):
    # The forwarding check is the reader's decode, no stricter: a nested
    # array whose bytes are not canonically framed is NOT an error on
    # either side (the core walker surfaces it opaque so a producer-defined
    # convention survives; the subscriber reads the raw bytes), so it
    # forwards verbatim, while a canonically framed one decodes.
    schemas = cerulion.SchemaSet()
    schemas.add_yaml("schemas:\n  Tags:\n    fields:\n      uint32 id: {}\n      string[] tags: {}\n")
    topic = unique_topic("typed-publish-frame-opaque")
    pub = session.publisher(topic, schema="Tags", schemas=schemas)
    sub = session.subscriber(topic, schema="Tags", schemas=schemas)
    bound = schemas.schema_hash("Tags")
    opaque = b"\x05\x00\xff"  # three bytes: not a u32 count, not an element
    canonical = struct.pack("<I", 1) + struct.pack("<I", 1) + b"a"
    for blob, expected in ((opaque, opaque), (canonical, ["a"])):
        body = struct.pack("<I", 1) + struct.pack("<II", 12, len(blob)) + blob
        pub.publish_frame(_wire_frame(bound, body, count=1))
        frame = sub.receive(1000)
        assert frame is not None
        assert bytes(frame.payload) == body
        tags = frame.view().tags
        assert (bytes(tags) if isinstance(expected, bytes) else tags) == expected
        frame.release()
    assert pub.sequence == 2


def test_typed_publish_frame_bounds_validation_to_the_frame(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    pub = session.publisher(
        unique_topic("typed-publish-frame-bounds"),
        schema="Probe",
        schemas=schemas,
        max_payload_len=64,
    )
    bound = schemas.schema_hash("Probe")
    # A frame whose declared payload exceeds max_payload_len is refused
    # before any validation copy, however large the buffer behind it.
    with pytest.raises(cerulion.EncodeError, match="exceeds max_payload_len"):
        pub.publish_frame(_wire_frame(bound, b"\0" * 65, count=2, offset=4))
    with pytest.raises(cerulion.EncodeError, match="outside its buffer"):
        pub.publish_frame(_wire_frame(bound, b"\0" * 8, count=2, offset=4)[:-1])
    with pytest.raises(cerulion.EncodeError, match="shorter than the wire header"):
        pub.publish_frame(b"\0" * 8)
    # A foreign hash is reported as such even when the frame is also too big.
    with pytest.raises(cerulion.SchemaMismatch):
        pub.publish_frame(_wire_frame(0xDEADBEEF, b"\0" * 65, count=2, offset=4))
    assert pub.sequence == 0

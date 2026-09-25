import os
import struct
import subprocess
import sys

import pytest

import cerulion


VECTOR3_MSG = "float64 x\nfloat64 y\nfloat64 z\n"
HASH_A = 0x0BADC0DE0BADC0DE
HASH_B = 0x123456789ABCDEF0


def write_bag(fixture_bin, path):
    result = subprocess.run(
        [fixture_bin, "write-bag", "--path", os.fspath(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout == "WROTE 9\n"


def oracle_frame(schema_hash, sequence):
    payload = bytes((sequence * 7 + i) & 0xFF for i in range(8 + sequence % 5))
    total_size = cerulion.WIRE_HEADER_SIZE + len(payload)
    header = struct.pack(
        "<QIIIIQ",
        schema_hash,
        total_size,
        0,
        0,
        sequence,
        2_000_000_000 + sequence * 10_000_000,
    )
    return header + payload


def vector3_schemas():
    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg(VECTOR3_MSG, "geometry_msgs/Vector3")
    return schemas


def test_record_validates_wire_header():
    with pytest.raises(cerulion.BagError):
        cerulion.Record(b"\x00" * 31)

    invalid = struct.pack("<QIIIIQ", 1, 96, 0, 0, 0, 0) + bytes(8)
    with pytest.raises(cerulion.BagError, match="total_size=96"):
        cerulion.Record(invalid)

    payload = bytes(range(8))
    valid = struct.pack("<QIIIIQ", 1, 40, 0, 0, 0, 0) + payload
    record = cerulion.Record(valid)
    assert bytes(record.payload) == payload


def test_bag_topics_and_records(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    schemas = vector3_schemas()
    vector_hash = schemas.schema_hash("geometry_msgs/Vector3")

    bag = cerulion.open_bag(path)
    assert bag.topics() == [
        cerulion.TopicInfo("/py_bag/a", "py_bag/A", HASH_A, 5),
        cerulion.TopicInfo("/py_bag/b", "py_bag/B", HASH_B, 3),
        cerulion.TopicInfo("/py_bag/vec", "geometry_msgs/Vector3", vector_hash, 1),
    ]
    with cerulion.open_bag(os.fsencode(path)) as bytes_bag:
        assert bytes_bag.topics() == bag.topics()
    expected = [
        ("/py_bag/a", 0, HASH_A),
        ("/py_bag/b", 0, HASH_B),
        ("/py_bag/a", 1, HASH_A),
        ("/py_bag/a", 2, HASH_A),
        ("/py_bag/b", 1, HASH_B),
        ("/py_bag/a", 3, HASH_A),
        ("/py_bag/b", 2, HASH_B),
        ("/py_bag/a", 4, HASH_A),
    ]
    records = list(bag.messages())
    assert len(records) == 9
    assert [topic for topic, _ in records[:8]] == [topic for topic, _, _ in expected]
    for (topic, record), (_, sequence, schema_hash) in zip(records[:8], expected):
        raw = oracle_frame(schema_hash, sequence)
        assert record.schema_hash == schema_hash
        assert record.sequence == sequence
        assert record.timestamp_ns == 2_000_000_000 + sequence * 10_000_000
        assert record.total_size == len(raw)
        assert record.raw == raw
        assert bytes(record.payload) == raw[cerulion.WIRE_HEADER_SIZE :]
        assert record.payload.readonly
    topic, record = records[8]
    assert topic == "/py_bag/vec"
    message = record.view(schemas, "geometry_msgs/Vector3")
    assert (message.x, message.y, message.z) == (1.5, -2.0, 0.25)
    with pytest.raises(TypeError):
        record.view(schemas)
    with pytest.raises(TypeError):
        record.view(schema="geometry_msgs/Vector3")
    bag.close()
    assert isinstance(record.raw, bytes)
    with pytest.raises(cerulion.BagError):
        bag.topics()


def test_bag_filters_and_context_manager(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    with cerulion.open_bag(path) as bag:
        assert [record.sequence for _, record in bag.messages("/py_bag/b")] == [0, 1, 2]
        assert len(list(bag.messages(["/py_bag/a", "/py_bag/b"]))) == 8
        with pytest.raises(ValueError, match="unknown topic"):
            list(bag.messages("/py_bag/missing"))
    with pytest.raises(cerulion.BagError):
        bag.topics()


def test_bag_errors_and_determinism(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    second = tmp_path / "bag2.mcap"
    write_bag(fixture_bin, second)
    assert path.read_bytes() == second.read_bytes()
    bag = cerulion.open_bag(path)
    first = [(topic, record.raw) for topic, record in bag.messages()]
    second_pass = [(topic, record.raw) for topic, record in bag.messages()]
    assert first == second_pass

    with pytest.raises(cerulion.BagError):
        cerulion.open_bag(tmp_path / "missing.mcap")
    invalid = tmp_path / "invalid.mcap"
    invalid.write_bytes(b"not an mcap bag")
    with pytest.raises(cerulion.BagError):
        invalid_bag = cerulion.open_bag(invalid)
        invalid_bag.topics()
    truncated = tmp_path / "truncated.mcap"
    truncated.write_bytes(path.read_bytes()[:-64])
    with pytest.raises(cerulion.BagError):
        list(cerulion.open_bag(truncated).messages())


@pytest.mark.skipif(sys.platform == "darwin", reason="APFS rejects non-UTF-8 names")
def test_open_bag_accepts_a_non_utf8_bytes_path(fixture_bin, tmp_path):
    written = tmp_path / "bag.mcap"
    write_bag(fixture_bin, written)
    path = os.fsencode(tmp_path) + b"/bag-\xff.mcap"
    os.rename(written, path)
    with cerulion.open_bag(path) as bag:
        assert [topic.count for topic in bag.topics()] == [5, 3, 1]


def test_open_bag_rejects_a_corrupted_chunk_body(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    data = bytearray(path.read_bytes())
    frame = oracle_frame(HASH_A, 0)
    offset = data.find(frame)
    assert offset > 0
    data[offset + len(frame) - 1] ^= 0xFF
    corrupted = tmp_path / "corrupted.mcap"
    corrupted.write_bytes(bytes(data))
    with pytest.raises(cerulion.BagError, match="(?i)crc"):
        cerulion.open_bag(corrupted)


def test_close_invalidates_pending_iterators_but_not_yielded_records(fixture_bin, tmp_path):
    path = tmp_path / "bag.mcap"
    write_bag(fixture_bin, path)
    bag = cerulion.open_bag(path)
    records = bag.messages("/py_bag/a")
    native = bag._native.messages(None)
    topic, first = next(records)
    bag.close()
    with pytest.raises(cerulion.BagError, match="bag is closed"):
        next(native)
    with pytest.raises(StopIteration):
        next(native)
    with pytest.raises(cerulion.BagError, match="bag is closed"):
        next(records)
    with pytest.raises(StopIteration):
        next(records)
    assert (topic, first.raw) == ("/py_bag/a", oracle_frame(HASH_A, 0))

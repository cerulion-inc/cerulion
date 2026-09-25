"""Rust<->Python interop round trips through the fixture binary."""

import numpy as np
import pytest

import cerulion

from conftest import (
    expected_header_bytes,
    finish_proc,
    fnv1a64,
    pattern,
    spawn_fixture,
    unique_topic,
    wait_ready,
)

SCHEMA_HASH = 0x1122334455667788
TIMESTAMP_NS = 4242
SIZES = [64, 65536, 4 << 20]


@pytest.mark.parametrize("size", SIZES, ids=lambda s: f"{s}B")
def test_rust_publish_to_python(session, fixture_bin, size):
    topic = unique_topic(f"r2p{size}")
    sub = session.subscriber(topic, depth=4)
    proc = spawn_fixture(
        fixture_bin,
        [
            "publish",
            "--topic",
            topic,
            "--count",
            "3",
            "--size",
            str(size),
            "--schema-hash",
            str(SCHEMA_HASH),
            "--timestamp-ns",
            str(TIMESTAMP_NS),
        ],
    )
    try:
        for i in range(3):
            frame = sub.receive(5000)
            assert frame is not None, f"frame {i} not received"
            assert frame.schema_hash == SCHEMA_HASH
            assert frame.timestamp_ns == TIMESTAMP_NS
            assert frame.sequence == i
            assert frame.total_size == 32 + size
            assert frame.recv_ns > 0
            assert bytes(frame.raw[:32]) == expected_header_bytes(
                SCHEMA_HASH, 32 + size, i, TIMESTAMP_NS
            )
            expected_body = pattern(size, i)
            assert np.array_equal(frame.as_numpy(), np.frombuffer(expected_body, np.uint8))
            assert frame.to_bytes() == expected_body
            frame.release()
    finally:
        finish_proc(proc)


@pytest.mark.parametrize("size", [64, 4 << 20], ids=["64B", "4MiB"])
def test_python_publish_to_rust(session, fixture_bin, size):
    topic = unique_topic(f"p2r{size}")
    proc = spawn_fixture(
        fixture_bin,
        ["subscribe", "--topic", topic, "--count", "2", "--timeout-ms", "30000"],
    )
    wait_ready(proc)
    try:
        pub = session.publisher(topic, SCHEMA_HASH, max_payload_len=size)
        pub.publish(pattern(size, 0), timestamp_ns=99)
        with pub.loan(size) as loan:
            loan.payload[:] = pattern(size, 1)
            loan.timestamp_ns = 100
        lines = [proc.stdout.readline().strip() for _ in range(2)]
        expected = [
            f"frame seq=0 schema_hash={SCHEMA_HASH} timestamp_ns=99 total_size={32 + size} "
            f"body_len={size} fnv1a64={fnv1a64(pattern(size, 0)):016x}",
            f"frame seq=1 schema_hash={SCHEMA_HASH} timestamp_ns=100 total_size={32 + size} "
            f"body_len={size} fnv1a64={fnv1a64(pattern(size, 1)):016x}",
        ]
        assert lines == expected
    finally:
        finish_proc(proc)


def test_publish_buffer_types(session):
    topic = unique_topic("buftypes")
    sub = session.subscriber(topic, depth=4)
    pub = session.publisher(topic, 1, max_payload_len=64)
    for payload in (
        bytes(range(16)),
        bytearray(range(16)),
        memoryview(bytes(range(16))),
        np.arange(16, dtype=np.uint8),
    ):
        pub.publish(payload, timestamp_ns=1)
    for _ in range(4):
        frame = sub.receive(2000)
        assert frame is not None
        assert frame.to_bytes() == bytes(range(16))
        frame.release()

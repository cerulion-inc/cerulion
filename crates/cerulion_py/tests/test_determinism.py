"""Determinism: same payload twice differs only in the 4 sequence bytes."""

import cerulion

from conftest import expected_header_bytes, pattern, unique_topic


def test_two_publishes_differ_only_in_sequence(session):
    topic = unique_topic("det")
    sub = session.subscriber(topic, depth=4)
    pub = session.publisher(topic, 0xBEEF, max_payload_len=64)
    body = pattern(64, 0)
    pub.publish(body, timestamp_ns=7)
    pub.publish(body, timestamp_ns=7)
    f0 = sub.receive(2000)
    f1 = sub.receive(2000)
    assert f0 is not None and f1 is not None
    r0 = bytes(f0.raw)
    r1 = bytes(f1.raw)
    assert r0[20:24] != r1[20:24]
    assert r0[:20] == r1[:20] and r0[24:] == r1[24:]
    assert r0 == expected_header_bytes(0xBEEF, 96, 0, 7) + body
    assert r1 == expected_header_bytes(0xBEEF, 96, 1, 7) + body
    assert pub.sequence == 2
    f0.release()
    f1.release()

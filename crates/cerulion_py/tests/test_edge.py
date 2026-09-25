"""Edge cases: empty payloads, zero timeouts, iterator auto-release, context managers."""

import time

import pytest

import cerulion

from conftest import unique_topic


def test_sub_ms_poll_does_not_spin(session):
    """OpenBLAS workers spin independently, so measure only this thread."""
    sub = session.subscriber(unique_topic("sub-ms"), depth=1)
    cpu_start = time.thread_time()
    wall_start = time.perf_counter()
    for _ in range(50):
        assert sub.receive(timeout_ms=1) is None
    cpu_elapsed = time.thread_time() - cpu_start
    wall_elapsed = time.perf_counter() - wall_start
    assert wall_elapsed >= 0.05
    assert cpu_elapsed < 0.025


def test_publish_empty_payload(session):
    topic = unique_topic("empty")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"", timestamp_ns=5)
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.total_size == 32
    assert len(frame.payload) == 0
    assert frame.to_bytes() == b""
    frame.release()


def test_loan_zero_length(session):
    topic = unique_topic("loan0")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    loan = pub.loan(0)
    loan.commit()
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.total_size == 32
    assert len(frame.payload) == 0
    frame.release()


def test_receive_zero_timeout_empty(session):
    sub = session.subscriber(unique_topic("t0"), depth=1)
    assert sub.receive(timeout_ms=0) is None
    assert sub.try_receive() is None


def test_receive_u64_max_timeout_delivers(session, tmp_path):
    """An unrepresentable deadline (2**64-1 ms) is treated as unbounded, not an error.

    The frame is published from a SECOND PROCESS after receive() has
    entered its wait path (Publisher is unsendable - a thread cannot
    do it) - a buggy 'treat as expired' implementation would return
    None before the publish lands."""
    import subprocess
    import sys

    topic = unique_topic("u64max")
    sub = session.subscriber(topic, depth=2)
    ready = tmp_path / "receiving"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-c",
            f"import cerulion, os, time\n"
            f"s = cerulion.connect()\n"
            f"p = s.publisher({topic!r}, 1, max_payload_len=64)\n"
            f"while not os.path.exists({str(ready)!r}):\n"
            f"    time.sleep(0.01)\n"
            f"time.sleep(1.0)\n"
            f"p.publish(b'x')\n"
            f"time.sleep(2.0)\n",  # linger so SHM outlives the publisher
        ]
    )
    try:
        ready.touch()
        frame = sub.receive(timeout_ms=2**64 - 1)
    finally:
        proc.kill()
        proc.wait()
    assert frame is not None
    frame.release()


def test_receive_timeout_above_u64_raises_overflow(session):
    """pyo3's u64 extraction: a Python int > u64::MAX is an OverflowError."""
    sub = session.subscriber(unique_topic("overu64"), depth=1)
    with pytest.raises(OverflowError):
        sub.receive(timeout_ms=2**64)


def test_iterator_releases_previous_frame(session):
    topic = unique_topic("iter")
    sub = session.subscriber(topic, depth=4)
    pub = session.publisher(topic, 1, max_payload_len=64)
    for _ in range(3):
        pub.publish(b"x")
    it = iter(sub)
    f1 = next(it)
    f2 = next(it)
    assert f1.is_released is True
    assert f2.is_released is False
    assert f2.sequence == 1
    f2.release()


def test_connect_is_singleton_and_cm(session):
    assert cerulion.connect() is cerulion.connect()
    with cerulion.connect() as s:
        assert s is cerulion.connect()


def test_frame_context_manager_releases(session):
    topic = unique_topic("fcm")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"hi")
    with sub.receive(2000) as frame:
        assert frame is not None
        assert frame.to_bytes() == b"hi"
        assert frame.is_released is False
    assert frame.is_released is True
    frame.release()  # idempotent


def test_loan_discarded_on_exception(session):
    topic = unique_topic("loandisc")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    with pytest.raises(RuntimeError):
        with pub.loan(4) as loan:
            loan.payload[:] = b"data"
            raise RuntimeError("boom")
    assert loan.is_open is False
    assert sub.receive(timeout_ms=200) is None


def test_loan_double_commit_raises(session):
    pub = session.publisher(unique_topic("ldbl"), 1, max_payload_len=64)
    loan = pub.loan(4)
    loan.commit()
    assert loan.is_open is False
    with pytest.raises(ValueError, match="committed or discarded"):
        loan.commit()


def test_publisher_sequence_property(session):
    pub = session.publisher(unique_topic("pseq"), 1, max_payload_len=64)
    assert pub.sequence == 0
    pub.publish(b"a")
    assert pub.sequence == 1


def test_subscriber_topic_property(session):
    topic = unique_topic("stopic")
    sub = session.subscriber(topic, depth=1)
    assert sub.topic == topic
    assert sub.depth == 1

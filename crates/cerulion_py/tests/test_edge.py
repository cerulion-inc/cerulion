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
    # Bounded on BOTH sides: the floor proves the 1 ms waits are real waits,
    # the ceiling proves they are 1 ms waits. Fifty polls that each slept a
    # coarse 100 ms would take 5 s and pass the floor alone; 2 s leaves room
    # for a loaded shared runner's scheduler jitter and nothing else.
    assert wall_elapsed >= 0.05
    assert wall_elapsed < 2.0
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

    The unbounded receive() runs in a SECOND PROCESS and this process
    publishes into it once the receiver has entered its wait path - a
    buggy 'treat as expired' implementation returns None before the
    publish lands and the child exits 3. The roles are this way round so
    the wait with no deadline is the CHILD's: every wait here is bounded
    (readiness 15 s, exit 30 s), so a receiver that dies before the
    publish, or never wakes after it, fails the test with its stderr
    instead of hanging the pytest process until the job timeout."""
    import subprocess
    import sys

    topic = unique_topic("u64max")
    pub = session.publisher(topic, 1, max_payload_len=64)
    ready = tmp_path / "receiving"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-c",
            f"import cerulion, pathlib, sys\n"
            f"s = cerulion.connect()\n"
            f"sub = s.subscriber({topic!r}, depth=2)\n"
            f"pathlib.Path({str(ready)!r}).touch()\n"
            f"frame = sub.receive(timeout_ms=2**64 - 1)\n"
            f"if frame is None:\n"
            f"    print('none')\n"
            f"    sys.exit(3)\n"
            f"print(frame.to_bytes().hex())\n"
            f"frame.release()\n",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 15.0
        while not ready.exists():
            if proc.poll() is not None:
                out, err = proc.communicate()
                pytest.fail(f"receiver exited {proc.returncode} before receive(): {err}")
            if time.monotonic() > deadline:
                pytest.fail("receiver did not signal readiness within 15 s")
            time.sleep(0.01)
        # Let the receiver reach its wait path before the publish lands.
        time.sleep(1.0)
        pub.publish(b"x")
        try:
            out, err = proc.communicate(timeout=30.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, err = proc.communicate()
            pytest.fail(f"receive(2**64 - 1) did not return within 30 s of the publish: {err}")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    assert proc.returncode == 0, f"receiver exited {proc.returncode}: stdout={out!r} stderr={err!r}"
    assert out.strip() == b"x".hex()


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

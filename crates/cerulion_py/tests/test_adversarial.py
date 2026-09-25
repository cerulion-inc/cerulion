"""Adversarial cases: use-after-release, borrow exhaustion, oversize, live-view commits."""

import numpy as np
import pytest

import cerulion

from conftest import pattern, unique_topic


def test_use_after_release(session):
    topic = unique_topic("uar")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"data")
    frame = sub.receive(2000)
    assert frame is not None
    frame.release()
    assert frame.is_released is True
    for op in (
        lambda: frame.schema_hash,
        lambda: frame.sequence,
        lambda: frame.timestamp_ns,
        lambda: frame.total_size,
        lambda: frame.recv_ns,
        lambda: frame.payload,
        lambda: frame.raw,
        lambda: frame.as_numpy(),
        lambda: frame.to_bytes(),
    ):
        with pytest.raises(cerulion.ReleasedFrame):
            op()


def test_borrow_limit_exceeded(session):
    topic = unique_topic("borrow")
    sub = session.subscriber(topic, depth=8)
    pub = session.publisher(topic, 1, max_payload_len=64)
    held = []
    with pytest.raises(cerulion.BorrowLimitExceeded) as exc:
        for _ in range(32):
            pub.publish(b"x")
            held.append(sub.receive(2000))
    msg = str(exc.value)
    assert "release" in msg
    assert len(held) == sub.max_borrowed_samples
    held[0].release()
    pub.publish(b"y")
    frame = sub.receive(2000)
    assert frame is not None
    frame.release()
    for f in held[1:]:
        f.release()


def test_oversize_publish_and_loan(session):
    pub = session.publisher(unique_topic("over"), 1, max_payload_len=64)
    with pytest.raises(cerulion.EncodeError):
        pub.publish(b"x" * 65)
    with pytest.raises(cerulion.EncodeError):
        pub.loan(65)
    assert pub.sequence == 0  # failed loan did not consume a sequence number


def test_loan_oversize_slice_write(session):
    pub = session.publisher(unique_topic("lover"), 1, max_payload_len=64)
    with pub.loan(4) as loan:
        with pytest.raises(ValueError):
            loan.payload[:] = b"too long"


def test_loan_commit_with_live_view(session):
    pub = session.publisher(unique_topic("live"), 1, max_payload_len=64)
    loan = pub.loan(4)
    view = loan.payload
    with pytest.raises(cerulion.EncodeError, match="loan.payload views"):
        loan.commit()
    del view
    loan.commit()
    assert loan.is_open is False


def test_type_errors(session):
    with pytest.raises(TypeError):
        session.publisher(123, 1)
    with pytest.raises(TypeError):
        session.publisher("t", "x")
    with pytest.raises(TypeError):
        session.subscriber("t", depth="1")
    pub = session.publisher(unique_topic("types"), 1, max_payload_len=64)
    with pytest.raises(TypeError):
        pub.publish("a string")
    sub = session.subscriber(unique_topic("types2"), depth=1)
    with pytest.raises(TypeError):
        sub.receive(timeout_ms="1")


def test_wrong_dtype_ndarray_raises_typeerror(session):
    pub = session.publisher(unique_topic("dtype"), 1, max_payload_len=64)
    with pytest.raises(TypeError, match="view"):
        pub.publish(np.zeros(8, dtype=np.float32))


def test_memoryview_released_on_foreign_thread(session):
    """Frame is `unsendable`: a memoryview released on another thread must not
    panic the pyclass borrow - bookkeeping is skipped (RuntimeWarning) and the
    slot is returned when the Frame itself drops."""
    import threading
    import warnings

    topic = unique_topic("xthread")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"data")
    frame = sub.receive(2000)
    assert frame is not None
    mv = frame.raw  # memoryview over the native Frame's buffer export

    def release_offthread():
        mv.release()

    worker = threading.Thread(target=release_offthread)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        worker.start()
        worker.join()
    assert any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]

    # The frame is still live and receivable-after on the owning thread.
    frame.release()
    # Reclamation proof: the skipped decrement left `exports` elevated,
    # so the slot is only returned at Frame drop. Deleting the frame on
    # the owning thread must free it - hold `max_borrowed_samples`
    # unreleased frames at once, which a leaked slot would prevent.
    del frame
    held = []
    for _ in range(sub.max_borrowed_samples):
        pub.publish(b"again")
        f = sub.receive(2000)
        assert f is not None
        held.append(f)
    for f in held:
        f.release()


def test_loan_memoryview_released_on_foreign_thread(session):
    """Releasing `loan.payload`'s memoryview on a foreign thread must not
    panic the unsendable Loan borrow (same pyo3 class as Frame): the
    decrement is skipped with a RuntimeWarning, so commit() keeps
    refusing while exports read as outstanding, and dropping the Loan
    still frees the slot."""
    import threading
    import warnings

    pub = session.publisher(unique_topic("loan-frn"), 1, max_payload_len=64)
    loan = pub.loan(8)
    mv = loan.payload
    errors = []

    def worker():
        try:
            mv.release()
        except Exception as e:  # pragma: no cover
            errors.append(e)

    t = threading.Thread(target=worker)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        t.start()
        t.join(10)
    assert not t.is_alive()
    assert errors == []
    assert any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]
    # The skipped decrement leaves exports > 0: commit still refuses.
    with pytest.raises(cerulion.EncodeError, match="loan.payload views"):
        loan.commit()


def test_owner_thread_release_after_prior_frame_dropped(session):
    import warnings

    topic = unique_topic("owner-reuse")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)

    pub.publish(b"a")
    frame_a = sub.receive(2000)
    assert frame_a is not None
    view_a = frame_a.raw
    view_a.release()
    del view_a
    del frame_a

    pub.publish(b"b")
    frame_b = sub.receive(2000)
    assert frame_b is not None
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        view_b = frame_b.raw
        del view_b
    assert not any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]
    frame_b.release()


def test_dropped_subscriber_then_publish(session):
    topic = unique_topic("drop")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(pattern(64, 0), timestamp_ns=1)
    frame = sub.receive(2000)
    assert frame is not None
    frame.release()
    del sub
    pub.publish(pattern(64, 1), timestamp_ns=2)
    assert pub.sequence == 2

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
    topic = unique_topic("live")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    loan = pub.loan(4)
    view = loan.payload
    view[:] = b"live"
    with pytest.raises(cerulion.EncodeError, match="loan.payload views"):
        loan.commit()
    assert sub.receive(0) is None  # the refused commit sent nothing
    del view
    loan.commit()
    assert loan.is_open is False
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.to_bytes() == b"live"
    frame.release()


def test_oversize_noncontiguous_publish_raises_typeerror(session):
    pub = session.publisher(unique_topic("over-nc"), 1, max_payload_len=64)
    strided = np.zeros(256, dtype=np.uint8)[::2]  # 128 bytes, not contiguous
    with pytest.raises(TypeError, match="contiguous"):
        pub.publish(strided)


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


def _release_on_thread(mv):
    import threading

    errors = []

    def worker():
        try:
            mv.release()
        except Exception as e:  # pragma: no cover
            errors.append(e)

    t = threading.Thread(target=worker)
    t.start()
    t.join(10)
    assert not t.is_alive()
    assert errors == []


def _hold_all_borrowed(sub, pub):
    held = []
    for _ in range(sub.max_borrowed_samples):
        pub.publish(b"again")
        f = sub.receive(2000)
        assert f is not None
        held.append(f)
    for f in held:
        f.release()


def test_memoryview_released_on_foreign_thread(session):
    """Frame is `unsendable`: a memoryview released on another thread must not
    panic the pyclass borrow, and its export is still uncounted, so a later
    owner-thread `release()` returns the slot at once."""
    import warnings

    topic = unique_topic("xthread")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"data")
    frame = sub.receive(2000)
    assert frame is not None
    mv = frame.raw  # memoryview over the native Frame's buffer export

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _release_on_thread(mv)
    assert not any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]

    # The frame stays referenced, so only release() can free its slot:
    # holding `max_borrowed_samples` frames at once proves it did.
    frame.release()
    _hold_all_borrowed(sub, pub)
    assert frame.is_released


def test_last_view_of_released_frame_freed_on_foreign_thread(session):
    """A released frame whose last view dies on a foreign thread warns (the
    slot cannot drop off the owner) and frees the slot at frame drop."""
    import warnings

    topic = unique_topic("xthread-late")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"data")
    frame = sub.receive(2000)
    assert frame is not None
    mv = frame.raw
    frame.release()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _release_on_thread(mv)
    assert any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]
    del frame
    _hold_all_borrowed(sub, pub)


def test_released_frame_slot_returns_at_next_receive_while_referenced(session):
    """The last view of a released frame closing on a foreign thread returns
    the slot at the subscriber's next receive, even with the frame object
    still referenced."""
    import warnings

    topic = unique_topic("xthread-park")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"data")
    frame = sub.receive(2000)
    assert frame is not None
    mv = frame.raw
    frame.release()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _release_on_thread(mv)
    assert any("next receive" in str(w.message) for w in caught), [
        str(w.message) for w in caught
    ]
    _hold_all_borrowed(sub, pub)
    assert frame.is_released


def test_iterator_does_not_retain_last_frame(session):
    """Leaving a `for frame in sub` loop and dropping the frame frees its
    slot: the iterator keeps no strong reference."""
    topic = unique_topic("iter-drop")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"iter")
    for frame in sub:
        assert frame.to_bytes() == b"iter"
        break
    del frame
    _hold_all_borrowed(sub, pub)


def test_loan_memoryview_released_on_foreign_thread(session):
    """Releasing `loan.payload`'s memoryview on a foreign thread must not
    panic the unsendable Loan borrow, and uncounts the export: commit()
    then succeeds and the subscriber receives the written bytes."""
    import warnings

    topic = unique_topic("loan-frn")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    loan = pub.loan(8)
    mv = loan.payload
    mv[:8] = b"xthread!"
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _release_on_thread(mv)
    assert not any(w.category is RuntimeWarning for w in caught), [
        str(w.message) for w in caught
    ]
    loan.commit()
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.to_bytes() == b"xthread!"
    frame.release()


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


def test_independent_iterators_do_not_release_each_others_frames(session):
    """Each `iter(sub)` owns its previous-frame slot: advancing one iterator
    never releases a frame another iterator handed out."""
    topic = unique_topic("iter-indep")
    sub = session.subscriber(topic, depth=4)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"one")
    pub.publish(b"two")
    it1, it2 = iter(sub), iter(sub)
    assert it1 is not it2
    f1 = next(it1)
    f2 = next(it2)
    assert f1.is_released is False
    assert f1.to_bytes() == b"one" and f2.to_bytes() == b"two"
    f1.release()
    f2.release()


def _max_open_loans(pub):
    loans = []
    try:
        while len(loans) < 64:
            loans.append(pub.loan(8))
    except cerulion.TransportError:
        pass
    for loan in loans:
        loan.discard()
    return len(loans)


def test_discarded_loan_slot_returns_after_foreign_thread_view_release(session):
    """A loan discarded with a live view parks its slot with the publisher;
    when that view closes on a foreign thread the slot returns at the next
    loan(), so the publisher's loan budget is not consumed permanently."""
    import warnings

    topic = unique_topic("loan-park")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    budget = _max_open_loans(pub)
    assert budget >= 1
    loan = pub.loan(8)
    mv = loan.payload
    loan.discard()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _release_on_thread(mv)
    assert any("next publish() or loan()" in str(w.message) for w in caught), [
        str(w.message) for w in caught
    ]
    assert loan.is_open is False
    assert _max_open_loans(pub) == budget
    pub.publish(b"after")
    frame = sub.receive(2000)
    assert frame is not None
    assert frame.to_bytes() == b"after"
    frame.release()

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
"""


def test_typed_loan_is_writable_and_detaches_on_commit(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-loan")
    publisher = session.publisher(topic, schema="Probe", schemas=schemas)
    subscriber = session.subscriber(topic, schema="Probe", schemas=schemas)

    with publisher.loan(values=3) as message:
        message.id = 9
        message.values[:] = [13, 21, 34]

    frame = subscriber.receive(1000)
    assert frame is not None
    assert frame.view().copy()["id"] == 9
    frame.release()


def test_frame_message_rejects_writes(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-readonly")
    publisher = session.publisher(topic, schema="Probe", schemas=schemas)
    subscriber = session.subscriber(topic, schema="Probe", schemas=schemas)
    publisher.publish({"id": 4, "values": [1, 2]})
    frame = subscriber.receive(1000)
    message = frame.view()
    with pytest.raises(TypeError, match="read-only"):
        message.id = 5
    frame.release()


def test_frame_release_rejects_typed_access(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-release")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    pub.publish({"id": 4, "values": [1, 2]})
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    frame.release()
    with pytest.raises(cerulion.ReleasedFrame):
        _ = message.id


def test_array_keeps_frame_alive_until_array_deleted(session):
    import gc
    import weakref

    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-array-lifetime")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    pub.publish({"id": 4, "values": [1, 2]})
    frame = sub.receive(1000)
    assert frame is not None
    message = frame.view()
    array = message.values
    ref = weakref.ref(frame)
    del message
    del frame
    gc.collect()
    assert ref() is not None
    assert list(array) == [1, 2]
    del array
    gc.collect()
    assert ref() is None


def test_borrow_limit_is_bounded(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-borrow-limit")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    held = []
    for i in range(sub.max_borrowed_samples):
        pub.publish({"id": i, "values": [i]})
        frame = sub.receive(1000)
        assert frame is not None
        held.append(frame)
    pub.publish({"id": sub.max_borrowed_samples, "values": [0]})
    with pytest.raises(cerulion.BorrowLimitExceeded):
        sub.receive(100)
    for frame in held:
        frame.release()


def test_loan_message_is_detached_after_context(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    pub = session.publisher(unique_topic("typed-loan-detach"), schema="Probe", schemas=schemas)
    with pub.loan(values=1) as message:
        message.id = 9
        retained = message
    with pytest.raises(cerulion.ReleasedFrame, match="loan is closed"):
        _ = retained.id


def test_loan_field_view_escaping_block_discards_loan(session):
    schemas = cerulion.SchemaSet()
    schemas.add_yaml(SCHEMA)
    topic = unique_topic("typed-loan-escape")
    pub = session.publisher(topic, schema="Probe", schemas=schemas)
    sub = session.subscriber(topic, schema="Probe", schemas=schemas)
    with pytest.raises(cerulion.EncodeError, match="escaped the `with` block"):
        with pub.loan(values=3) as message:
            message.id = 1
            escaped = message.values  # live export past block exit
    # The loan was discarded: nothing may arrive.
    assert sub.receive(300) is None
    del escaped
    with pub.loan(values=2) as message:
        message.id = 9
        message.values[:] = [7, 8]
    frame = sub.receive(1000)
    assert frame is not None
    copied = frame.view().copy()
    assert copied["id"] == 9
    np.testing.assert_array_equal(copied["values"], [7, 8])
    frame.release()

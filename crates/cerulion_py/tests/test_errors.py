"""Error-arm coverage for the `TransportError` -> Python mapping (src/errors.rs).

Reachable from the PR1 client surface and tested here or in test_adversarial.py:
SubscriberCreation (depth 0 / >16, topic too long), Receive/ExceedsMaxBorrows -> BorrowLimitExceeded,
NotInitialized (native class before connect()), EncodeError (facade-level length checks).

Unreachable from PR1 and why (kept in the match so a future surface maps them, not `_`):
- SchemaMismatch / SchemaHashMismatch: raised by typed proxies; raw frames carry no type check.
- LoanCapacity -> TransportError: only via the core's fault-injection hook, which the module does
  not expose. EncodeError is facade length checks only (publish/loan oversize, live-view commit).
- Loan / Publish: iceoryx2 internal loan/send failure (pool exhausted by a peer, segment gone) -
  not producible deterministically in-process.
- NodeCreation / DuplicateNode / InvalidTransportConfig / UnrepresentableNodeName: connect() uses a
  fixed default config; the singleton is idempotent, so no second node is ever attempted.
- PublisherCreation: only on an iceoryx2 service conflict with a foreign process.
- TopicNotFound / NodeNotFound / SchedulerError / GraphError / ExternalNodesInertAtLaunch /
  LiveReactorBuild / GraphParseError / NodeError / NodeTickPanicked / NodeInfoParse /
  SessionCreation: graph-runtime and network-daemon paths this module never calls.
- Deserialization / BufferTooSmall / ProxyBufferTooSmall / MissingVariableField /
  MaxSliceLenRequired / PushAfterNonTail / NestedWriteConflict / NestedChildIncomplete /
  PayloadTooLarge / AllocationFailed / Internal: typed-codec paths; PR1 is raw bytes only.
"""

import inspect
import subprocess
import sys

import pytest

import cerulion

from conftest import unique_topic


@pytest.mark.skipif(sys.version_info < (3, 12), reason="__buffer__ wrapper is 3.12+")
def test_writable_request_on_frame_raises_buffer_error(session):
    """A writable buffer request hits CPython's own `BufferError` inside
    `PyBuffer_FillInfo`; the exporter must propagate it (not replace it with
    `TransportError`) and must not count the failed export."""
    topic = unique_topic("readonly")
    sub = session.subscriber(topic, depth=2)
    pub = session.publisher(topic, 1, max_payload_len=64)
    pub.publish(b"x")
    frame = sub.receive(2000)
    assert frame is not None

    with pytest.raises(BufferError, match="not writable"):
        frame._native.__buffer__(inspect.BufferFlags.WRITABLE)

    assert bytes(frame.raw)[32:] == b"x"
    frame.release()
    assert frame.is_released is True
    # Reclamation proof: a counted-but-never-released export would defer
    # the slot's return past release(), and the frame object is still
    # alive here - so holding `max_borrowed_samples` unreleased frames
    # succeeds only if the failed export left nothing borrowed.
    held = []
    for _ in range(sub.max_borrowed_samples):
        pub.publish(b"again")
        f = sub.receive(2000)
        assert f is not None
        held.append(f)
    for f in held:
        f.release()


def test_subscriber_depth_zero(session):
    with pytest.raises(cerulion.TransportError, match="buffer_size"):
        session.subscriber(unique_topic("d0"), depth=0)


def test_subscriber_depth_over_ceiling(session):
    with pytest.raises(cerulion.TransportError, match="buffer_size"):
        session.subscriber(unique_topic("d17"), depth=17)


def test_topic_too_long(session):
    with pytest.raises(cerulion.TransportError):
        session.subscriber("/" + "t" * 300, depth=1)


def test_native_publisher_without_connect():
    """NATIVE class (not facade) without connect() -> TransportError, not a hang or crash."""
    result = subprocess.run(
        [sys.executable, "-c", "import cerulion._native as n; n.Publisher('/t', 1, 64)"],
        capture_output=True,
        text=True,
        cwd="/tmp",
        timeout=30,
    )
    assert result.returncode != 0
    assert "TransportError" in result.stderr


def test_native_connect_is_idempotent():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import cerulion._native as n\n"
            "n.connect('/py/idempotent')\n"
            "n.connect('/py/idempotent')\n"
            "print('ok')",
        ],
        capture_output=True,
        text=True,
        cwd="/tmp",
        timeout=30,
    )
    assert result.returncode == 0
    assert "ok" in result.stdout

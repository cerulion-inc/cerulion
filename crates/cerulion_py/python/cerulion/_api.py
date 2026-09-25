"""Pure-Python facade over ``cerulion._native``.

One process-wide transport (``connect()`` is idempotent), facade classes
that delegate to the native objects and add iteration / context-manager
/ NumPy conveniences.
"""

import threading

import numpy as np

from cerulion import _native
from cerulion._native import (
    WIRE_HEADER_SIZE,
    BorrowLimitExceeded,
    CerulionError,
    EncodeError,
    ReleasedFrame,
    SchemaMismatch,
    TransportError,
    real_ns,
)

_DEFAULT_NODE_NAME = "cerulion_py"
DEFAULT_MAX_PAYLOAD_LEN = 1 << 20

_session = None
_session_lock = threading.Lock()


def connect() -> "Session":
    """Initialise the process-wide transport and return THE ``Session``.

    Idempotent - every call returns the same object.
    """
    global _session
    with _session_lock:
        if _session is None:
            _native.connect(_DEFAULT_NODE_NAME)
            _session = Session()
        return _session


class Session:
    """Handle on the process-wide transport.

    ``with cerulion.connect() as s:`` works, but ``__exit__`` is a no-op:
    the transport is process-wide and lives until process exit.
    """

    def publisher(self, topic, schema_hash, *, max_payload_len=DEFAULT_MAX_PAYLOAD_LEN):
        """Create a publisher on ``topic`` (str) with wire ``schema_hash``
        (int). ``max_payload_len`` bounds every frame's body bytes."""
        return Publisher(_native.Publisher(topic, schema_hash, max_payload_len))

    def subscriber(self, topic, depth=1):
        """Create a subscriber on ``topic`` (str) with queue ``depth``
        (1..=16 - the core rejects anything else)."""
        if depth is None:
            raise TypeError("depth must be an int, not None")
        return Subscriber(_native.Subscriber(topic, depth))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        # Deliberately a no-op: the transport is process-wide and lives
        # until process exit.
        return False


class Publisher:
    def __init__(self, native):
        self._native = native

    @property
    def topic(self):
        return self._native.topic

    @property
    def schema_hash(self):
        return self._native.schema_hash

    @property
    def max_payload_len(self):
        return self._native.max_payload_len

    @property
    def sequence(self):
        return self._native.sequence

    def publish(self, payload, timestamp_ns=None):
        """Single-copy publish: ``payload`` is copied once into a
        shared-memory loan.

        The buffer must expose single-byte items: ``bytes``,
        ``bytearray``, a ``memoryview`` of bytes, or a ``np.uint8``
        array - use ``arr.view(np.uint8)`` for other dtypes.
        """
        try:
            self._native.publish(payload, timestamp_ns)
        except BufferError as e:
            raise TypeError(
                "payload must be a contiguous bytes-like object of "
                "single-byte items (bytes, bytearray, memoryview of "
                "bytes, np.uint8 array); for other dtypes use "
                "arr.view(np.uint8)"
            ) from e

    def loan(self, payload_len):
        """Take a zero-copy writable SHM loan for a ``payload_len``-byte
        body. Commit with ``loan.commit()`` (or the ``with`` block's
        normal exit); ``discard()`` frees the slot unsent."""
        return Loan(self._native.loan(payload_len))


class Loan:
    def __init__(self, native):
        self._native = native

    @property
    def payload(self):
        """Writable memoryview over the loan's payload region
        (``len == payload_len``)."""
        return memoryview(self._native)

    @property
    def payload_len(self):
        return self._native.payload_len

    @property
    def timestamp_ns(self):
        return self._native.timestamp_ns

    @timestamp_ns.setter
    def timestamp_ns(self, value):
        self._native.timestamp_ns = value

    @property
    def is_open(self):
        return self._native.is_open

    def commit(self):
        """Stamp the wire header (sequence + timestamp) and send."""
        self._native.commit()

    def discard(self):
        """Return the SHM slot without sending."""
        self._native.discard()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is not None:
            self.discard()
        elif self.is_open:
            self.commit()
        return False


class Subscriber:
    def __init__(self, native):
        self._native = native
        self._iter_prev = None

    @property
    def topic(self):
        return self._native.topic

    @property
    def depth(self):
        return self._native.depth

    @property
    def max_borrowed_samples(self):
        return self._native.max_borrowed_samples

    def try_receive(self):
        """Pop at most one queued frame without blocking."""
        native = self._native.try_receive()
        return None if native is None else Frame(native)

    def receive(self, timeout_ms=None):
        """Pop the next frame, blocking up to ``timeout_ms``
        (None = forever). The GIL is released while waiting."""
        native = self._native.receive(timeout_ms)
        return None if native is None else Frame(native)

    def __iter__(self):
        return self

    def __next__(self):
        # Release the previous frame handed out by THIS iterator before
        # blocking for the next - one outstanding borrow per iterator.
        prev, self._iter_prev = self._iter_prev, None
        if prev is not None:
            prev.release()
        frame = self.receive(None)
        self._iter_prev = frame
        return frame


class Frame:
    def __init__(self, native):
        self._native = native

    @property
    def schema_hash(self):
        return self._native.schema_hash

    @property
    def sequence(self):
        return self._native.sequence

    @property
    def timestamp_ns(self):
        return self._native.timestamp_ns

    @property
    def total_size(self):
        return self._native.total_size

    @property
    def recv_ns(self):
        return self._native.recv_ns

    @property
    def is_released(self):
        return self._native.is_released

    @property
    def raw(self):
        """Read-only memoryview over the whole wire frame (header+body)."""
        return memoryview(self._native)

    @property
    def payload(self):
        """Read-only memoryview over the frame body (past the header)."""
        return self.raw[WIRE_HEADER_SIZE:]

    def as_numpy(self, dtype=np.uint8):
        """Zero-copy, read-only NumPy view over the payload bytes."""
        return np.frombuffer(self.payload, dtype=dtype)

    def to_bytes(self):
        """Materialise (copy) the payload into an owned ``bytes``."""
        return bytes(self.payload)

    def release(self):
        """Return the SHM slot to the publisher pool (idempotent)."""
        self._native.release()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False

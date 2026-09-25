"""Pure-Python facade over ``cerulion._native``.

One process-wide transport (``connect()`` is idempotent), facade classes
that delegate to the native objects and add iteration / context-manager
/ NumPy conveniences.
"""

import threading
import weakref

import numpy as np

from cerulion import _native
from cerulion._native import (
    WIRE_HEADER_SIZE,
    BorrowLimitExceeded,
    CerulionError,
    EncodeError,
    DecodeError,
    ReleasedFrame,
    SchemaError,
    SchemaMismatch,
    TransportError,
    real_ns,
)
from cerulion._typed import (
    Layout,
    Message,
    SchemaSet,
    _dynamic_descriptor,
    _encode_message,
    _wire_length,
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

    def publisher(self, topic, schema_hash=None, *, schema=None, schemas=None,
                  max_payload_len=DEFAULT_MAX_PAYLOAD_LEN):
        """Create a publisher on ``topic`` (str) with wire ``schema_hash``
        (int). ``max_payload_len`` bounds every frame's body bytes."""
        if (schema_hash is None) == (schema is None):
            raise TypeError("exactly one of schema_hash or schema is required")
        if schema is not None and schemas is None:
            raise TypeError("typed publisher requires schemas")
        if schema is not None:
            schema_hash = schemas.schema_hash(schema)
        return Publisher(_native.Publisher(topic, schema_hash, max_payload_len),
                         schemas=schemas, schema=schema)

    def subscriber(self, topic, depth=1, *, schema=None, schemas=None):
        """Create a subscriber on ``topic`` (str) with queue ``depth``
        (1..=16 - the core rejects anything else)."""
        if depth is None:
            raise TypeError("depth must be an int, not None")
        if (schema is None) != (schemas is None):
            raise TypeError("schema and schemas must be provided together")
        return Subscriber(_native.Subscriber(topic, depth), schemas=schemas, schema=schema)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        # Deliberately a no-op: the transport is process-wide and lives
        # until process exit.
        return False


class Publisher:
    def __init__(self, native, schemas=None, schema=None):
        self._native = native
        self._schemas = schemas
        self._schema = schema
        # A typed publisher is bound to the schema AS IT WAS at creation:
        # mutating the set afterwards must not silently change the hash
        # this publisher stamps. publish()/loan() re-check via
        # `_check_schema_binding`.
        if schema is not None:
            self._bound_generation = schemas._generation
            self._bound_hash = native.schema_hash

    def _check_schema_binding(self):
        if self._schemas._generation == self._bound_generation:
            return
        new_hash = self._schemas.schema_hash(self._schema)
        if new_hash != self._bound_hash:
            error = SchemaMismatch(
                f"schema {self._schema!r} changed after the publisher was "
                f"created (hash {self._bound_hash:#x} -> {new_hash:#x}); "
                "create a new publisher"
            )
            error.kind = "SchemaMismatch"  # match the native-raised shape
            raise error
        self._bound_generation = self._schemas._generation

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
        if self._schema is not None:
            self._check_schema_binding()
            frame = _encode_message(self._schemas, self._schema, payload, timestamp_ns)
            self._native.publish_frame(frame, timestamp_ns)
            return
        try:
            self._native.publish(payload, timestamp_ns)
        except BufferError as e:
            raise TypeError(
                "payload must be a contiguous bytes-like object of "
                "single-byte items (bytes, bytearray, memoryview of "
                "bytes, np.uint8 array); for other dtypes use "
                "arr.view(np.uint8)"
            ) from e

    def loan(self, payload_len=None, **variable_lengths):
        """Take a zero-copy writable SHM loan for a ``payload_len``-byte
        body. Commit with ``loan.commit()`` (or the ``with`` block's
        normal exit); ``discard()`` frees the slot unsent."""
        if self._schema is not None:
            self._check_schema_binding()
            layout = self._schemas.layout(self._schema)
            unknown = set(variable_lengths) - set(field.name for field in layout.variable_fields)
            if unknown:
                raise TypeError(f"unknown variable field(s): {', '.join(sorted(unknown))}")
            lengths = []
            for field in layout.variable_fields:
                length = int(variable_lengths.get(field.name, 0))
                if length < 0:
                    raise ValueError(f"length for {field.name} must be non-negative")
                lengths.append(_wire_length(field.field_type, length))
            native = self._native.loan_typed(
                self._schemas._native, self._schema, lengths, None
            )
            loan = Loan(native)
            entries = native.variable_entries() or []
            descriptors = {}
            for field, entry in zip(layout.variable_fields, entries):
                name, offset, byte_len = entry
                descriptors[name] = _dynamic_descriptor(field.field_type, offset, byte_len)
            message = Message(memoryview(native), layout, self._schemas, descriptors, loan)
            return _TypedLoanContext(loan, message)
        if payload_len is None:
            raise TypeError("payload_len is required for raw loans")
        return Loan(self._native.loan(payload_len))

    def publish_frame(self, frame_bytes, timestamp_ns=None):
        """Publish a complete wire frame, preserving its offset table."""
        self._native.publish_frame(frame_bytes, timestamp_ns)


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

    def _check_alive(self):
        if not self.is_open:
            raise ReleasedFrame("loan is closed")

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


class _TypedLoanContext:
    def __init__(self, loan, message):
        self._loan = loan
        self._message = message

    def __enter__(self):
        return self._message

    def __exit__(self, exc_type, exc, tb):
        self._message._detach()
        if exc_type is not None:
            self._loan.discard()
        elif self._loan.is_open:
            try:
                self._loan.commit()
            except _native.EncodeError as e:
                # Field views (numpy arrays / memoryviews) obtained inside
                # the block still export the slot. discard() defers the
                # slot's return to the last __releasebuffer__, so nothing
                # leaks and nothing is sent.
                self._loan.discard()
                raise _native.EncodeError(
                    "typed loan field views escaped the `with` block (live "
                    "buffer exports); delete them or use Message.copy() "
                    "before the block exits - the loan was discarded, "
                    "nothing was sent"
                ) from e
        return False


class Subscriber:
    def __init__(self, native, schemas=None, schema=None):
        self._native = native
        self._schemas = schemas
        self._schema = schema
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
        return None if native is None else self._frame(native)

    def receive(self, timeout_ms=None):
        """Pop the next frame, blocking up to ``timeout_ms``
        (None = forever). The GIL is released while waiting."""
        native = self._native.receive(timeout_ms)
        return None if native is None else self._frame(native)

    def _frame(self, native):
        frame = Frame(native)
        frame._typed = (self._schemas, self._schema) if self._schema is not None else None
        return frame

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
        self._typed = None
        self._message = None
        self._message_key = None

    def _check_alive(self):
        if self.is_released:
            raise ReleasedFrame("frame already released")

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
        cached = self._message() if self._message is not None else None
        if cached is not None:
            cached._detach()
        self._message = None
        self._native.release()

    def view(self, schemas=None, schema=None):
        from cerulion._typed import Message
        if schemas is None and schema is None:
            if self._typed is None:
                raise ValueError("frame has no typed schema binding")
            schemas, schema = self._typed
        elif schemas is None or schema is None:
            raise TypeError("schemas and schema must be provided together")
        self._check_alive()
        key = (id(schemas), schemas._generation, schema)
        cached = self._message() if self._message is not None else None
        if cached is not None and self._message_key == key:
            return cached
        descriptor = schemas._native.resolve_frame(self._native, schema)
        layout = schemas.layout(descriptor["schema"])
        message = Message(self.payload, layout, schemas, descriptor["variables"], self)
        # Weak: a strong cache would cycle with ``Message._owner`` and pin
        # the slot's buffer export until cyclic GC.
        self._message = weakref.ref(message)
        self._message_key = key
        return message

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False

"""Finalized MCAP bag reading facade."""

from __future__ import annotations

import struct
from typing import Any, NamedTuple, TypeVar, overload

from cerulion import _native
from cerulion._native import BagError, WIRE_HEADER_SIZE
from cerulion._typed import Message, SchemaSet, _schema_arg, _view, _view_arguments

M = TypeVar("M", bound=Message)


class TopicInfo(NamedTuple):
    name: str
    schema_name: str
    schema_hash: int
    count: int


class Record:
    def __init__(self, raw):
        self._raw = bytes(raw)
        if len(self._raw) < WIRE_HEADER_SIZE:
            raise BagError(
                f"bag record is {len(self._raw)} bytes, shorter than the "
                f"{WIRE_HEADER_SIZE}-byte wire header"
            )
        (
            self._schema_hash,
            self._total_size,
            _offset_table_offset,
            _offset_table_count,
            self._sequence,
            self._timestamp_ns,
            self._step_tag,
        ) = struct.unpack_from("<QIIIIQQ", self._raw)
        if self._total_size != len(self._raw):
            raise BagError(
                f"bag record total_size={self._total_size} does not match "
                f"record length {len(self._raw)}"
            )

    def _check_alive(self):
        return None

    @property
    def schema_hash(self):
        return self._schema_hash

    @property
    def total_size(self):
        return self._total_size

    @property
    def sequence(self):
        return self._sequence

    @property
    def timestamp_ns(self):
        return self._timestamp_ns

    @property
    def step_tag(self):
        """The wire header's step tag as an int (0 means untagged)."""
        return self._step_tag

    @property
    def raw(self):
        return self._raw

    @property
    def payload(self):
        return memoryview(self._raw)[WIRE_HEADER_SIZE:]

    def to_bytes(self):
        return bytes(self.payload)

    @overload
    def view(self, schema: "type[M]") -> "M": ...
    @overload
    def view(self, schema: str, *, schemas: "SchemaSet | None" = None) -> Any: ...
    def view(self, schema, *extra, schemas=None):
        """The typed view of this record as ``schema``: a message class, or a
        schema name with ``schemas=``."""
        _view_arguments(extra, "view()")
        schemas, schema = _schema_arg(schema, schemas, "view()")
        descriptor = schemas._native.resolve_frame_buffer(self.raw, schema)
        layout = schemas.layout(descriptor["schema"])
        return _view(self.payload, layout, schemas, descriptor["variables"], self)


class _Messages:
    """The ``(topic, Record)`` iterator ``Bag.messages()`` returns.

    A thin shell over the native iterator rather than a generator: a finished
    generator keeps raising ``StopIteration`` on its own, so it would never
    reach the native closed-bag check and a drained public iterator would not
    report ``bag.close()`` the way the native one does.
    """

    __slots__ = ("_native",)

    def __init__(self, native):
        self._native = native

    def __iter__(self):
        return self

    def __next__(self):
        topic, raw = next(self._native)
        return topic, Record(raw)


class Bag:
    def __init__(self, native):
        self._native = native

    def topics(self):
        return [TopicInfo(*topic) for topic in self._native.topics()]

    def messages(self, topics=None):
        if isinstance(topics, str):
            topics = [topics]
        elif topics is not None:
            topics = list(topics)
        return _Messages(self._native.messages(topics))

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def open_bag(path):
    return Bag(_native.open_bag(path))

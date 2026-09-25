"""Finalized MCAP bag reading facade."""

from __future__ import annotations

import os
import struct
from typing import NamedTuple

from cerulion import _native
from cerulion._native import BagError, WIRE_HEADER_SIZE
from cerulion._typed import Message


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
        ) = struct.unpack_from("<QIIIIQ", self._raw)
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
    def raw(self):
        return self._raw

    @property
    def payload(self):
        return memoryview(self._raw)[WIRE_HEADER_SIZE:]

    def to_bytes(self):
        return bytes(self.payload)

    def view(self, schemas=None, schema=None):
        if schemas is None or schema is None:
            raise TypeError("schemas and schema must be provided together")
        descriptor = schemas._native.resolve_frame(self.raw, schema)
        layout = schemas.layout(descriptor["schema"])
        return Message(self.payload, layout, schemas, descriptor["variables"], self)


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
        return (
            (topic, Record(raw))
            for topic, raw in self._native.messages(topics)
        )

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def open_bag(path):
    return Bag(_native.open_bag(os.fsdecode(os.fspath(path))))

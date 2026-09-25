"""cerulion - raw zero-copy Python client for the Cerulion transport.

One process-wide iceoryx2 shared-memory transport on the local host
lives behind ``connect()`` (cross-host zenoh is not exposed by this
client yet);
publishers and subscribers exchange whole wire frames over shared
memory. See ``docs/python.md`` for the full contract.
"""

from cerulion._native import (
    WIRE_HEADER_SIZE,
    BorrowLimitExceeded,
    CerulionError,
    EncodeError,
    DecodeError,
    BagError,
    ReleasedFrame,
    SchemaError,
    SchemaMismatch,
    TransportError,
    real_ns,
)
from cerulion._typed import Layout, Message, SchemaSet
from cerulion._bag import Bag, Record, TopicInfo, open_bag

from cerulion._api import (
    DEFAULT_MAX_PAYLOAD_LEN,
    Frame,
    Loan,
    Publisher,
    Session,
    Subscriber,
    connect,
)
from cerulion._node import NodeContext, _Runtime, input, node, output

__version__ = "0.1.0"

__all__ = [
    "BorrowLimitExceeded",
    "Bag",
    "BagError",
    "CerulionError",
    "DEFAULT_MAX_PAYLOAD_LEN",
    "EncodeError",
    "DecodeError",
    "Frame",
    "Loan",
    "Publisher",
    "ReleasedFrame",
    "SchemaMismatch",
    "SchemaError",
    "SchemaSet",
    "Layout",
    "Message",
    "Record",
    "Session",
    "Subscriber",
    "TransportError",
    "TopicInfo",
    "WIRE_HEADER_SIZE",
    "connect",
    "open_bag",
    "real_ns",
    "NodeContext",
    "input",
    "node",
    "output",
]

# Python client (`cerulion`)

A raw, zero-copy Python client for the Cerulion transport: iceoryx2 shared
memory on the local host; cross-host (zenoh) transport is not exposed by
this client yet. Frames are 32-byte wire headers plus
an opaque payload; there is no serialization envelope and no type registry.

## Install

```bash
pip install maturin
cd crates/cerulion_py && maturin develop --release
```

## Connect

One process-wide transport lives behind `connect()`. Calling it again returns
the same `Session`; the session is also a no-op context manager.

```python
import cerulion

session = cerulion.connect()
print(cerulion.connect() is session)
```

```text
True
```

## Publish

`session.publisher(topic, schema_hash, max_payload_len=N)` creates a
publisher. `schema_hash` identifies the payload layout to receivers; this
client treats payloads as opaque bytes.

`publish(payload, timestamp_ns=...)` copies a contiguous buffer into one
shared-memory loan slot and stamps the wire header (schema hash, sequence,
timestamp) - a **single-copy publish**. The buffer must expose single-byte
items: `bytes`, `bytearray`, `memoryview` of bytes, or `np.uint8` arrays.
For other dtypes pass `arr.view(np.uint8)`.

`loan(n)` hands out a writable view onto a loan slot - the **zero-copy**
publish path. The slot is zero-initialised. Committing sends it; discarding
(or leaving the context manager on an exception) drops it. A live exported
view blocks `commit()` - drop the view first.

```python
pub = session.publisher("/cerulion_py/docs", schema_hash=0xC0DE, max_payload_len=1024)
pub.publish(b"hello", timestamp_ns=42)
with pub.loan(4) as loan:
    loan.payload[:] = b"data"
    loan.timestamp_ns = 7
print(pub.topic, pub.sequence)
```

```text
/cerulion_py/docs 2
```

## Subscribe

`session.subscriber(topic, depth=N)` buffers up to `depth` frames
(1 to 16; the oldest is dropped past it). `receive(timeout_ms)` blocks until a
frame arrives (releasing the GIL) and returns a `Frame` or `None` on
timeout; `try_receive()` is the non-blocking form. Iterating releases the
previous frame automatically.

```python
sub = session.subscriber("/cerulion_py/docs", depth=4)
pub.publish(b"hello", timestamp_ns=42)
frame = sub.receive(1000)
print(frame.sequence, frame.timestamp_ns, frame.to_bytes())
```

```text
2 42 b'hello'
```

```python
pub.publish(b"data", timestamp_ns=7)
frame = sub.receive(1000)
print(frame.sequence, frame.timestamp_ns, frame.to_bytes())
frame.release()
```

```text
3 7 b'data'
```

## Zero-copy contract

Received `Frame`s are views onto **shared memory - no copy happens on
receive**. `frame.raw`, `frame.payload`, and `frame.as_numpy()` are
read-only views (the segment is mapped read-only). `frame.to_bytes()`
materialises the payload - that call **copies**.

Held frames hold borrowed loan slots; the budget is
`sub.max_borrowed_samples` (currently 2). `release()` returns the slot - but
if you still hold a live view (`frame.payload`, `frame.as_numpy()`, …), the
slot stays borrowed until the view dies. Exhaust the budget and
`receive()` raises `BorrowLimitExceeded`; release frames to recover.

```python
pub.publish(b"!")
frame = sub.receive(1000)
a = frame.as_numpy()
print(a.flags.writeable, len(frame.payload))
frame.release()
del a
```

```text
False 1
```

## Errors

Client errors derive from `cerulion.CerulionError`:

- `TransportError` - transport failures (bad topic, depth outside the
  ceiling, connect-time errors, receive errors).
- `SchemaMismatch` - schema hash disagreement (reserved for future
  type-asserting helpers; raw frames are not type-checked).
- `BorrowLimitExceeded` - received frames held past the borrow budget;
  `release()` frames to recover.
- `ReleasedFrame` - use of a released frame.
- `EncodeError` - facade-side length checks only: a payload larger than
  `max_payload_len`, or committing a loan while a live buffer view
  exists. A core `LoanCapacity` failure (loan-pool exhaustion) maps to
  `TransportError`, not `EncodeError`.

Invalid arguments raise built-in exceptions instead: `TypeError` for a
wrong argument type or a non-contiguous or non-byte buffer, `ValueError`
for using a loan after `commit()` or `discard()`, and `OverflowError` for
an integer outside its range (a negative `timeout_ms`, say).

```python
try:
    session.subscriber("/cerulion_py/docs/err", depth=0)
except cerulion.TransportError as e:
    print(type(e).__name__)
```

```text
TransportError
```

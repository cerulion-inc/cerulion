# Python client (`cerulion`)

A raw, zero-copy Python client for the Cerulion transport: iceoryx2 shared
memory on the local host; cross-host (zenoh) transport is not exposed by
this client yet. Frames are 32-byte wire headers plus
an opaque payload; there is no serialization envelope and no type registry.

## Install

```bash
pip install maturin
cd crates/cerulion_py && RUSTUP_TOOLCHAIN=1.93.0 maturin develop --release
```

## Writing nodes in Python

Python-authored nodes are standalone node `cdylib` libraries built against the current node ABI. The CLI creates
the embedding shim, while `node.py` contains the node logic:

```python-source
import cerulion

@cerulion.node(period_ms=10)
class Counter:
    inp = cerulion.input("test_msgs/Inp", depth=1)
    out = cerulion.output("test_msgs/Out")

    def tick(self):
        self.out.value = self.inp.value
```

Create and build one with:

```shell
cerulion node create counter --lang python -i test_msgs/Inp inp -o test_msgs/Out out --policy period_ms=10
cerulion node build counter
```

`node build` resolves the interpreter in this order: `$CERULION_PYTHON`,
`<workspace>/.venv/bin/python`, then `python3` on `PATH`. It imports
`nodes/counter/node.py`, regenerates the baked metadata block in `src/lib.rs`,
and invokes Cargo. The Python process receives `CERULION_WORKSPACE` so schema
declarations resolve against the same workspace used by the graph runtime.
At run time a Python node resolves schemas against `CERULION_WORKSPACE` when it
is set, otherwise against the workspace that contains its `nodes/<type>/`
directory, so `cerulion graph run` works from any subdirectory.
A Python node exports no state capture or restore: it behaves like a Rust node
that declares no state, so it starts from `init` on every run.
ROS 2 built-in messages such as `geometry_msgs/*` and `sensor_msgs/*` resolve
without a `schemas/` entry. A workspace schema with the same qualified name
overrides the built-in.
The built cdylib carries an rpath to the interpreter's libdir, so no
`LD_LIBRARY_PATH` is needed; rebuilding after switching interpreters refreshes it.
`node build` bakes the interpreter's site-packages so the node imports the same
`cerulion` you installed; `CERULION_PY_PATH` prepends paths for overrides.
Node cdylibs must be built by the same `rustc` as the `cerulion` binary:
`node build` warns when the `rustc` on `PATH` differs, and the loader refuses a
node whose compiler fingerprint does not match. Use `RUSTUP_TOOLCHAIN` only as a
per-command prefix for Python wheel commands.

A Python node must declare exactly one scheduling policy: `period_ms`, one
`trigger=True` input, or `sync_window_ms` for multiple trigger inputs. The
`node create --lang python` command (alias `node new`) therefore requires `-T` or `--policy`.

The embedded host supports one interpreter per process and serializes Python
execution through the GIL. Python-created threads and `fork()` are unsupported.
The host installs no Python signal handlers and never changes the process's
SIGINT disposition; graph shutdown stays with the runtime. Build one host
cdylib per CPython minor.
This path is intended for control and
integration nodes, not kHz loops: every tick crosses into the interpreter.

```python
print("Python nodes are built as ordinary node cdylibs.")
```

```text
Python nodes are built as ordinary node cdylibs.
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

## Schemas

Typed publishers and subscribers use a `SchemaSet` loaded from Cerulion YAML.
`schemas.layout("Point")` returns the cached fixed/variable layout and
`schemas.schema_hash("Point")` returns its wire hash. Adding another schema
invalidates cached layout objects.
Fixed fields are exposed as Python scalars or read-only NumPy views, while
variable arrays remain views into the received frame. `publish()` accepts a
dictionary or a `Message`; it materialises the complete wire frame once.

```python
schemas = cerulion.SchemaSet()
schemas.add_yaml("""
schemas:
  Point:
    fields:
      float64 x: {}
      float64 y: {}
""")
point_pub = session.publisher("/cerulion_py/docs/typed", schema="Point", schemas=schemas)
point_sub = session.subscriber("/cerulion_py/docs/typed", schema="Point", schemas=schemas)
point_pub.publish({"x": 1.5, "y": -2.0})
point_frame = point_sub.receive(1000)
print(point_frame.view().copy())
point_frame.release()
```

```text
{'x': 1.5, 'y': -2.0}
```

## Typed messages

`Publisher.loan(**lengths)` reserves a writable shared-memory slot for a
typed message. `Frame.view()` maps a received frame without copying; its
fixed arrays and variable arrays are read-only views. Use `.copy()` when a
materialized, writeable owned dictionary is needed. A dictionary passed to
`publish()` is encoded into a complete frame and therefore is materialized
before the single transport copy.

Field views handed out inside a `loan()` block (NumPy arrays and raw
memoryviews over the slot) are block-scoped: if one is still alive when
the `with` block exits, commit fails, the loan is discarded unsent, and
`EncodeError` is raised - delete the view or use `.copy()` first. On a
loan, a `string[]` or nested-message `Type[]` field is exposed as the raw
pre-framed byte slice, and its `loan(**lengths)` keyword is the framed
BYTE length; the same fields on `publish()` accept only pre-framed bytes
(element-wise encoding of `string[]`/`nested[]` is not supported).

```python
with point_pub.loan() as point:
    point.x = 2.5
    point.y = -4.0
point_frame = point_sub.receive(1000)
point = point_frame.view()
print(point.x, point.copy())
point_frame.release()
```

```text
2.5 {'x': 2.5, 'y': -4.0}
```

## Errors

Every exception derives from `cerulion.CerulionError`:

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
- `DecodeError` - malformed wire layout or invalid UTF-8 in a fixed or
  variable string field of a received frame.
- `SchemaError` - invalid schema documents, unknown schemas, or incompatible
  fixed layouts.

```python
try:
    session.subscriber("/cerulion_py/docs/err", depth=0)
except cerulion.TransportError as e:
    print(type(e).__name__)
```

```text
TransportError
```

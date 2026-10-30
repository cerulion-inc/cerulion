# Python client (`cerulion`)

A raw, zero-copy Python client for the Cerulion transport: iceoryx2 shared
memory on the local host; cross-host (zenoh) transport is not exposed by
this client yet. Frames are 40-byte wire headers plus
an opaque payload; there is no serialization envelope and no type registry.

## Install

```bash
pip install maturin
cd crates/cerulion_py && RUSTUP_TOOLCHAIN=1.95.0 maturin develop --release
```

## Writing nodes in Python

Python-authored nodes are standalone node `cdylib` libraries built against the current node ABI. The CLI creates
the embedding shim, while `node.py` contains the node logic:

```python-source
import cerulion
from cerulion.msgs.std_msgs import Int32

@cerulion.node(period_ms=10)
class Counter:
    inp = cerulion.input(Int32, depth=1)
    out = cerulion.output(Int32)

    def tick(self):
        msg = self.inp
        if msg is None:
            return
        self.out.data = msg.data
```

Create and build one with:

```shell
cerulion node create counter --lang python -i std_msgs/Int32 inp -o std_msgs/Int32 out --policy period_ms=10
cerulion node build counter
```

`node build` resolves the interpreter in this order: `$CERULION_PYTHON`,
`<workspace>/.venv/bin/python`, then `python3` on `PATH`. It imports
`nodes/counter/node.py`, regenerates the baked metadata block in `src/lib.rs`,
writes the workspace's message stubs (see [Message classes](#message-classes-cerulionmsgs)),
and invokes Cargo. The Python process receives `CERULION_WORKSPACE` so schema
declarations resolve against the same workspace used by the graph runtime.
At run time a Python node resolves schemas against `CERULION_WORKSPACE` when it
is set, otherwise against the workspace that contains its `nodes/<type>/`
directory, so `cerulion graph run` works from any subdirectory.
A Python node exports no state capture or restore: it behaves like a Rust node
that declares no state, so it starts from `init` on every run.
Its inputs follow the same discipline as a Rust node's: a periodic node reads
the latest frame of each input on every tick, and a tick with no new frame
re-reads the last one; a `trigger=True` input is served one frame per fire; a
`sync_window_ms` node reads the aligned set the scheduler chose.
ROS 2 built-in messages such as `geometry_msgs/*` and `sensor_msgs/*` resolve
without a `schemas/` entry. A workspace schema with the same qualified name
overrides the built-in; the built-in messages that embed the overridden one
(`geometry_msgs/Vector3Stamped` for `geometry_msgs/Vector3`) are dropped with a
warning rather than read against a layout they no longer have. The Rust build
applies the same rule, so both languages see the same set of messages.
The built cdylib carries an rpath to the interpreter's libdir, so no
`LD_LIBRARY_PATH` is needed; rebuilding after switching interpreters refreshes it.
`node build` bakes the interpreter's site-packages (and its user site, when
that interpreter enables one) so the node imports the same `cerulion` you
installed; their `.pth` files are processed, so an editable install imports
too. `CERULION_PY_PATH` prepends paths for overrides. A `helpers.py` beside
`node.py` is private to that node type: an import of it, at module level or
inside `init`, `tick` or `shutdown`, resolves in that node's directory, and two
node types in one process each keep their own, even when other code imported a
module of the same name first. Two instances of one node type share its
modules; a rebuilt node loads them afresh.
Node cdylibs must be built by the same `rustc` as the `cerulion` binary:
`node build` warns when the `rustc` on `PATH` differs, and the loader refuses a
node whose compiler fingerprint does not match. When the default `rustc`
differs from the one that built `cerulion`, prefix `cerulion node build` with
`RUSTUP_TOOLCHAIN=<that toolchain>` for that command only.

A Python node must declare exactly one scheduling policy: `period_ms`, one
`trigger=True` input, or `sync_window_ms` for multiple trigger inputs. The
`node create --lang python` command (alias `node new`) therefore requires `-T` or `--policy`.
A `trigger=True` input is refused under `period_ms`, and under `trigger=` unless it
names that same input; `trigger` must be `True` or `False`.
A Python node takes `-i` and `-o` repeatedly; a `sync_window_ms` node needs at
least two `-i` inputs, and each one joins the aligned set (`node modify` does not
edit Python nodes; edit `node.py` and run `cerulion node build`).

The embedded host supports one interpreter per process and serializes Python
execution through the GIL. Python-created threads and `fork()` are unsupported.
The host installs no Python signal handlers and never changes the process's
SIGINT disposition; graph shutdown stays with the runtime. Build one host
cdylib per CPython minor.
This path is intended for control and
integration nodes, not kHz loops: every tick crosses into the interpreter.

An output is loaned on first touch (`self.out`) and committed at the end of
the tick. A variable-length output is loaned explicitly with its lengths as
element counts: `self.loan("out", name=5, samples=3)` reserves five characters
and three doubles. A node that marks one input `trigger=True` and names no
other policy is a data-trigger node on that input:

```python
import cerulion
from cerulion.msgs.std_msgs import Int32

@cerulion.node()
class Relay:
    cmd = cerulion.input(Int32, trigger=True)
    out = cerulion.output(Int32)

    def tick(self):
        self.out.data = self.cmd.data

print(Relay.__cerulion_policy__)
```

```text
{'data_trigger': {'input_name': 'cmd'}}
```

## Message classes (`cerulion.msgs`)

Every message is a class, one module per package: `cerulion.msgs.sensor_msgs`
holds `LaserScan`, the type a Rust node names as
`native_ros2_messages::sensor_msgs::LaserScan`. A workspace's own messages join the
same namespace from the one source file both languages read: a
`schemas/<package>/msg/<Type>.msg` file is
`from cerulion.msgs.<package> import <Type>`, a YAML schema named `pkg/Name`
is `cerulion.msgs.pkg.Name`, and a package-less YAML schema is
`cerulion.msgs.<Name>`. A package name's `-` is `_` in the module name.

The classes are built when a module is imported, from the built-ins under the
workspace `cerulion.msgs` is bound to. A node is bound to its own workspace
before its `node.py` is imported; two node types in one process each get their
own modules, the way they keep their own `helpers`. A script is bound to
`$CERULION_WORKSPACE` when it is set, else to the workspace that contains the
current directory (a `Cargo.toml` with `[workspace]` beside a `graphs/`
directory), else to the built-in messages alone.

A port, publisher, subscriber or view takes the class. Every view a schema set
opens of that schema is an instance of it, nested fields included, so
`isinstance(self.scan, LaserScan)` holds. A class names a type and cannot be
called: a value comes from a receive, a loan, or a `publish()` of a dict.
A class keeps the schema hash its set resolved it to; one used against a set
that resolves its name to a different hash (a class from another workspace, or
a set mutated since) raises `SchemaMismatch` naming both hashes.

```python
from cerulion.msgs.geometry_msgs import Vector3
from cerulion.msgs.std_msgs import Header

print(Vector3.__module__, Vector3.__cerulion_schema__)
print(hex(Vector3.__cerulion_schema_hash__))
print(issubclass(Header, cerulion.Message))
try:
    Vector3()
except TypeError as error:
    print(type(error).__name__)
```

```text
cerulion.msgs.geometry_msgs geometry_msgs/Vector3
0xd43ee5592039b9df
True
TypeError
```

A schema name in quotes still works where a class is expected:
`cerulion.input("pkg/Name")` in a node, `schema="Name", schemas=s` on a
publisher, subscriber or view. It is the escape hatch for a schema that has no
class in the bound set.

Type stubs come with the classes, so pyright and mypy check field names and
types (`self.scan.ranges` is a `float32` array). The wheel carries the stubs of
every built-in package. `cerulion node build` writes the stubs of the
workspace's set (its own packages and every built-in package) to
`<workspace>/.cerulion/python/cerulion-stubs/`, and `python -m cerulion.msgs`
writes them for a script's workspace. Point the type checker at that directory:
add `.cerulion/python` to pyright's `extraPaths`, or put the directory's
absolute path in a `.pth` file in the environment's `site-packages`, which
pyright and mypy both read.

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

Typed publishers and subscribers take a message class with `schema=`: one
from `cerulion.msgs`, or one a `SchemaSet` loaded from Cerulion YAML hands out
with `schemas.message_type("Point")`.
`schemas.layout("Point")` returns the cached fixed/variable layout and
`schemas.schema_hash("Point")` returns its wire hash. Adding another schema
invalidates cached layout objects and retires each class whose schema hash
it changed: take that class from the set again.
Fixed fields are exposed as Python scalars or read-only NumPy views;
primitive variable arrays remain views into the received frame, while
`string[]` and nested-message arrays are decoded into Python lists.
`publish()` accepts a dictionary or a `Message` and encodes it field by field
into a shared-memory loan, the path `loan()` takes. A received view passed to
`publish()` is forwarded as the frame bytes it arrived as (padding included;
the header is re-stamped). Fields are also reachable by item access,
`message["copy"]`, which is the path for a field whose name is also a
`Message` method or starts with an underscore. A view opened before
`add_yaml()`, and every nested view reached through it, refuses every field
access afterwards (`SchemaError`): open it again.

```python
schemas = cerulion.SchemaSet()
schemas.add_yaml("""
schemas:
  Point:
    fields:
      float64 x: {}
      float64 y: {}
""")
Point = schemas.message_type("Point")
point_pub = session.publisher("/cerulion_py/docs/typed", schema=Point)
point_sub = session.subscriber("/cerulion_py/docs/typed", schema=Point)
point_pub.publish({"x": 1.5, "y": -2.0})
point_frame = point_sub.receive(1000)
point = point_frame.view()
print(point.copy(), isinstance(point, Point))
point_frame.release()
```

```text
{'x': 1.5, 'y': -2.0} True
```

## Typed messages

`Publisher.loan(**lengths)` reserves a writable shared-memory slot for a
typed message; the lengths are keyword-only, so the raw form `loan(64)`
raises `TypeError` on a typed publisher. `Frame.view()` maps a received
frame without copying, as the subscriber's class (`Frame.view(T)` maps it as
class `T`, `Frame.view("Name", schemas=s)` by name); its fixed arrays and primitive variable arrays are
read-only views. Use `.copy()` when a materialized, writeable owned
dictionary is needed. A dictionary passed to `publish()` is encoded
straight into a loan (a nested message given as a dictionary is encoded
to its body bytes first); a field value that does not fit its type (a
float for an integer, an out-of-range integer, a finite float that
narrows to infinity in a `float32` field, a wrong-length array) raises
`EncodeError`, also inside a fixed array of nested messages, and nothing
is sent.

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

## Bags

`cerulion.open_bag(path) -> Bag` opens a finalized MCAP bag written by
`cerulion bag record` (see `docs/bag.md`); `Bag` is a context manager.
`topics()` returns `list[TopicInfo(name, schema_name, schema_hash, count)]`
in bag channel order, excluding reserved `__cerulion/` channels
(`schema_hash` is `0` for a channel whose schema encoding is not
`cerulion`). `messages(topics=None)` returns an iterator of
`(topic, Record)` in log order; `topics` is one topic name or an iterable
of names. `Record` exposes the header attributes `schema_hash`,
`total_size`, `sequence`, and `timestamp_ns`, plus `raw`, `payload`,
`to_bytes()`, and `view(schema)`.

Bag records are not zero-copy: the reader memory-maps the bag, and each
record is copied out of the map into owned Python bytes when the iterator
yields it. `messages()` streams the bag: the iterator keeps only its place
in the data section and the record it is yielding, so its memory does not
grow with the record count or the bag size, and a loop that stops early
never reads the rest. `topics()` takes its counts from the bag's summary
statistics, without a walk over the data. `Record.raw` owns the bytes,
`Record.payload` is a read-only memoryview over them, and records stay
valid after `bag.close()`; a `messages()` iterator, finished or not, raises
`BagError` once after its bag is closed and then stops.
`open_bag` verifies every chunk CRC in one pass before returning, so a
corrupted, truncated, or unfinalized bag raises `BagError` at open. Every
walk over the data (`open_bag`, each `messages()` iterator as it yields,
and `topics()` on a bag whose summary carries no statistics) releases the
mapped pages behind it, so resident memory stays bounded on large bags.
The one exception is a bag recorded by a release with the 32-byte wire header:
its frames are upgraded to the current 40-byte layout when the bag is first
walked, into one copy reserved at its exact size (the bag's frames plus 8 bytes
each) and held until the bag is closed. Such a bag whose copy would pass 2 GiB
raises `BagError` naming `cerulion bag migrate`, which writes a current copy of
it.
`Record.view(schema)` decodes a typed message from those bytes; `schema` is a
message class, or a schema name with `schemas=`.
A record whose embedded wire header is shorter than 40 bytes, or whose
`total_size` disagrees with the record length, raises `BagError`. An
unknown topic filter raises `ValueError`.

```python-source
import cerulion
from cerulion.msgs.geometry_msgs import Vector3

with cerulion.open_bag("session.mcap") as bag:
    for topic, record in bag.messages("/echo/echo/out"):
        message = record.view(Vector3)
        print(topic, record.sequence, message.x, message.y, message.z)
```

```python
try:
    cerulion.open_bag("/nonexistent/session.mcap")
except cerulion.BagError as e:
    print(type(e).__name__)
```

```text
BagError
```

## Replay-testing a policy

[`examples/policy_replay/`](../crates/cerulion_py/examples/policy_replay/)
holds a helper and a pytest template that replay a recorded bag through a
policy and assert its actions against the recorded ones. Copy both files next
to your tests. `recorded_steps(bag, observation_topics, action_topic)` pairs
each record on the action topic with the newest record on every observation
topic logged before it, in log order. That is the observation the policy acted
on when the robot logs a new observation only after publishing the action for
the previous one, and the recorder logged frames in the order the policy
received them. A frame logged while the policy was still computing pairs a
newer observation with that action, and the replayed action can then differ
from the recorded one (for a policy that keeps state, so can the steps after
it). Action records logged before every observation topic has a record are
counted in `skipped`, not replayed. `assert_replay_matches(steps, policy,
recorded_action, atol=0.0, rtol=0.0)` calls `policy(step)` on every step in
order and compares the result with `recorded_action(step.action)`. With both
tolerances zero the arrays must match in dtype, shape and bytes. Otherwise
each element must equal the recorded one or satisfy `abs(replayed - recorded)
<= atol + rtol * abs(recorded)`; under a tolerance, a NaN in either array is a
mismatch and an infinity matches only an equal infinity. A failure names the
first mismatching steps by index, sequence and timestamp. A policy that keeps
state between calls (an action chunk queue, temporal ensembling) reproduces
the recording only from the same starting state: start the recorder before the
policy acts, reset the policy once, and replay every step in order.

```python-source
from policy_replay import assert_replay_matches, recorded_steps

with cerulion.open_bag("session.mcap") as bag:
    steps = recorded_steps(bag, ["/camera/top/image", "/arm/joint_states"], "/arm/command")
    result = assert_replay_matches(steps, policy, recorded_action)
```

`test_policy_replay.py` loads a LeRobot checkpoint with its saved pre- and
postprocessors and runs the check when `REPLAY_BAG` and `REPLAY_POLICY` are
set. Byte-exact replay holds on CPU with deterministic algorithms on, when the
replay runs the same weights, library versions, CPU model and `torch` thread
count as the recording: set `REPLAY_THREADS` to the `torch.get_num_threads()`
the recording ran with (default 1), and leave `REPLAY_ATOL` and `REPLAY_RTOL`
unset. A GPU replay, or a recording made on another device type, matches
within a tolerance (`REPLAY_ATOL`, `REPLAY_RTOL`). A policy that samples noise
when it acts repeats its actions only if the recording seeded the same
generator the same way (the template seeds 0 before loading the checkpoint).

## Errors

Client errors derive from `cerulion.CerulionError`:

- `TransportError` - transport failures (bad topic, depth outside the
  ceiling, connect-time errors, receive errors).
- `SchemaMismatch` - schema hash disagreement: a message class used against a
  set that resolves its name to a different hash, a typed publisher whose set
  changed its schema, a node input frame of another schema. Raw frames are not
  type-checked.
- `BorrowLimitExceeded` - received frames held past the borrow budget;
  `release()` frames to recover.
- `ReleasedFrame` - use of a released frame.
- `EncodeError` - facade-side checks on what is about to be sent: a payload
  larger than `max_payload_len`, committing a loan while a live buffer view
  exists, a typed `publish()` payload that is not a dict or `Message` or
  has missing, extra or ill-typed fields, and a frame handed to a typed
  publisher's `publish_frame()` that is not well-formed for its schema. A
  core `LoanCapacity` failure (loan-pool exhaustion) maps to
  `TransportError`, not `EncodeError`.
- `DecodeError` - malformed wire layout or invalid UTF-8 in a fixed or
  variable string field of a received frame.
- `SchemaError` - invalid schema documents, unknown schemas, or incompatible
  fixed layouts.
- `BagError` - bag open or read failures: missing files, non-MCAP files,
  truncated or unfinalized bags, and malformed records.

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

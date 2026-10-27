# Frames and transforms: the tf buffer and `lookup`

A robot is a tree of coordinate frames: `map` holds `odom`, `odom` holds
`base_link`, `base_link` holds every sensor mount. A node that fuses a lidar
point into the map needs the transform between two of those frames at the
time the point was captured. The `cerulion_tf` crate gives a node one place
to collect the transforms it receives and one call to ask for that transform:
`lookup(parent, child, time)`.

Related: `docs/user-api.md`, "Accumulate inputs" (the input kind a `/tf`
listener declares).

## The layout: `/tf` and `/tf_static`

Transforms travel as `tf2_msgs/TFMessage`, a batch of
`geometry_msgs/TransformStamped`, on two topics, the ROS layout:

- `/tf` carries the transforms that move: `odom` to `base_link`, a joint,
  a tracked object. Many nodes publish on it, each its own edges.
- `/tf_static` carries the transforms that never change: sensor mounts, the
  fixed links of a URDF. A static transform is published once (or re-published
  harmlessly) and holds at every time.

A node that listens declares both as accumulate inputs. Each fire then
receives every `TFMessage` that arrived since the last fire, from every
writer, up to `cap` frames per input; past the cap the oldest frames are
dropped. Size `cap` to the number of `TFMessage`s that can arrive between two
fires: the broadcasters on the topic, their rates and the node's own rate,
with margin. Both topics have many writers, so the graph lists them in
`multi_publisher_topics`, where the cap is at most 16; a node that would
receive more than that between two fires must fire more often, and
`self.tf.dropped()` reports what a fire lost.

```rust
#[input(accumulate, cap = 16)]
tf: TFMessage,
#[input(accumulate, cap = 16)]
tf_static: TFMessage,
```

A static transform is published once, so `/tf_static` is latched: an output
on `/tf_static` with schema `tf2_msgs/TFMessage` keeps its last message for a
listener that starts later, with no `history_size` in the graph YAML:

```yaml
nodes:
  - id: mounts
    type: mounts
    outputs:
      - name: tf_static
        topic: /tf_static
        schema: tf2_msgs/TFMessage
```

The publisher sends its whole static set, merged by child frame, in every
message, so the one message it keeps holds every static transform it
publishes. Keep the set in a `TfBuffer`: `insert_static` for a child the
buffer already holds replaces that child's transform, and
`buffer.static_edges()` lists one edge per child frame (parent, child and
transform) to write into each `TFMessage`.

A late listener receives the kept message of every static publisher at once,
through its `tf_static` queue. The graph load checks every listener in the
graph: its `cap` must be at least the number of `/tf_static` outputs in the
graph times the messages each keeps, which is 1 unless an output sets a
larger `history_size`. A smaller cap refuses the graph with
`TfStaticCapTooSmall`, which names the listener, its cap and the publishers,
so a dropped static transform is never silent. On a `multi_publisher_topics`
topic the cap is at most 16; a graph whose static publishers need more is
refused too: move static transforms into fewer publishers. Publishers in
another graph are not counted, so a listener of another graph's static
publishers sets its cap itself: their number times the messages each keeps.
The topic's publishers share one history, the largest `history_size` any of
them declares and at least 1, so with no `history_size` that is their number.

A `/tf` output can keep `history_size` at 0. A listener that starts late then
holds no sample older than the first it receives, and a lookup at an earlier
stamp returns `OutOfRange`. When a listener must answer stamps from before it
started, give the `/tf` outputs history too, and set the listener's `tf` cap
to at least the number of `/tf` publishers times that history, within the
same limit of 16.

Each `TransformStamped` names its parent (`header.frame_id`), its child
(`child_frame_id`), its stamp (`header.stamp`) and the transform
`T_parent_child`: the child origin in parent coordinates and the rotation that
turns child axes into parent axes.

## The buffer

`cerulion_tf::TfBuffer` is a plain field of the node. It holds a frame tree
with a time window of stamped samples per edge, and static edges beside it.

The node's `Cargo.toml` adds the crate beside the two the scaffold writes:

```toml
[dependencies]
cerulion_core = { workspace = true }
native_ros2_messages = { workspace = true }
cerulion_tf = { workspace = true }
```

`cerulion_tf` is not on crates.io. A workspace created by a `cerulion` built
from a source checkout lists it under `[workspace.dependencies]` as a path
into that checkout, so the line above works as written. A workspace created by
a `cerulion` installed from crates.io does not list it: add
`cerulion_tf = { path = "<checkout>/crates/cerulion_tf" }` to the workspace's
root `Cargo.toml` under `[workspace.dependencies]`, with `<checkout>` a
`cerulion` source checkout of the same version as the CLI.

```rust
use cerulion_tf::{TfBuffer, TfError, Transform, UnitQuaternion};

let mut buffer = TfBuffer::new();

// A dynamic transform: odom -> base_link at 1 000 000 000 ns, 2 m along +x.
let rotation = UnitQuaternion::new(0.0, 0.0, 0.0, 1.0)?;
let transform = Transform::new([2.0, 0.0, 0.0], rotation)?;
buffer.insert("odom", "base_link", 1_000_000_000, transform)?;

// A static transform: the lidar mount, 0.25 m forward and 0.5 m up.
let mount = Transform::new([0.25, 0.0, 0.5], UnitQuaternion::IDENTITY)?;
buffer.insert_static("base_link", "lidar", mount)?;

// The lidar in odom, at the stamp of the sample above.
let odom_lidar = buffer.lookup("odom", "lidar", 1_000_000_000)?;
assert_eq!(odom_lidar.translation(), [2.25, 0.0, 0.5]);
```

The feed is one `insert` per `TransformStamped` the node receives on `/tf`
and one `insert_static` per entry on `/tf_static`. The stamp is
`header.stamp` in nanoseconds, `sec * 1_000_000_000 + nanosec` as a `u64`; a
negative `sec` has no place in the buffer, so refuse the message before
`insert`. The transform is built through `UnitQuaternion::new` and
`Transform::new`, which refuse a NaN, an infinite component and a zero
quaternion, so a malformed transform never reaches the buffer.

The rules:

- **One parent per frame.** A transform naming a child that already has a
  different parent is refused (`SecondParent`); so is a self edge and an edge
  that would make a frame its own ancestor (`Cycle`). The tree stays a tree.
- **One feed per edge.** The buffer does not know which node published a
  transform: an accumulate input does not say which node sent each frame,
  so every sample of an edge is one feed. Two nodes that publish the same
  edge interleave on its ring; publish each edge from one node.
- **A window of history.** Each dynamic edge keeps the samples of the last
  10 s before its newest stamp, the tf2 default, whatever its publish rate.
  An insert drops every sample older than that window (counted in
  `expired()`), and a stamp older than the window is refused
  (`StampBeforeWindow`). A stamp more than the jump limit past the newest
  sample is refused too (`StampTooFarAhead`): accepted, one bad stamp would
  expire the whole ring and leave every later good sample before the window.
  The jump limit is `TfBufferConfig::jump_limit`, 60 s by default, or the
  window when that is longer. It is apart from the window, so an edge that
  publishes less often than its window still updates, each sample replacing
  the last, as long as each gap is within the jump limit. The next insert
  confirms a refused jump when its stamp is later than the refused one and
  within the jump limit of it, so a publisher that paused for longer than
  that and resumes loses one sample, and a lone far-future stamp changes
  nothing. An edge whose every gap is longer than the jump limit never
  updates: each sample is refused and no later one confirms it, so raise
  `jump_limit` to at least the longest gap for such an edge. `Duration::MAX`
  turns the check off. Each refusal states the limit in force and names the
  setting. Both refusals come back every time and count in the edge's
  `window_refusals` and `jump_refusals`, read through `edge()`, so a stuck
  edge is observable. A sample at a stamp the ring already holds is a no-op
  when the transform is the same and replaces it when it differs (counted
  in `replacements()`).
- **Rings sized up front.** Each ring holds enough samples for the window at
  up to 100 Hz (1001 samples). An edge that publishes faster fills its ring
  before its window: the oldest sample makes room and each one counts in the
  edge's `overflows`, read through `edge()`, so a short window is never
  silent. Size the buffer for a faster edge with
  `TfBuffer::with_config(TfBufferConfig::new(frames, retention, max_rate_hz))`.
  That constructor is the only place the buffer allocates. `insert` and
  `lookup` never touch the heap, so they belong inside a tick.
- **Static wins.** A static edge holds at every time. A dynamic transform
  for a static edge is refused (`EdgeIsStatic`), and a static insert on an
  edge with dynamic history clears that history.
- **Every refusal comes back.** A refused insert returns its error every
  time: a sample the buffer did not keep is never an `Ok(())`, so no data is
  lost silently. The edge counts each kind of refusal as well
  (`static_refusals`, `window_refusals`, `jump_refusals`, read through
  `edge()`), so a node that would log every error on a 100 Hz edge can log
  the first and report the running total instead.
- **Frame ids** have no length limit of their own. The buffer interns each
  id once, in a name table sized at construction (64 bytes per frame on
  average, 4096 bytes for the default 64 frames), and names a frame by its
  `FrameId` everywhere else. Leading `/`s are removed first: `/odom`,
  `//odom` and `odom` name one frame, so a frame name read back from `edge()`
  names the same frame when passed back. The empty id, and an id of only
  `/`s, is refused (`EmptyFrameId`).
- **Capacity.** A transform naming a frame the buffer has no room for is
  refused (`FrameTableFull`), and so is one whose new id does not fit in what
  is left of the name budget (`NameBudgetFull`); both name the frame.

Every refusal happens before the buffer changes: a refused insert leaves it
exactly as it was, apart from the refusal counters above and the stamp a
`StampTooFarAhead` refusal keeps for the next insert to confirm.

### Time jumps

A bag that loops or a simulator that resets moves time backwards. Call
`buffer.begin_epoch()` at that point: it clears every dynamic ring, keeps
every frame, edge, policy and static transform, and counts the epoch
in `epoch()`. The new timeline then fills the rings from empty. Without it,
the first stamps of the new timeline are older than the window and come back
as `StampBeforeWindow`, so a missed reset is an error, never a frozen tree.

## Lookup

`lookup(parent, child, time)` returns `T_parent_child` at `time` (nanoseconds)
as a `Result<Transform, TfError>`. It walks from both frames up to their
nearest common ancestor and composes the edges it crosses, so it works
whether `child` descends from `parent`, `parent` descends from `child` (the
result is the inverse), or the two are siblings under a shared ancestor. On
each dynamic edge the sample at `time` is an exact hit, returned bit for bit,
or the interpolation between the two samples that bracket it: linear in
translation, shortest-arc SLERP in rotation.

`lookup_latest(parent, child)` resolves at the newest time every edge on
the chain answers, through the same code path: with the default policy, the
oldest of the edges' newest stamps.
A chain of static edges only has no time and resolves directly. With the
default window, a 1 Hz `map` to `odom` edge and a 100 Hz or 1 kHz `odom` to
`base_link` edge resolve at the slow edge's newest stamp.

A node that looks up the same frames on every tick resolves them once with
`buffer.frame_id("map")` and calls `lookup_ids(map, lidar, time)` and
`lookup_latest_ids`, which skip the string work. A handle belongs to the
buffer that issued it; another buffer refuses it with `UnknownFrameId`.

Time is a plain stamp: `lookup(parent, child, 0)` asks for 0 ns, which is out
of range on an edge whose oldest sample is later. For the ROS meaning of
`Time(0)`, the latest transform available, call `lookup_latest`.

### Past the newest sample: edge policies

By default a lookup one nanosecond after an edge's newest sample is
`NotYetAvailable`, never a guessed transform. An edge can answer further with
`buffer.set_edge_policy(parent, child, policy)`:

| Policy | Past the newest sample |
|---|---|
| `EdgePolicy::Interpolate` (default) | Nothing: `NotYetAvailable`. |
| `EdgePolicy::HoldLast { max_age_ns }` | The newest sample, for up to `max_age_ns`. For a slow edge such as `map` to `odom`. |
| `EdgePolicy::Extrapolate { max_ns }` | The motion between the two newest samples, continued for up to `max_ns`. For an edge that publishes behind the data it is used with. |

A policy can be set before the edge's first sample: the edge is declared with
an empty ring. `lookup_latest` counts an edge's policy as part of what it
answers (an `Extrapolate` edge with one sample has no motion yet, so it
reaches only that sample): with `HoldLast { max_age_ns: 1_000_000_000 }` on `map` to `odom`,
the chain to `base_link` resolves at the fast edge's newest stamp while the
slow edge's newest sample is at most 1 s older, and at that sample plus 1 s
when it is older than that. It never resolves later than the newest stamp
on the chain. A time before an edge's oldest
sample is `OutOfRange` under every policy.

**Lookups never block.** There is no listener thread and no timeout: an
error is a fact about the buffer at this fire, and the node asks again on its
next fire.

```rust
match self.buffer.lookup("map", "lidar", stamp_ns) {
    Ok(map_lidar) => {
        let p_map = map_lidar.apply_point(p_lidar);
        // ...
    }
    Err(TfError::NotYetAvailable { .. }) => {
        // The transform for this stamp has not arrived; try next fire.
    }
    Err(err) => tracing::warn!(error = %self.buffer.describe(&err), "tf lookup refused"),
}
```

### The errors

`TfError` is the crate's one error type: the math refusals of
`UnitQuaternion::new` and `Transform::new` and the buffer refusals are
variants of the same enum, so a feed function that builds a transform and
inserts it uses `?` on both. A frame the buffer has registered is named in
an error by its `FrameId`, which prints as `#n` on its own;
`buffer.describe(&err)` prints the error with the frame names, and
`buffer.frame_name(id)` gives one name. A frame id the buffer has not
registered is carried as a `FrameLabel`, its first 48 bytes. `TfMathError`,
the name of the math refusals before the buffer, stays as a deprecated
alias of `TfError`.

| Variant | When | What to do |
|---|---|---|
| `UnknownFrame { frame }` | No inserted transform has named `frame`. | Check the frame id; wait for the publisher. |
| `UnknownFrameId { frame }` | `lookup_ids` got a handle this buffer never issued, such as one from another buffer. | Resolve the handle from the same buffer. |
| `NoPath { parent, child }` | Both frames are known but no chain of edges joins them. | A broadcaster is missing an edge. |
| `EdgeEmpty { parent, child }` | A dynamic edge on the chain has no sample since it was declared or since the last epoch. | Ask again on the next fire. |
| `NotYetAvailable { parent, child, requested_ns, newest_ns }` | `requested_ns` is past the newest sample on the edge `parent` to `child`, further than its policy reaches. | Ask again on the next fire, or set an edge policy. |
| `OutOfRange { parent, child, requested_ns, oldest_ns, newest_ns }` | `requested_ns` is older than the oldest sample on that edge: it left the window, or was never buffered. | Waiting does not help; a longer retention or a fresher stamp does. |
| `ChainMath { parent, child }` | Composing the chain between `parent` and `child` overflowed `f64`. | The input data is wrong: a translation along the chain, or an extrapolated edge, is far too large. |
| `NonFiniteQuaternion`, `DegenerateQuaternion { norm }`, `NonFiniteTranslation` | `UnitQuaternion::new` or `Transform::new` refused a value in the feed, carried through `?`. | The input data is wrong. |
| `EmptyFrameId` | The frame id is empty (or only `/`s). | Fix the publisher's frame ids. |
| `SelfEdge { frame }` | Parent and child are the same frame. | Fix the publisher. |
| `SecondParent { child, parent, existing_parent }` | `child` already has a different parent. | Two broadcasters claim the same child; keep one. |
| `Cycle { parent, child }` | The edge would make a frame its own ancestor. | Fix the frame tree. |
| `EdgeIsStatic { parent, child }` | A dynamic transform or an edge policy arrived for a static edge. | Publish the edge on one topic only. |
| `StampBeforeWindow { parent, child, stamp_ns, newest_ns }` | The stamp is older than the window before the edge's newest sample. | Call `begin_epoch` on a clock reset or a bag loop. |
| `StampTooFarAhead { parent, child, stamp_ns, newest_ns, jump_limit_ns }` | The stamp is more than the jump limit in force (`jump_limit_ns`) past the edge's newest sample, and no insert just before it agrees. | Fix the publisher's stamps; a clock that moved forward is confirmed by its next stamp. An edge whose every gap is longer than the limit needs a larger `TfBufferConfig::jump_limit`. |
| `FrameTableFull { frame, capacity }` | The buffer holds `capacity` frames and `frame` is new. | Build the buffer with `TfBuffer::with_config` and a larger `TfBufferConfig::frames`. |
| `NameBudgetFull { frame, len, left, budget }` | The new frame's `len`-byte id does not fit in the `left` bytes of the name budget. | Build the buffer with `TfBuffer::with_config` and a larger `TfBufferConfig::name_bytes`. |

Errors are plain `Copy` values: building one allocates nothing, so a lookup
that fails on every fire costs no more than one that succeeds.

## Determinism

A lookup reads only recorded frames, and the math is a fixed sequence of
correctly rounded IEEE-754 operations with no platform math library and no
fused multiply-add. The walk over the tree is fixed by the tree, the name
index is an unseeded hash, an extrapolated edge composes its steps in a
fixed order, and the
interpolation fraction is formed from the stamp differences, so a stamp in
2023 wall-clock nanoseconds interpolates exactly like a stamp near zero. Two
runs over the same frames give bit-identical transforms, on every machine, so
a graph that does tf lookups replays byte-exact.

## Conventions

Quaternions are `(x, y, z, w)`, the field order of `geometry_msgs/Quaternion`,
with `w` the scalar part. A transform is `T_parent_child`; `apply_point` maps
child coordinates to parent coordinates. `T_a_b.compose(&T_b_c)` is `T_a_c`.
Frames follow REP-103: right-handed, metres, radians.

## Not supported

- Declaring static transforms in graph YAML, a command-line view of the frame
  tree, a robot state publisher from URDF, and a Python tf lookup API are not
  supported; a node publishes its transforms and holds its own buffer.
- A blocking lookup with a timeout.
- Re-parenting a frame at runtime.
- Telling two broadcasters of one edge apart. An accumulate input does not
  say which node published each frame, so the buffer cannot refuse a second
  broadcaster of an edge: their samples interleave on its ring. Publish each
  edge from one node.

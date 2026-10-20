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

A node that listens declares both as accumulate inputs, so no frame from any
writer is lost between two fires:

```rust
#[input(accumulate, cap = 32)]
tf: TFMessage,
#[input(accumulate, cap = 16)]
tf_static: TFMessage,
```

Each `TransformStamped` names its parent (`header.frame_id`), its child
(`child_frame_id`), its stamp (`header.stamp`) and the transform
`T_parent_child`: the child origin in parent coordinates and the rotation that
turns child axes into parent axes.

## The buffer

`cerulion_tf::TfBuffer` is a plain field of the node. It holds a frame tree
with a bounded ring of stamped samples per edge, and static edges beside it.

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
and one `insert_static` per entry on `/tf_static`, with the stamp in
nanoseconds and the transform built through `UnitQuaternion::new` and
`Transform::new`. Those two constructors refuse a NaN, an infinite component
and a zero quaternion, so a malformed transform never reaches the buffer.

The rules:

- **One parent per frame.** A transform naming a child that already has a
  different parent is refused (`SecondParent`); so is a self edge and an edge
  that would make a frame its own ancestor (`Cycle`). The tree stays a tree.
- **Bounded rings.** Each dynamic edge keeps its samples sorted by stamp on a
  ring of fixed capacity. A sample at a stamp the ring already holds replaces
  it. When the ring is full, the oldest sample is evicted to make room, and a
  sample older than everything on a full ring is dropped on arrival. Both
  count in `evictions()`, every time. The defaults are 64 frames and 64
  samples per edge; `TfBuffer::with_capacity` sets both, and that constructor
  is the only place the buffer allocates. `insert` and `lookup` never touch
  the heap, so they belong inside a tick.
- **Static wins.** A static edge holds at every time. A dynamic transform
  for a static edge is refused (`EdgeIsStatic`), and a static insert on an
  edge with dynamic history clears that history.
- **Frame ids** are at most 36 bytes. Longer ids are refused
  (`FrameIdTooLong`); the empty id is refused (`EmptyFrameId`).
- **Capacity.** A transform naming a frame the buffer has no room for is
  refused (`FrameTableFull`), and names the frame.

Every refusal happens before the buffer changes: a refused insert leaves it
exactly as it was.

## Lookup

`lookup(parent, child, time)` returns `T_parent_child` at `time` (nanoseconds)
as a `Result<Transform, TfError>`. It walks from both frames up to their
nearest common ancestor and composes the edges it crosses, so it works
whether `child` descends from `parent`, `parent` descends from `child` (the
result is the inverse), or the two are siblings under a shared ancestor. On
each dynamic edge the sample at `time` is an exact hit, returned bit for bit,
or the interpolation between the two samples that bracket it: linear in
translation, shortest-arc SLERP in rotation.

`lookup_latest(parent, child)` resolves at the newest time every dynamic edge
on the chain has a sample for, through the same code path. A chain of static
edges only has no time and resolves directly.

**Lookups never extrapolate.** One nanosecond before an edge's oldest sample
or after its newest is an error, never a guessed transform. **Lookups never
block.** There is no listener thread and no timeout: an error is a fact about
the buffer at this fire, and the node asks again on its next fire.

```rust
match self.buffer.lookup("map", "lidar", stamp_ns) {
    Ok(map_lidar) => {
        let p_map = map_lidar.apply_point(p_lidar);
        // ...
    }
    Err(TfError::NotYetAvailable { .. }) => {
        // The transform for this stamp has not arrived; try next fire.
    }
    Err(err) => tracing::warn!(error = %err, "tf lookup refused"),
}
```

### The errors

Every variant of `TfError` names the frames involved in its text.

| Variant | When | What to do |
|---|---|---|
| `UnknownFrame { frame }` | No inserted transform has named `frame`. | Check the frame id; wait for the publisher. |
| `NoPath { parent, child }` | Both frames are known but no chain of edges joins them. | A broadcaster is missing an edge. |
| `NotYetAvailable { parent, child, requested_ns, newest_ns }` | `requested_ns` is newer than the newest sample on the edge `parent` to `child`. | Ask again on the next fire. |
| `OutOfRange { parent, child, requested_ns, oldest_ns, newest_ns }` | `requested_ns` is older than the oldest sample on that edge: evicted, or never buffered. | Waiting does not help; a larger ring or a fresher stamp does. |
| `Math(TfMathError)` | Composing the chain overflowed, or a constructor refused a value. | The input data is wrong. |
| `EmptyFrameId`, `FrameIdTooLong { len }` | The frame id is empty or over 36 bytes. | Fix the publisher's frame ids. |
| `SelfEdge { frame }` | Parent and child are the same frame. | Fix the publisher. |
| `SecondParent { child, parent, existing_parent }` | `child` already has a different parent. | Two broadcasters claim the same child; keep one. |
| `Cycle { parent, child }` | The edge would make a frame its own ancestor. | Fix the frame tree. |
| `EdgeIsStatic { parent, child }` | A dynamic transform arrived for a static edge. | Publish the edge on one topic only. |
| `FrameTableFull { frame, capacity }` | The buffer holds `capacity` frames and `frame` is new. | Build the buffer with a larger frame capacity. |

Errors are plain values with the frame ids inline: building one allocates
nothing, so a lookup that fails on every fire costs no more than one that
succeeds.

## Determinism

A lookup reads only recorded frames, and the math is a fixed sequence of
correctly rounded IEEE-754 operations with no platform math library and no
fused multiply-add. The walk over the tree is fixed by the tree, and the
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
- Extrapolation of any kind, and a blocking lookup with a timeout.
- Re-parenting a frame at runtime.

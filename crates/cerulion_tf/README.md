# cerulion_tf

Deterministic SE(3) math for [Cerulion](https://github.com/cerulion-inc/cerulion)
frames, and the tf buffer that answers `lookup(parent, child, time)`.

- A rigid `Transform` (translation plus unit quaternion) composes, inverts,
  applies to points and vectors, and interpolates (linear translation, SLERP
  rotation) between two stamps.
- A `TfBuffer` holds a frame tree with a stamp-sorted window of history per
  edge (10 s by default, whatever the publish rate) and static edges that
  never expire. `lookup` walks the chain between two frames through their
  common ancestor and interpolates on every dynamic edge; `lookup_latest`
  does the same at the newest time the whole chain answers.
  `static_edges` lists the static set, one edge per child frame, for a node
  that publishes it on `/tf_static`.
- Frame ids are interned into `FrameId` handles with no length limit of
  their own; `lookup_ids` skips the string work on every tick.
- Past an edge's newest sample, a per-edge `EdgePolicy` decides: nothing
  (the default), the newest sample held, or the last motion continued, each
  within a bound. `begin_epoch` starts a new timeline after a clock reset or
  a bag loop. Lookups never block: a time outside what an edge answers, an
  unknown frame or two frames in separate trees is a typed `TfError` that
  names the frames involved.
- Conventions follow REP-103 and `geometry_msgs/TransformStamped`: quaternions
  are `(x, y, z, w)`, a transform is `T_parent_child`, and
  `T_a_b.compose(&T_b_c)` is `T_a_c`.
- No allocation after the buffer is built, and no I/O.
- Bit-identical results on every machine: a fixed order of correctly rounded
  IEEE-754 operations, no fused multiply-add and no platform math library.
- A NaN, infinite or zero quaternion, a non-finite translation, a self edge,
  a second parent for a frame or an edge that would close a cycle is an
  error at insert, never a corrupted buffer.

The crate docs (`cargo doc -p cerulion_tf --open`) state every rule; the user
guide is `docs/transforms.md`.

## Gates

Two gates hold the crate to these rules. `tests/determinism_test.rs` pins
exact output bits, and `mutants-baseline.txt` records the
`cargo mutants -p cerulion_tf` result: a surviving mutant that is not listed
there with a reason is a gap in the tests.

## License

AGPL-3.0-only.

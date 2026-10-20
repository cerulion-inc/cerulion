# cerulion_tf

Deterministic SE(3) math for [Cerulion](https://github.com/cerulion-inc/cerulion)
frames: a rigid `Transform` (translation plus unit quaternion) that composes,
inverts and applies to points and vectors.

- Conventions follow REP-103 and `geometry_msgs/TransformStamped`: quaternions
  are `(x, y, z, w)`, a transform is `T_parent_child`, and
  `T_a_b.compose(&T_b_c)` is `T_a_c`.
- No allocation and no I/O.
- Bit-identical results on every machine: a fixed order of correctly rounded
  IEEE-754 operations, no fused multiply-add and no platform math library.
- A NaN, infinite or zero quaternion, or a non-finite translation, is an
  error, never a NaN result.

The crate docs (`cargo doc -p cerulion_tf --open`) state every rule.

## License

AGPL-3.0-only.

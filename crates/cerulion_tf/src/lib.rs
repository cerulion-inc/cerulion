// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic SE(3) math for Cerulion frames, and the tf buffer.
//!
//! A [`Transform`] is a rigid motion: a translation plus a unit-quaternion
//! rotation ([`UnitQuaternion`]). This crate composes, inverts and applies
//! transforms. A [`TfBuffer`] collects stamped transforms into a frame tree
//! and answers [`TfBuffer::lookup`] for any two frames at a time. Nothing
//! allocates after a buffer is built and nothing does I/O, and the same
//! inputs give bit-identical `f64` results on every machine, so frame
//! lookups replay byte-exact.
//!
//! # The buffer
//!
//! A node holds one [`TfBuffer`], feeds it every transform it receives
//! ([`TfBuffer::insert`] for `/tf`, [`TfBuffer::insert_static`] for
//! `/tf_static`) and reads it with [`TfBuffer::lookup`] or
//! [`TfBuffer::lookup_latest`]. The buffer keeps one parent per frame and a
//! bounded ring of samples per edge, refuses a malformed transform before
//! writing anything, and returns a [`TfError`] that names the frames
//! involved when a lookup cannot be answered. It never extrapolates and it
//! never blocks. The type docs state every rule.
//!
//! # Conventions
//!
//! Frames follow REP-103: right-handed, distances in metres, angles in
//! radians.
//!
//! - **Quaternion order** is `(x, y, z, w)`, the field order of
//!   `geometry_msgs/Quaternion`; `w` is the scalar part. Products follow the
//!   Hamilton convention and rotations are active: rotating `v` by `q` is
//!   `q v q*`.
//! - **A transform is `T_parent_child`.** Its translation is the child
//!   origin in parent coordinates and its rotation turns child axes into
//!   parent axes, the shape of `geometry_msgs/TransformStamped` with
//!   `header.frame_id` as the parent and `child_frame_id` as the child.
//!   [`Transform::apply_point`] maps child coordinates to parent coordinates:
//!   `p_parent = R p_child + t`.
//! - **Compose reads left to right along the frame chain:**
//!   `T_a_b.compose(&T_b_c)` is `T_a_c`. [`Transform::inverse`] turns
//!   `T_a_b` into `T_b_a`.
//! - **Interpolation** between two stamped transforms of the same pair,
//!   [`Transform::interpolate`] with `t` in `[0, 1]`, is linear in
//!   translation and SLERP on the shortest arc in rotation. A `t` outside
//!   `[0, 1]` is an error: it never extrapolates.
//!
//! # Normalization and sign
//!
//! [`UnitQuaternion::new`] normalizes its input, and every operation that
//! returns a quaternion renormalizes its result, under one rule: divide the
//! components by the largest component magnitude, then by the norm of the
//! scaled vector. The scaling means no square overflows or underflows. A
//! quaternion with a NaN or infinite component, or a norm below
//! [`UnitQuaternion::MIN_NORM`], is an error, never a NaN result.
//!
//! The sign of a quaternion is never changed by construction or by a
//! product: `q` and `-q` name the same rotation and this crate keeps the one
//! the arithmetic produced. Interpolation is the one place a sign matters:
//! it negates the target when that makes the arc shorter (see
//! [`UnitQuaternion::slerp`]).
//!
//! # Determinism
//!
//! Every result is a fixed sequence of IEEE-754 binary64 additions,
//! subtractions, multiplications, divisions and square roots, each correctly
//! rounded, in an order written out in the source. There is no fused
//! multiply-add (Rust never fuses `a * b + c` on its own) and no
//! platform math library call: the sine and arctangent SLERP needs are
//! fixed polynomials in this crate. The same inputs therefore give the same
//! bits on every target with IEEE-754 binary64 arithmetic.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod buffer;
mod error;
mod frame_name;
mod kernels;
mod quaternion;
mod transform;

pub use buffer::{EdgeInfo, TfBuffer, DEFAULT_FRAME_CAPACITY, DEFAULT_SAMPLES_PER_EDGE};
pub use error::TfError;
pub use frame_name::{FrameName, MAX_FRAME_ID_LEN};
pub use quaternion::UnitQuaternion;
pub use transform::Transform;

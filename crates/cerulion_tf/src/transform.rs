// SPDX-License-Identifier: AGPL-3.0-only
//! Rigid transforms: compose, invert and apply.

use crate::quaternion::check_fraction;
use crate::{TfMathError, UnitQuaternion};

/// A rigid transform `T_parent_child`: where the child frame sits in the
/// parent frame.
///
/// `translation` is the child origin in parent coordinates, in metres.
/// `rotation` turns child axes into parent axes. This is the shape of
/// `geometry_msgs/TransformStamped`, with `header.frame_id` the parent and
/// `child_frame_id` the child.
///
/// Rotation components are `(x, y, z, w)`, composed by the Hamilton
/// product. `T_a_b.compose(&T_b_c)` is `T_a_c`; `inverse` turns `T_a_b`
/// into `T_b_a`.
///
/// The fields are private, so every value has a finite translation and a
/// unit rotation. See the crate docs for the conventions and the
/// determinism rules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    translation: [f64; 3],
    rotation: UnitQuaternion,
}

impl Transform {
    /// The child frame is the parent frame.
    pub const IDENTITY: Self = Self {
        translation: [0.0, 0.0, 0.0],
        rotation: UnitQuaternion::IDENTITY,
    };

    /// Builds `T_parent_child` from the child origin in parent coordinates
    /// and the child orientation in the parent.
    ///
    /// # Errors
    ///
    /// [`TfMathError::NonFiniteTranslation`] when a translation component is
    /// NaN or infinite.
    pub fn new(translation: [f64; 3], rotation: UnitQuaternion) -> Result<Self, TfMathError> {
        check_finite(translation)?;
        Ok(Self {
            translation,
            rotation,
        })
    }

    /// The child origin in parent coordinates.
    pub fn translation(&self) -> [f64; 3] {
        self.translation
    }

    /// The child orientation in the parent.
    pub fn rotation(&self) -> UnitQuaternion {
        self.rotation
    }

    /// Maps a point from child coordinates to parent coordinates:
    /// `R p + t`.
    ///
    /// The point is not checked: a non-finite point gives a non-finite
    /// result, and so can a point within a factor of about 3 of `f64::MAX`.
    pub fn apply_point(&self, p: [f64; 3]) -> [f64; 3] {
        let r = self.rotation.rotate(p);
        [
            r[0] + self.translation[0],
            r[1] + self.translation[1],
            r[2] + self.translation[2],
        ]
    }

    /// Maps a direction (a free vector) from child to parent coordinates:
    /// `R v`. Translation does not apply to a direction. Unchecked, like
    /// [`Self::apply_point`].
    pub fn apply_vector(&self, v: [f64; 3]) -> [f64; 3] {
        self.rotation.rotate(v)
    }

    /// Chains two transforms: `T_a_b.compose(&T_b_c)` is `T_a_c`.
    ///
    /// The rotation is `R_a_b * R_b_c` (renormalized); the translation is
    /// `R_a_b t_b_c + t_a_b`. `compose(x).apply_point(p)` equals
    /// `apply_point(x.apply_point(p))` up to rounding.
    ///
    /// # Errors
    ///
    /// [`TfMathError::NonFiniteTranslation`] when the translation overflows.
    pub fn compose(&self, rhs: &Self) -> Result<Self, TfMathError> {
        let translation = self.apply_point(rhs.translation);
        check_finite(translation)?;
        Ok(Self {
            translation,
            rotation: self.rotation.compose(&rhs.rotation),
        })
    }

    /// The reverse transform: `T_a_b.inverse()` is `T_b_a`.
    ///
    /// The rotation is the conjugate (exact); the translation is
    /// `-(R^-1 t)`.
    ///
    /// # Errors
    ///
    /// [`TfMathError::NonFiniteTranslation`] when the translation overflows.
    pub fn inverse(&self) -> Result<Self, TfMathError> {
        let rotation = self.rotation.inverse();
        let r = rotation.rotate(self.translation);
        let translation = [-r[0], -r[1], -r[2]];
        check_finite(translation)?;
        Ok(Self {
            translation,
            rotation,
        })
    }

    /// Interpolates from `self` (at `t = 0`) to `other` (at `t = 1`):
    /// translation linearly as `(1 - t) a + t b`, rotation by
    /// [`UnitQuaternion::slerp`] on the shortest arc.
    ///
    /// `t == 0` returns `self` and `t == 1` returns `other`, bit for bit.
    /// A translation component that is equal at both ends is returned as
    /// is, so a frame that does not move interpolates to itself bit for bit
    /// (the blend `(1 - t) a + t a` can round 1 ulp away from `a`). Other
    /// components stay within rounding of the segment between their
    /// endpoints, so two finite translations do not overflow when blended.
    ///
    /// # Errors
    ///
    /// [`TfMathError::FractionNotANumber`] when `t` is NaN;
    /// [`TfMathError::FractionOutOfRange`] when `t` is outside `[0, 1]`.
    pub fn interpolate(&self, other: &Self, t: f64) -> Result<Self, TfMathError> {
        check_fraction(t)?;
        if t == 0.0 {
            return Ok(*self);
        }
        if t == 1.0 {
            return Ok(*other);
        }
        let (a, b) = (self.translation, other.translation);
        let u = 1.0 - t;
        // Equal components short-circuit: the blend can round 1 ulp off.
        let blend = |from: f64, to: f64| if from == to { from } else { u * from + t * to };
        Ok(Self {
            translation: [blend(a[0], b[0]), blend(a[1], b[1]), blend(a[2], b[2])],
            rotation: self.rotation.slerp(&other.rotation, t)?,
        })
    }
}

fn check_finite(t: [f64; 3]) -> Result<(), TfMathError> {
    if t[0].is_finite() && t[1].is_finite() && t[2].is_finite() {
        Ok(())
    } else {
        Err(TfMathError::NonFiniteTranslation)
    }
}

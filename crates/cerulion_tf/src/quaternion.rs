// SPDX-License-Identifier: AGPL-3.0-only
//! Unit quaternions: construction, product, conjugate and vector rotation.

use crate::TfMathError;

/// A rotation as a unit quaternion, components in ROS order `(x, y, z, w)`.
///
/// The fields are private, so every value is unit length: [`Self::new`]
/// normalizes its input, and every operation that returns a quaternion
/// renormalizes its result under the rule in the crate docs.
///
/// Hamilton convention: `w` is the scalar part, `i * j = k`, and
/// [`Self::rotate`] applies the active rotation `q v q*`. The sign is never
/// changed: `q` and `-q` name the same rotation, and this type keeps
/// whichever one the arithmetic produced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnitQuaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl UnitQuaternion {
    /// No rotation: `(0, 0, 0, 1)`.
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    /// The smallest norm [`Self::new`] accepts. A shorter quaternion is
    /// almost certainly an uninitialised or corrupted value, not a rotation,
    /// so it is refused rather than normalized.
    pub const MIN_NORM: f64 = 1e-12;

    /// Builds a unit quaternion from `(x, y, z, w)`, normalizing it.
    ///
    /// # Errors
    ///
    /// [`TfMathError::NonFiniteQuaternion`] when a component is NaN or
    /// infinite; [`TfMathError::DegenerateQuaternion`] when the norm is
    /// below [`Self::MIN_NORM`].
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Result<Self, TfMathError> {
        if !(x.is_finite() && y.is_finite() && z.is_finite() && w.is_finite()) {
            return Err(TfMathError::NonFiniteQuaternion);
        }
        let m = max_abs(x, y, z, w);
        if m == 0.0 {
            return Err(TfMathError::DegenerateQuaternion { norm: 0.0 });
        }
        let (unit, scaled_norm) = normalize_scaled(x, y, z, w, m);
        let norm = m * scaled_norm;
        if norm < Self::MIN_NORM {
            return Err(TfMathError::DegenerateQuaternion { norm });
        }
        Ok(unit)
    }

    /// Builds a unit quaternion from `[x, y, z, w]`; see [`Self::new`].
    ///
    /// # Errors
    ///
    /// The same as [`Self::new`].
    pub fn from_array(c: [f64; 4]) -> Result<Self, TfMathError> {
        Self::new(c[0], c[1], c[2], c[3])
    }

    /// The `x` component.
    pub fn x(&self) -> f64 {
        self.x
    }

    /// The `y` component.
    pub fn y(&self) -> f64 {
        self.y
    }

    /// The `z` component.
    pub fn z(&self) -> f64 {
        self.z
    }

    /// The `w` (scalar) component.
    pub fn w(&self) -> f64 {
        self.w
    }

    /// The components as `[x, y, z, w]`.
    pub fn to_array(&self) -> [f64; 4] {
        [self.x, self.y, self.z, self.w]
    }

    /// The inverse rotation: the conjugate `(-x, -y, -z, w)`. Exact, so no
    /// renormalization.
    pub fn inverse(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    /// The Hamilton product `self * rhs`: rotate by `rhs` first, then by
    /// `self`. Renormalized.
    pub fn compose(&self, rhs: &Self) -> Self {
        let (a, b) = (self, rhs);
        let x = ((a.w * b.x + a.x * b.w) + a.y * b.z) - a.z * b.y;
        let y = ((a.w * b.y - a.x * b.z) + a.y * b.w) + a.z * b.x;
        let z = ((a.w * b.z + a.x * b.y) - a.y * b.x) + a.z * b.w;
        let w = ((a.w * b.w - a.x * b.x) - a.y * b.y) - a.z * b.z;
        Self::renormalize(x, y, z, w)
    }

    /// Rotates the vector `v` by this rotation (`q v q*`).
    ///
    /// Computed as `v + w t + u x t` with `u = (x, y, z)` and
    /// `t = 2 (u x v)`, in that fixed order. Finite input gives finite
    /// output unless a component is within a factor of about 3 of
    /// `f64::MAX`.
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let u = [self.x, self.y, self.z];
        let c = cross(u, v);
        let t = [2.0 * c[0], 2.0 * c[1], 2.0 * c[2]];
        let ut = cross(u, t);
        [
            (v[0] + self.w * t[0]) + ut[0],
            (v[1] + self.w * t[1]) + ut[1],
            (v[2] + self.w * t[2]) + ut[2],
        ]
    }

    /// Renormalizes components that are already close to unit length (the
    /// result of a product or a blend of unit quaternions). Such a value has
    /// a largest component of at least about 0.5 and no non-finite part, so
    /// it cannot fail.
    pub(crate) fn renormalize(x: f64, y: f64, z: f64, w: f64) -> Self {
        normalize_scaled(x, y, z, w, max_abs(x, y, z, w)).0
    }
}

/// The largest component magnitude.
fn max_abs(x: f64, y: f64, z: f64, w: f64) -> f64 {
    x.abs().max(y.abs()).max(z.abs()).max(w.abs())
}

/// THE normalization rule. Divide every component by the largest magnitude
/// `m` (so no square can overflow or underflow), then by the norm of the
/// scaled vector, summed in the fixed order `((x² + y²) + z²) + w²`.
/// Returns the unit quaternion and the scaled norm (the true norm is
/// `m * scaled_norm`). `m` must be nonzero.
fn normalize_scaled(x: f64, y: f64, z: f64, w: f64, m: f64) -> (UnitQuaternion, f64) {
    let (sx, sy, sz, sw) = (x / m, y / m, z / m, w / m);
    let n = (((sx * sx + sy * sy) + sz * sz) + sw * sw).sqrt();
    (
        UnitQuaternion {
            x: sx / n,
            y: sy / n,
            z: sz / n,
            w: sw / n,
        },
        n,
    )
}

/// The cross product `a x b`.
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

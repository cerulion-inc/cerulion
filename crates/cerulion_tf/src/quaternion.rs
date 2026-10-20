// SPDX-License-Identifier: AGPL-3.0-only
//! Unit quaternions: construction, product, conjugate and vector rotation.

use crate::kernels::{atan_unit, sin_quarter_turn};
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

    /// Below this 4-D angle (radians) between two quaternions, [`Self::slerp`]
    /// blends with the normalized linear weights `(1 - t, t)` instead of the
    /// sine weights, whose `sin(theta)` divisor reaches zero for equal
    /// inputs. The 4-D angle is half the rotation angle, so this is a
    /// rotation difference of 2e-6 rad. There the two blends differ by order
    /// `theta^3`, below 1e-17 absolute: under an ulp of every component near
    /// 1, but not a bit-exact match on the small components.
    pub const SLERP_LERP_THRESHOLD: f64 = 1e-6;

    /// Spherical linear interpolation from `self` (at `t = 0`) to `other`
    /// (at `t = 1`) on the shortest arc.
    ///
    /// The rules, in order:
    ///
    /// 1. `t` must be in `[0, 1]`; interpolation never extrapolates.
    /// 2. `t == 0` returns `self` and `t == 1` returns `other`, bit for bit
    ///    and with their signs as given.
    /// 3. Shortest arc: when `|b - a| > |b + a|` (the same test as a
    ///    negative 4-D dot product), `other` is negated first (same rotation,
    ///    the nearer sign). Equal norms (a 180 degree difference, both arcs
    ///    equally long) keep `other` as given.
    /// 4. The 4-D angle is `theta = 2 atan(|b - a| / |b + a|)`, accurate even
    ///    for nearly equal inputs. Below [`Self::SLERP_LERP_THRESHOLD`] the
    ///    weights are `(1 - t, t)`; otherwise they are
    ///    `sin((1 - t) theta) / sin(theta)` and `sin(t theta) / sin(theta)`.
    /// 5. The blend is renormalized.
    ///
    /// # Errors
    ///
    /// [`TfMathError::FractionNotANumber`] when `t` is NaN;
    /// [`TfMathError::FractionOutOfRange`] when `t` is outside `[0, 1]`.
    pub fn slerp(&self, other: &Self, t: f64) -> Result<Self, TfMathError> {
        check_fraction(t)?;
        if t == 0.0 {
            return Ok(*self);
        }
        if t == 1.0 {
            return Ok(*other);
        }
        let a = self.to_array();
        let mut b = other.to_array();
        let mut diff = norm4([b[0] - a[0], b[1] - a[1], b[2] - a[2], b[3] - a[3]]);
        let mut sum = norm4([b[0] + a[0], b[1] + a[1], b[2] + a[2], b[3] + a[3]]);
        // |b - a| > |b + a| exactly when a . b < 0: -b is the nearer sign.
        // Negating b swaps the two norms, so no recomputation.
        if diff > sum {
            b = [-b[0], -b[1], -b[2], -b[3]];
            std::mem::swap(&mut diff, &mut sum);
        }
        let theta = 2.0 * atan_unit(diff / sum);
        let (wa, wb) = if theta < Self::SLERP_LERP_THRESHOLD {
            (1.0 - t, t)
        } else {
            let s = sin_quarter_turn(theta);
            (
                sin_quarter_turn((1.0 - t) * theta) / s,
                sin_quarter_turn(t * theta) / s,
            )
        };
        Ok(Self::renormalize(
            wa * a[0] + wb * b[0],
            wa * a[1] + wb * b[1],
            wa * a[2] + wb * b[2],
            wa * a[3] + wb * b[3],
        ))
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

/// Refuses a fraction outside `[0, 1]`. Shared by every interpolation.
pub(crate) fn check_fraction(t: f64) -> Result<(), TfMathError> {
    if t.is_nan() {
        return Err(TfMathError::FractionNotANumber);
    }
    if !(0.0..=1.0).contains(&t) {
        return Err(TfMathError::FractionOutOfRange { t });
    }
    Ok(())
}

/// The Euclidean norm of a 4-vector of magnitude at most about 2, summed in
/// a fixed order.
fn norm4(v: [f64; 4]) -> f64 {
    (((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]) + v[3] * v[3]).sqrt()
}

/// The cross product `a x b`.
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The error type of the SE(3) math. The tf buffer has its own,
//! [`crate::TfError`], which wraps this one.

/// Why an SE(3) operation refused its input.
///
/// Every operation that builds a [`crate::Transform`] or a
/// [`crate::UnitQuaternion`] checks its result, so a value of either type is
/// always finite and its rotation always unit length. A bad input is an
/// error here, never a NaN in the output.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum TfMathError {
    /// A quaternion component is NaN or infinite.
    #[error("quaternion has a non-finite component (NaN or infinity)")]
    NonFiniteQuaternion,
    /// The quaternion is too short to name a rotation: its norm is below
    /// [`crate::UnitQuaternion::MIN_NORM`]. The zero quaternion reports a
    /// norm of `0.0`.
    #[error("quaternion norm {norm:e} is below the minimum of 1e-12, so it names no rotation")]
    DegenerateQuaternion {
        /// The norm of the refused quaternion.
        norm: f64,
    },
    /// A translation component is NaN or infinite, in the input or in the
    /// result of a compose or inverse that overflowed. Interpolation never
    /// returns it: its linear blend stays within rounding of the segment
    /// between two finite endpoints.
    #[error("translation has a non-finite component (NaN or infinity)")]
    NonFiniteTranslation,
    /// The interpolation fraction is NaN.
    #[error("interpolation fraction is NaN")]
    FractionNotANumber,
    /// The interpolation fraction is outside `[0, 1]`. Interpolation never
    /// extrapolates.
    #[error("interpolation fraction {t} is outside [0, 1]; interpolation never extrapolates")]
    FractionOutOfRange {
        /// The refused fraction.
        t: f64,
    },
}

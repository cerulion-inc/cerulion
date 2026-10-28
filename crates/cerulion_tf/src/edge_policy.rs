// SPDX-License-Identifier: AGPL-3.0-only
//! What a dynamic edge answers for a time past its newest sample.

use crate::{TfError, Transform};

/// What a dynamic edge answers for a time past its newest sample; set per
/// edge with [`crate::TfBuffer::set_edge_policy`].
///
/// A time between two samples is always an interpolation and a time before
/// the oldest is always [`TfError::OutOfRange`]; the policy only decides the
/// time after the newest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgePolicy {
    /// No answer past the newest sample: [`TfError::NotYetAvailable`]. The
    /// default, and the tf2 behavior.
    #[default]
    Interpolate,
    /// The newest sample holds for up to `max_age_ns` past its stamp, for a
    /// slow edge such as `map` to `odom` that a fast chain reads between
    /// two of its samples.
    HoldLast {
        /// How long the newest sample holds, in nanoseconds.
        max_age_ns: u64,
    },
    /// The motion between the two newest samples continues for up to
    /// `max_ns` past the newest stamp, for an edge that publishes behind
    /// the data it is used with. One sample alone gives no motion, so an
    /// edge with one sample answers nothing past it.
    Extrapolate {
        /// How far past the newest stamp the motion continues, in
        /// nanoseconds.
        max_ns: u64,
    },
}

impl EdgePolicy {
    /// How far past the newest sample this policy answers, in nanoseconds.
    pub(crate) fn reach_ns(self) -> u64 {
        match self {
            Self::Interpolate => 0,
            Self::HoldLast { max_age_ns } => max_age_ns,
            Self::Extrapolate { max_ns } => max_ns,
        }
    }
}

/// `T_parent_child` at `ahead_ns` past `after`, continuing the motion from
/// `before` to `after`, which are `span_ns` apart.
///
/// The step `delta = before^-1 * after` is the motion over one span, in the
/// child frame of `before`. The answer is `after * delta^f` with
/// `f = ahead_ns / span_ns`: the whole steps of `f` are composed by repeated
/// squaring and its fraction is the interpolation from the identity to
/// `delta` ([`Transform::interpolate`]). Every operation is the
/// deterministic math of this crate, so the result replays bit for bit.
///
/// # Errors
///
/// [`TfError::NonFiniteTranslation`] when the composed translation
/// overflows.
pub(crate) fn extrapolate(
    before: &Transform,
    after: &Transform,
    span_ns: u64,
    ahead_ns: u64,
) -> Result<Transform, TfError> {
    let delta = before.inverse()?.compose(after)?;
    let whole = ahead_ns / span_ns;
    // The remainder is below the span, so the fraction is in [0, 1). Both
    // convert to f64 exactly below 2^53 ns (about 104 days); a wider span
    // rounds, the same way on every run.
    let fraction = (ahead_ns % span_ns) as f64 / span_ns as f64;
    let mut acc = *after;
    let mut base = delta;
    let mut k = whole;
    while k > 0 {
        if k & 1 == 1 {
            acc = acc.compose(&base)?;
        }
        k >>= 1;
        if k > 0 {
            base = base.compose(&base)?;
        }
    }
    if fraction > 0.0 {
        acc = acc.compose(&Transform::IDENTITY.interpolate(&delta, fraction)?)?;
    }
    Ok(acc)
}

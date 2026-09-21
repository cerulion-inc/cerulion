// SPDX-License-Identifier: AGPL-3.0-only
//! The env-gated, per-stage LATENCY PROBE shared by the desk daemons.
//!
//! Live use showed the Studio video feed lagging after both the live
//! queue fix and local OpenH264 decode were verified active. Nothing in the desk
//! chain is instrumented, so "where do the milliseconds go" is currently a guess.
//! This module is the ONE gate + the ONE join clock behind which each daemon emits
//! a stage stamp, so the residual can be ATTRIBUTED instead of argued about.
//!
//! # What can and cannot be measured
//!
//! **Not measurable: robot → desk.** A frame's wire `timestamp_ns` is stamped by
//! the PRODUCER's clock, and a `graph run-worker`'s gating clock advances by a fixed
//! logical quantum — its seconds are not wall seconds and its epoch is not
//! ours. Subtracting a desk wall clock from a wire stamp compares two unrelated
//! number lines (the lesson, learned the expensive way). This module
//! therefore NEVER touches wire stamps for timing; it uses them only as the frame
//! IDENTITY (`sequence`).
//!
//! **Not measurable: viewer playback.** vizd hands a frame to the rerun sink; what
//! the viewer does with it afterwards is inside the viewer. The probe brackets that
//! stage — it reports when vizd finished handing off, and the remainder up to glass
//! is whatever the operator observes minus that. Stated plainly rather than
//! silently attributed to the last stage we can see.
//!
//! **Measurable, and what this ships:** every stage BETWEEN those two, on one desk.
//!
//! # How the stages join across processes
//!
//! Stage stamps live in two processes (`cerulion-netd` and `cerulion-vizd`), so they
//! cannot share an `Instant`. They share the machine's `CLOCK_REALTIME` instead
//! ([`wall_ns`]) — legitimate precisely because it is ONE clock on ONE machine, and
//! ms-scale attribution does not care about its jitter.
//!
//! Correlation is by the wire `sequence`, and the sampling decision is a pure
//! function OF that sequence ([`should_sample`]) rather than a per-process counter.
//! That is the load-bearing design choice: two processes sampling independently with
//! counters would pick DIFFERENT frames and nothing would ever join. Keyed on the
//! sequence, both daemons stamp the same frames without exchanging a word.
//!
//! # Cost when disabled
//!
//! One `OnceLock` load and a branch — no env read, no clock read, no allocation, no
//! log line. The gate is checked before the sequence is even needed at the sites
//! that would otherwise have to parse a header for it.

use std::num::NonZeroU32;
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};

/// The env var enabling the probe. Value is a SAMPLING STRIDE: `1` stamps every
/// frame, `N` stamps one frame in `N` (by wire sequence). Unset / `off` / `0`
/// disables it. Anything else is a loud warn + disabled.
pub const LAT_PROBE_ENV: &str = "CERULION_H264_LAT_PROBE";

/// The parsed meaning of [`LAT_PROBE_ENV`]. Pure, so the parse is oracle-testable
/// without touching the process environment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeSpec {
    /// No probe: the daemons behave exactly as if this module did not exist.
    Off,
    /// Stamp one frame in `stride` (`stride == 1` ⇒ every frame).
    Sample {
        /// How many frames one stage line covers — a frame is sampled iff its
        /// wire `sequence` is a multiple of this.
        stride: NonZeroU32,
    },
    /// The value was set but unusable — disabled, and the caller warns LOUDLY
    /// naming the offending value (a silent fallback would leave an operator
    /// staring at an empty log convinced the probe is on).
    Invalid,
}

/// Classify a raw [`LAT_PROBE_ENV`] value. `None` = unset.
///
/// Accepted: `off` (any case) and `0` ⇒ [`ProbeSpec::Off`]; a positive decimal
/// ⇒ [`ProbeSpec::Sample`]. Everything else ⇒ [`ProbeSpec::Invalid`] — including an
/// empty string, which is what a shell `VAR=` leaves behind and which must not be
/// mistaken for "off" silently.
pub fn classify_probe_spec(raw: Option<&str>) -> ProbeSpec {
    let Some(raw) = raw else {
        return ProbeSpec::Off;
    };
    let trimmed = raw.trim();
    if trimmed.eq_ignore_ascii_case("off") || trimmed == "0" {
        return ProbeSpec::Off;
    }
    match trimmed.parse::<u32>() {
        Ok(n) => match NonZeroU32::new(n) {
            Some(stride) => ProbeSpec::Sample { stride },
            // `0` is handled above; this arm is unreachable in practice and is a
            // fail-closed guard rather than a claim about reachability.
            None => ProbeSpec::Off,
        },
        Err(_) => ProbeSpec::Invalid,
    }
}

/// The process-wide probe stride, resolved ONCE from the environment.
///
/// Cached because the per-frame sites are on the video path at 30 Hz+ per stream:
/// an `env::var` per frame would itself be a latency the probe then measured.
fn stride() -> Option<NonZeroU32> {
    static STRIDE: OnceLock<Option<NonZeroU32>> = OnceLock::new();
    *STRIDE.get_or_init(|| match classify_probe_spec(std::env::var(LAT_PROBE_ENV).ok().as_deref()) {
        ProbeSpec::Sample { stride } => {
            tracing::info!(
                env = LAT_PROBE_ENV,
                stride = stride.get(),
                "latency probe ENABLED — one stage line per `stride` frame(s), keyed by wire sequence"
            );
            Some(stride)
        }
        ProbeSpec::Off => None,
        ProbeSpec::Invalid => {
            tracing::warn!(
                env = LAT_PROBE_ENV,
                value = %std::env::var(LAT_PROBE_ENV).unwrap_or_default(),
                "latency probe DISABLED — the value is not `off`, `0`, or a positive sampling stride"
            );
            None
        }
    })
}

/// Whether the probe is on at all. Check this BEFORE doing any work a stage stamp
/// needs (a header re-parse, a clock read) so a disabled probe costs one load.
#[inline]
pub fn probe_enabled() -> bool {
    stride().is_some()
}

/// Whether this frame is one of the sampled ones, decided PURELY from its wire
/// `sequence` so every process picks the same frames with no coordination.
#[inline]
pub fn should_sample(sequence: u32) -> bool {
    match stride() {
        Some(stride) => sequence.is_multiple_of(stride.get()),
        None => false,
    }
}

/// The cross-process join clock: nanoseconds since the Unix epoch on THIS machine's
/// `CLOCK_REALTIME`.
///
/// This is the only quantity two daemons may legitimately compare here, and it is
/// legitimate only because both readings come from one machine's one system clock.
/// It must NEVER be compared against a frame's wire `timestamp_ns` — see the module
/// docs. A clock that cannot be read yields 0, which reads as an obviously-broken
/// stamp rather than a plausible wrong one.
#[inline]
pub fn wall_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Oracle vectors for the env classifier — hand-written, never a self-compare.
    // The gate is the whole safety story of this module (an accidentally-on probe
    // logs per frame on a 30 Hz video stream), so its parse is pinned exactly.

    #[test]
    fn an_unset_or_explicitly_off_value_disables_the_probe() {
        assert_eq!(classify_probe_spec(None), ProbeSpec::Off, "unset ⇒ off");
        assert_eq!(classify_probe_spec(Some("off")), ProbeSpec::Off);
        assert_eq!(
            classify_probe_spec(Some("OFF")),
            ProbeSpec::Off,
            "case-insensitive, like the other cerulion switches"
        );
        assert_eq!(
            classify_probe_spec(Some(" off ")),
            ProbeSpec::Off,
            "surrounding whitespace is a shell artefact, not an opinion"
        );
        assert_eq!(
            classify_probe_spec(Some("0")),
            ProbeSpec::Off,
            "a zero stride is off, not a division by zero"
        );
    }

    #[test]
    fn a_positive_stride_enables_sampling_at_exactly_that_stride() {
        for n in [1u32, 2, 30, 1000, u32::MAX] {
            assert_eq!(
                classify_probe_spec(Some(&n.to_string())),
                ProbeSpec::Sample {
                    stride: NonZeroU32::new(n).expect("n > 0")
                },
                "stride {n} must round-trip exactly"
            );
        }
    }

    #[test]
    fn an_unusable_value_is_invalid_and_never_silently_off() {
        // The distinction matters: `Invalid` makes the caller WARN. An operator who
        // typed the value expects lines; silence with no explanation is the worst
        // possible outcome for a diagnostic.
        for bad in [
            "", " ", "yes", "on", "true", "-1", "1.5", "1e3", "0x10", "30ms",
        ] {
            assert_eq!(
                classify_probe_spec(Some(bad)),
                ProbeSpec::Invalid,
                "{bad:?} is not a stride and must be reported, not ignored"
            );
        }
    }

    #[test]
    fn a_stride_selects_an_exact_predictable_multiple_set() {
        // THE join property: two processes must select the SAME frames with no
        // coordination, so the decision may depend on nothing but the sequence.
        //
        // SCOPE: this models the rule against the stride directly and does
        // NOT call `should_sample`, whose cached env read is process-global and
        // cannot be driven from a unit test — so this arm alone does not catch a
        // bug in the production function. The real pin is
        // `lat_probe_env_test::a_stride_selects_exactly_the_multiples_and_nothing_else`,
        // which drives `should_sample` in a SUBPROCESS against the hand oracle
        // `enabled=true selected=0,30,60,90`. What this arm buys is a readable,
        // in-module statement of the arithmetic that pin depends on.
        let pick = |seq: u32, stride: u32| seq.is_multiple_of(stride);

        // stride 1 ⇒ every frame, so a joined trace loses nothing.
        for seq in 0..50u32 {
            assert!(pick(seq, 1), "stride 1 must sample seq {seq}");
        }
        // stride 30 on a 30 Hz stream ⇒ ~1 line/second per stage.
        let sampled: Vec<u32> = (0..91u32).filter(|s| pick(*s, 30)).collect();
        assert_eq!(
            sampled,
            vec![0, 30, 60, 90],
            "a stride must select an exact, predictable set — this list IS what \
             the two daemons independently agree on"
        );
        // And the agreement survives a u32 sequence wrap: the rule is arithmetic on
        // the sequence alone, so a wrapped stream keeps sampling (the boundary is
        // simply not a multiple — an irregular gap, never a silent stop).
        assert!(pick(u32::MAX / 30 * 30, 30), "a pre-wrap multiple samples");
        assert!(pick(0, 30), "and the post-wrap restart samples immediately");
    }

    #[test]
    fn the_join_clock_is_a_real_advancing_wall_clock() {
        // The cross-process join rests on this being CLOCK_REALTIME ns, so pin both
        // that it is populated (not the 0 fallback) and that it advances.
        let a = wall_ns();
        assert!(
            a > 1_700_000_000_000_000_000,
            "wall_ns must be ns since the Unix epoch (got {a}) — a small value means \
             the clock read failed and every joined delta would be nonsense"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
        let b = wall_ns();
        assert!(b > a, "the join clock must advance ({a} → {b})");
    }
}

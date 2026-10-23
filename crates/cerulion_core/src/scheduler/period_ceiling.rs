// SPDX-License-Identifier: AGPL-3.0-only
//! The run-wide CEILING on `Period` fires per step, and the per-step bounds a
//! `Period` decision runs under.
//!
//! # Why a ceiling exists next to `max_catchup`
//!
//! `max_catchup` and the state-arm clamp ([`super::catchup_clamp`]) bound a
//! catch-up BURST: a node that fell behind fires up to the cap of its missed
//! deadlines in one step, oldest first. That is the right rule for a clock that
//! moves with the loop, where a missed deadline is a fire that is owed.
//!
//! A sim-time run steps the graph once per `/clock` sample, and a sample may
//! jump the clock across several periods. There the contract is "at most one
//! fire per accepted sample": the jump is the simulator's statement of where
//! time is now, not a backlog the node owes. The ceiling is that bound. It
//! applies to EVERY `Period` node, a declared `max_catchup` included, because
//! it is a property of the clock and not a node's burst policy.
//!
//! # The fires a ceiling keeps are the LATEST deadlines
//!
//! A capped burst keeps the earliest missed deadlines and drops the rest. A
//! ceiling keeps the latest ones: a 10 ms node whose clock jumps from 1.000 s
//! to 1.100 s fires once, at the 1.100 s deadline, rather than at 1.010 s with
//! the clock already ninety milliseconds past it. Either way the node's next
//! deadline lands one period after the last deadline the jump passed, exactly
//! where [`catchup_clamp::period_advance`] puts it.

use super::catchup_clamp::{self, PeriodAdvance};

/// The bounds one step's `Period` decisions run under, read once per step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PeriodCaps {
    /// The state-arm clamp: replaces an UNDECLARED `max_catchup` only (see
    /// [`catchup_clamp::effective_max_catchup`]).
    pub override_cap: Option<u32>,
    /// The run-wide ceiling: bounds every `Period` node's fires per step,
    /// declared or not. `None` on every run except sim time.
    pub ceiling: Option<u32>,
}

impl PeriodCaps {
    /// Advance one `Period` node whose own declaration is `declared`. With no
    /// ceiling this is [`catchup_clamp::period_advance`] under the effective
    /// cap, unchanged. With one, see [`ceiled_period_advance`].
    #[must_use]
    pub fn advance(
        self,
        declared: Option<u32>,
        next_fire_ns: u64,
        now_ns: u64,
        interval_ns: u64,
    ) -> PeriodAdvance {
        let cap = catchup_clamp::effective_max_catchup(declared, self.override_cap);
        match self.ceiling {
            None => catchup_clamp::period_advance(next_fire_ns, now_ns, interval_ns, cap),
            Some(ceiling) => {
                ceiled_period_advance(next_fire_ns, now_ns, interval_ns, cap.min(ceiling))
            }
        }
    }
}

/// PURE: [`catchup_clamp::period_advance`] with the fires it keeps moved to the
/// LATEST `cap` deadlines the advance passes. The fire count and the deadline
/// after the step are the same as the capped advance's; only the first fire's
/// instant differs, and only when the cap dropped deadlines.
#[must_use]
pub fn ceiled_period_advance(
    next_fire_ns: u64,
    now_ns: u64,
    interval_ns: u64,
    cap: u32,
) -> PeriodAdvance {
    let adv = catchup_clamp::period_advance(next_fire_ns, now_ns, interval_ns, cap);
    if adv.fire_count == 0 {
        return adv;
    }
    // The kept fires are the last `fire_count` deadlines the advance passed.
    // The latest one comes from the unsaturated inputs, never from
    // `adv.next_fire_ns`: that one saturates at `u64::MAX` when the next
    // deadline lies past it. `fire_count > 0` means `next_fire_ns <= now_ns`
    // and `interval_ns > 0`, so `passed * interval_ns <= behind` and the latest
    // deadline is at most `now_ns`: no step here can overflow or underflow.
    let behind = now_ns - next_fire_ns;
    let passed = behind / interval_ns;
    let latest = next_fire_ns + passed * interval_ns;
    let earlier = u64::from(adv.fire_count - 1) * interval_ns;
    PeriodAdvance {
        first_fire_ns: latest - earlier,
        ..adv
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    /// A 10 ms node due at 1.010 s, clock jumped to 1.100 s: ten deadlines
    /// passed (1.010 .. 1.100). Ceiling one: one fire, at 1.100 s, next
    /// deadline 1.110 s.
    #[test]
    fn a_ceiling_of_one_fires_the_latest_deadline_once() {
        let adv = ceiled_period_advance(1_010 * MS, 1_100 * MS, 10 * MS, 1);
        assert_eq!(
            adv,
            PeriodAdvance {
                fire_count: 1,
                first_fire_ns: 1_100 * MS,
                next_fire_ns: 1_110 * MS,
            }
        );
    }

    /// The next deadline after the jump lies past `u64::MAX`, so the capped
    /// advance saturates it. Due at MAX-7 with a 3 ns period, the clock at
    /// MAX: the deadlines passed are MAX-7, MAX-4 and MAX-1, so a ceiling of
    /// one fires at MAX-1 and a ceiling of two at MAX-4.
    #[test]
    fn the_latest_deadline_is_right_when_the_next_one_saturates() {
        let max = u64::MAX;
        let one = ceiled_period_advance(max - 7, max, 3, 1);
        assert_eq!(
            one,
            PeriodAdvance {
                fire_count: 1,
                first_fire_ns: max - 1,
                next_fire_ns: max,
            }
        );
        let two = ceiled_period_advance(max - 7, max, 3, 2);
        assert_eq!(two.fire_count, 2);
        assert_eq!(two.first_fire_ns, max - 4);
        let all = ceiled_period_advance(max - 7, max, 3, 9);
        assert_eq!(all.fire_count, 3);
        assert_eq!(all.first_fire_ns, max - 7);
    }

    /// The same jump through `PeriodCaps`: with the ceiling the node fires
    /// once, without it the unbounded default fires all ten, oldest first.
    #[test]
    fn the_ceiling_is_what_turns_ten_fires_into_one() {
        let unceiled = PeriodCaps::default().advance(None, 1_010 * MS, 1_100 * MS, 10 * MS);
        assert_eq!(
            unceiled,
            PeriodAdvance {
                fire_count: 10,
                first_fire_ns: 1_010 * MS,
                next_fire_ns: 1_110 * MS,
            }
        );
        let ceiled = PeriodCaps {
            override_cap: None,
            ceiling: Some(1),
        }
        .advance(None, 1_010 * MS, 1_100 * MS, 10 * MS);
        assert_eq!(
            ceiled,
            PeriodAdvance {
                fire_count: 1,
                first_fire_ns: 1_100 * MS,
                next_fire_ns: 1_110 * MS,
            }
        );
    }

    /// A declared `max_catchup` is bounded by the ceiling too, and a declared
    /// zero stays zero.
    #[test]
    fn the_ceiling_bounds_a_declared_max_catchup_and_keeps_a_declared_zero() {
        let caps = PeriodCaps {
            override_cap: Some(4),
            ceiling: Some(1),
        };
        assert_eq!(
            caps.advance(Some(8), 1_010 * MS, 1_100 * MS, 10 * MS)
                .fire_count,
            1
        );
        let zero = caps.advance(Some(0), 1_010 * MS, 1_100 * MS, 10 * MS);
        assert_eq!(zero.fire_count, 0);
        assert_eq!(
            zero.next_fire_ns,
            1_110 * MS,
            "the deadline still moves past now"
        );
    }

    /// One deadline passed: the ceiling changes nothing, the fire is at that
    /// deadline (1.020 s for a jump from 1.010 s to 1.025 s with 1.020 s due).
    #[test]
    fn a_single_passed_deadline_is_the_same_with_or_without_the_ceiling() {
        let expect = PeriodAdvance {
            fire_count: 1,
            first_fire_ns: 1_020 * MS,
            next_fire_ns: 1_030 * MS,
        };
        assert_eq!(
            ceiled_period_advance(1_020 * MS, 1_025 * MS, 10 * MS, 1),
            expect
        );
        assert_eq!(
            catchup_clamp::period_advance(1_020 * MS, 1_025 * MS, 10 * MS, 1),
            expect
        );
    }

    /// Nothing due: nothing fires and the deadline does not move.
    #[test]
    fn nothing_due_is_nothing_fired() {
        let adv = ceiled_period_advance(1_030 * MS, 1_025 * MS, 10 * MS, 1);
        assert_eq!(adv.fire_count, 0);
        assert_eq!(adv.next_fire_ns, 1_030 * MS);
    }
}

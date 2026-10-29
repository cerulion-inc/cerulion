// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::time::Duration;

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::Int32;

// Data-triggered sink: `#[input(trigger)]` fires this node once per published
// count, so there is no `period_ms` here. A sink declares no `#[output]`.
#[cerulion_node]
#[derive(Default)]
pub struct PrinterNode {
    #[input(trigger)]
    count: Int32,

    // The increment the timer uses. The graph file sets both nodes from the
    // same `step` launch argument, so a count that went up by `step` is the
    // next one, not a miss.
    #[param(
        default = 1,
        min = 1,
        max = 1000,
        description = "how much count goes up per tick upstream"
    )]
    step: i32,

    received: u32,
    last: Option<i32>,
    missed: u64,
}

#[cerulion_node_impl]
impl PrinterNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.received = self.received.wrapping_add(1);
        // A fixed field reads straight out of the borrowed shared-memory
        // sample: no copy, no allocation. Bind it to a local first, because
        // the macro does not rewrite port reads inside another macro's
        // arguments.
        let value = self.count.data;
        let received = self.received;
        tracing::info!(value, received, "count received");

        // The timer counts up by `step`, so a larger jump means counts were
        // missed.
        // `warn_throttle!` says so at most once per second of graph time,
        // however fast the counts arrive.
        if let Some(last) = self.last {
            let gap = missed_between(last, value, self.step);
            if gap > 0 {
                self.missed += gap;
                let missed = self.missed;
                warn_throttle!(self, Duration::from_secs(1), missed, "counts missed");
            }
        }
        self.last = Some(value);
        Ok(())
    }
}

/// How many counts fell between `last` and `value` on a counter that goes up
/// by `step`. The timer's counter wraps from `i32::MAX` to `i32::MIN`, so the
/// gap is taken in that same wrapping space: one step is no miss, and a count
/// that did not move forward (a repeat, or a restarted timer) is no miss
/// either. A gap that is not a whole number of steps counts the whole steps
/// that fit inside it, less the one that arrived.
///
/// In a 32-bit wrapping counter a forward gap of `d` and a backward step of
/// `2^32 - d` are the same bits, so one rule must choose. The rule is the
/// half range: a gap of 1 to `i32::MAX` is forward, and a larger forward gap
/// reads as a step back, the same as a restarted timer, and counts no miss.
/// A `step` below 1 is outside the param's range and reads as 1.
fn missed_between(last: i32, value: i32, step: i32) -> u64 {
    let step = u64::from(step.max(1).unsigned_abs());
    match value.wrapping_sub(last) {
        gap if gap > 0 => (u64::from(gap.unsigned_abs()) / step).saturating_sub(1),
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::missed_between;

    #[test]
    fn the_missed_count_is_the_forward_gap_in_the_wrapping_counter() {
        // (last, value, missed) with a step of 1, by hand.
        let vectors: &[(i32, i32, u64)] = &[
            (1, 2, 0),
            (1, 5, 3),
            // The counter wraps: MAX then MIN is one step.
            (i32::MAX, i32::MIN, 0),
            // Across the wrap, MAX - 1 then MIN skips MAX.
            (i32::MAX - 1, i32::MIN, 1),
            (i32::MAX, i32::MIN + 2, 2),
            // A repeat, or a timer that started over, is no miss.
            (7, 7, 0),
            (500, 1, 0),
            // The largest forward step, i32::MAX, is the largest miss.
            (0, i32::MAX, (i32::MAX - 1) as u64),
            (i32::MIN, -1, (i32::MAX - 1) as u64),
            // One past the half range reads as a step back: no miss.
            (0, i32::MIN, 0),
            (-1, i32::MAX, 0),
        ];
        for &(last, value, want) in vectors {
            assert_eq!(missed_between(last, value, 1), want, "{last} then {value}");
        }
    }

    #[test]
    fn a_count_that_went_up_by_the_step_is_no_miss() {
        // (last, value, step, missed), by hand. The shipped graph runs step 2.
        let vectors: &[(i32, i32, i32, u64)] = &[
            // Every count arrives: 2, 4, 6 is no miss.
            (2, 4, 2, 0),
            (4, 6, 2, 0),
            // 2 then 8 skips 4 and 6.
            (2, 8, 2, 2),
            // A gap that is not a whole number of steps counts whole steps.
            (2, 7, 2, 1),
            // A gap smaller than one step is no miss.
            (2, 3, 2, 0),
            (5, 10, 5, 0),
            (5, 20, 5, 2),
            // Across the wrap with step 2: MAX - 1 then MIN is one step.
            (i32::MAX - 1, i32::MIN, 2, 0),
            (i32::MAX - 1, i32::MIN + 2, 2, 1),
            // A repeat or a step back is no miss at any step.
            (6, 6, 2, 0),
            (6, 2, 2, 0),
            // The largest forward gap at the largest step: 2_147_483 whole
            // steps fit in i32::MAX, less the one that arrived.
            (0, i32::MAX, 1000, 2_147_482),
            // A step below the param's range reads as 1.
            (1, 5, 0, 3),
            (1, 5, -4, 3),
        ];
        for &(last, value, step, want) in vectors {
            assert_eq!(
                missed_between(last, value, step),
                want,
                "{last} then {value} at step {step}"
            );
        }
    }
}

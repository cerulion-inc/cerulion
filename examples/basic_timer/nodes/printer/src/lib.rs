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
struct PrinterNode {
    #[input(trigger)]
    count: Int32,

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

        // The timer counts up by one, so a jump means counts were missed.
        // `warn_throttle!` says so at most once per second of graph time,
        // however fast the counts arrive.
        if let Some(last) = self.last {
            let gap = missed_between(last, value);
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

/// How many counts fell between `last` and `value`. The timer's counter wraps
/// from `i32::MAX` to `i32::MIN`, so the step is taken in that same wrapping
/// space: one step is no miss, and a count that did not move forward (a
/// repeat, or a restarted timer) is no miss either.
///
/// In a 32-bit wrapping counter a forward gap of `d` and a backward step of
/// `2^32 - d` are the same bits, so one rule must choose. The rule is the
/// half range: a step of 1 to `i32::MAX` is forward, which caps the missed
/// count at `i32::MAX - 1`; a larger forward gap reads as a step back, the
/// same as a restarted timer, and counts no miss.
fn missed_between(last: i32, value: i32) -> u64 {
    match value.wrapping_sub(last) {
        step if step > 1 => u64::from(step.unsigned_abs() - 1),
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::missed_between;

    #[test]
    fn the_missed_count_is_the_forward_gap_in_the_wrapping_counter() {
        // (last, value, missed), by hand.
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
            assert_eq!(missed_between(last, value), want, "{last} then {value}");
        }
    }
}

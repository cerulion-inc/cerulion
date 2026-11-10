// SPDX-License-Identifier: AGPL-3.0-only
//! How a `block` input reports the drains its credit word clipped at 0.
//!
//! A drain that finds the word already at 0 removed frames the word never
//! counted ([`crate::credit::CreditShared::record_drained`] returns how many).
//! Each such drain is one failure of a [`FailureRegimeLatch`]: the head of a
//! regime is loud, repeats are counted, and each decade of the running total
//! is loud again.
//!
//! # What closes a regime
//!
//! The usual source of uncounted frames is a writer outside the graph on a
//! `multi_publisher_topics` topic, and its frames interleave with the in-graph
//! producer's, which the word counts and which drain clean. A clean drain
//! therefore says nothing about whether the outside writer has stopped, and a
//! regime that closed at every clean drain would open a new loud head at
//! every outside frame. So a regime closes only after a run of consecutive
//! clean drains, and every close doubles the run the next regime needs: 1,
//! then 2, 4, 8 and so on, saturating at `u64::MAX`.
//!
//! With outside frames at most `G >= 1` clean drains apart, at most
//! `floor(log2(G)) + 1` regimes close, so at most `floor(log2(G)) + 2` loud
//! heads are logged, and the one regime left open keeps reporting through the
//! decade lines. Once the outside frames stop, that regime closes after `2^c`
//! clean drains, `c` being the number of regimes closed before it (at most
//! `2G` when the interleave opened them all), and logs its recovery line if
//! it counted a repeat. A burst of uncounted frames with nothing interleaved
//! (native history to a late joiner) is one regime; when it is the input's
//! first, the first clean drain closes it.
//!
//! The decision reads only the stream of drains, never a clock, so a replay
//! of the same drains logs the same lines.

use super::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

/// What one drain's report logs, as [`DrainClipLatch::observe`] decides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DrainClipReport {
    /// Nothing: a clean drain with no regime open, or one that does not yet
    /// complete the run that closes the open regime, or one that closes a
    /// regime with no counted repeat.
    Quiet,
    /// The drain clipped: log it as the latch decided.
    Clipped(RegimeDecision),
    /// The drain completed the run that closes a regime with counted repeats:
    /// log one recovery line carrying how many.
    Recovered {
        /// Repeats the closed regime logged at `debug!` only.
        suppressed: u64,
    },
}

/// The clipped-drain report state of one `block` input: the shared latch plus
/// the clean run that closes its open regime (see the module docs).
#[derive(Debug)]
pub(crate) struct DrainClipLatch {
    latch: FailureRegimeLatch,
    /// Consecutive clean drains since the last clipped one, counted while a
    /// regime is open.
    clean_run: u64,
    /// The clean run that closes the open regime: 1 at first, doubled at every
    /// close.
    close_after: u64,
}

impl DrainClipLatch {
    /// No regime open, and a run of 1 closes the first one.
    pub(crate) const fn new() -> Self {
        Self {
            latch: FailureRegimeLatch::new(),
            clean_run: 0,
            close_after: 1,
        }
    }

    /// Record one drain that clipped `clipped` frames (0 for a clean drain)
    /// and return what to log. A clean drain with no regime open is one
    /// branch.
    #[inline]
    pub(crate) fn observe(&mut self, clipped: u64) -> DrainClipReport {
        if clipped > 0 {
            self.clean_run = 0;
            return DrainClipReport::Clipped(self.latch.on_failure());
        }
        if !self.latch.is_failing() {
            return DrainClipReport::Quiet;
        }
        self.clean_run = self.clean_run.saturating_add(1);
        if self.clean_run < self.close_after {
            return DrainClipReport::Quiet;
        }
        self.close_after = self.close_after.saturating_mul(2);
        match self.latch.on_success() {
            Some(suppressed) => DrainClipReport::Recovered { suppressed },
            None => DrainClipReport::Quiet,
        }
    }

    /// Drains that clipped, across every regime. Never reset.
    #[inline]
    pub(crate) fn total_failures(&self) -> u64 {
        self.latch.total_failures()
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`). A
// subscriber's block probe holds this latch by value.
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(DrainClipLatch {
        latch,
        clean_run,
        close_after
    })]
}

#[cfg(test)]
mod tests {
    use super::DrainClipReport::{Clipped, Quiet, Recovered};
    use super::*;
    use crate::transport::failure_regime_latch::RegimeDecision::{Loud, StillFailing, Suppressed};

    /// Every report of `drains` on a fresh latch, in order.
    fn reports(drains: &[u64]) -> Vec<DrainClipReport> {
        let mut latch = DrainClipLatch::new();
        drains.iter().map(|&c| latch.observe(c)).collect()
    }

    /// The reports that log something, with their index into `drains`.
    fn logged(drains: &[u64]) -> Vec<(usize, DrainClipReport)> {
        reports(drains)
            .into_iter()
            .enumerate()
            .filter(|(_, r)| *r != Quiet)
            .collect()
    }

    /// Clean drains on an input that never clipped log nothing and count
    /// nothing.
    #[test]
    fn clean_drains_with_no_regime_open_log_nothing() {
        let mut latch = DrainClipLatch::new();
        for _ in 0..3 {
            assert_eq!(latch.observe(0), Quiet);
        }
        assert_eq!(latch.total_failures(), 0);
    }

    /// Back-to-back clips are one regime, and the first regime closes at the
    /// first clean drain. Oracle, by hand: `[2, 1, 1, 0, 1]` reports a loud
    /// head, repeats 1 and 2, a recovery line carrying 2, and a loud head
    /// again; four clipped drains in all. The clip size does not matter, one
    /// drain is one failure.
    #[test]
    fn back_to_back_clips_are_one_regime_closed_by_the_first_clean_drain() {
        let mut latch = DrainClipLatch::new();
        let got: Vec<_> = [2, 1, 1, 0, 1].iter().map(|&c| latch.observe(c)).collect();
        assert_eq!(
            got,
            vec![
                Clipped(Loud),
                Clipped(Suppressed { suppressed: 1 }),
                Clipped(Suppressed { suppressed: 2 }),
                Recovered { suppressed: 2 },
                Clipped(Loud),
            ]
        );
        assert_eq!(latch.total_failures(), 4);
    }

    /// An outside writer whose every frame follows one counted frame: twenty
    /// cycles of `[0, 1]`, then three clean drains. Oracle, by hand: the first
    /// cycle's clip opens a regime, the second cycle's clean drain is the run
    /// of 1 that closes it with nothing to report, and the second cycle's clip
    /// opens a regime that needs a run of 2, which no cycle gives. So the clips
    /// of cycles 3 to 9 and 11 to 20 are repeats 1 to 17, and cycle 10's clip
    /// is the decade line at a total of 10. The second clean drain after the
    /// cycles closes the regime with a recovery line carrying 17, and the third
    /// finds no regime open. Two runs report the same.
    #[test]
    fn an_interleaved_writer_opens_two_regimes_and_holds_the_second_open() {
        let mut drains = Vec::new();
        for _ in 0..20 {
            drains.extend([0, 1]);
        }
        drains.extend([0, 0, 0]);
        let mut want = vec![Quiet, Clipped(Loud), Quiet, Clipped(Loud)];
        for n in 1..=7 {
            want.extend([Quiet, Clipped(Suppressed { suppressed: n })]);
        }
        want.extend([
            Quiet,
            Clipped(StillFailing {
                total: 10,
                suppressed: 7,
            }),
        ]);
        for n in 8..=17 {
            want.extend([Quiet, Clipped(Suppressed { suppressed: n })]);
        }
        want.extend([Quiet, Recovered { suppressed: 17 }, Quiet]);
        let first = reports(&drains);
        assert_eq!(first, want, "the reports equal the hand oracle");
        assert_eq!(reports(&drains), first, "and a second run equals the first");
    }

    /// Outside frames five clean drains apart: every close doubles the run the
    /// next regime needs. Five cycles of a clip and five clean drains, then
    /// three clean drains. Oracle, by hand: regimes needing runs of 1, 2 and 4
    /// open at drains 0, 6 and 12 and close silently at drains 1, 8 and 16;
    /// the regime opened at drain 18 needs a run of 8, so the clip at drain 24
    /// is its repeat 1, and its run reaches 8 at drain 32, the recovery line.
    #[test]
    fn every_close_doubles_the_clean_run_the_next_regime_needs() {
        let mut drains = Vec::new();
        for _ in 0..5 {
            drains.extend([1, 0, 0, 0, 0, 0]);
        }
        drains.extend([0, 0, 0]);
        assert_eq!(
            logged(&drains),
            vec![
                (0, Clipped(Loud)),
                (6, Clipped(Loud)),
                (12, Clipped(Loud)),
                (18, Clipped(Loud)),
                (24, Clipped(Suppressed { suppressed: 1 })),
                (32, Recovered { suppressed: 1 }),
            ]
        );
    }
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Per-topic arrival period learner for the period-learned park (feature `period-park`).
//!
//! Pure and integer-only: the same arrival sequence always yields the same state, with no clock reads.
//! A topic is REGULAR only after `REGULAR_AFTER` consecutive in-range gaps whose smoothed deviation is at most one eighth of the smoothed period and at most a quarter of [`MAX_WINDOW_NS`]. Any other topic has no prediction and keeps the stock park behaviour.
//! [`TopicLearners`] holds one learner per topic of a wait set and matches them by position to the wait set's mapped doorbells.

/// Shortest gap treated as a period (ns). Shorter gaps are bursts.
pub const MIN_PERIOD_NS: u64 = 200_000;
/// Longest gap treated as a period (ns). Longer gaps are idle spells.
pub const MAX_PERIOD_NS: u64 = 100_000_000;
/// In-range gaps required before a topic can be regular.
pub const REGULAR_AFTER: u32 = 8;
/// Smallest half-width of the park window (ns), and the first step of the early widening.
pub const MIN_WINDOW_NS: u64 = 20_000;
/// Largest half-width of the park window (ns). A topic whose deviation needs a wider window is not regular.
pub const MAX_WINDOW_NS: u64 = 100_000;
/// Longest window one prediction spans (ns), early widening included.
pub const MAX_PARK_NS: u64 = 2 * MAX_WINDOW_NS;
/// Learners kept beyond the mapped bell count for topics that left the wait set; a remap past this drops them.
pub const LEARNER_SLACK: usize = 16;

/// What one arrival did to the estimate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Observation {
    /// First arrival seen: no gap yet.
    First,
    /// Time did not advance: the estimate was reset.
    NotMonotonic,
    /// Gap outside the period range: the estimate was reset.
    OutOfRange,
    /// Gap folded in, not yet regular.
    Learning,
    /// Gap folded in, topic is regular.
    Regular,
    /// A regular topic missed its beat by more than the tolerance: relearning.
    Dropped,
}

/// Predicted next arrival and the park window around it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Prediction {
    /// Expected arrival time, ns on the caller's monotonic clock.
    pub next_ns: u64,
    /// Half-width of the park window, ns, at most [`MAX_WINDOW_NS`].
    pub window_ns: u64,
    /// Extra time the window opens before `next_ns - window_ns`, ns; `2 * window_ns + early_ns` is at most [`MAX_PARK_NS`].
    pub early_ns: u64,
}

impl Prediction {
    /// Window start: `next_ns - window_ns - early_ns`, saturating at 0.
    pub fn open_ns(&self) -> u64 {
        self.next_ns
            .saturating_sub(self.window_ns.saturating_add(self.early_ns))
    }

    /// Window end: `next_ns + window_ns`, saturating at `u64::MAX`.
    pub fn close_ns(&self) -> u64 {
        self.next_ns.saturating_add(self.window_ns)
    }
}

/// Smoothed period and deviation of one topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PeriodLearner {
    last_ns: Option<u64>,
    period_ns: u64,
    dev_ns: u64,
    run: u32,
    regular: bool,
    early_ns: u64,
}

fn ewma(old: u64, sample: u64) -> u64 {
    if sample >= old {
        old + (sample - old) / 8
    } else {
        old - (old - sample) / 8
    }
}

impl PeriodLearner {
    /// A learner that has seen nothing.
    pub fn new() -> Self {
        Self::default()
    }

    fn forget(&mut self) {
        self.period_ns = 0;
        self.dev_ns = 0;
        self.run = 0;
        self.regular = false;
        self.early_ns = 0;
    }

    fn window_ns(&self) -> u64 {
        self.dev_ns
            .saturating_mul(4)
            .max(MIN_WINDOW_NS)
            .min(self.period_ns / 2)
    }

    /// Fold in one arrival at `now_ns`.
    pub fn observe(&mut self, now_ns: u64) -> Observation {
        let Some(prev) = self.last_ns else {
            self.last_ns = Some(now_ns);
            return Observation::First;
        };
        self.last_ns = Some(now_ns);
        if now_ns <= prev {
            self.forget();
            return Observation::NotMonotonic;
        }
        let gap = now_ns - prev;
        if !(MIN_PERIOD_NS..=MAX_PERIOD_NS).contains(&gap) {
            self.forget();
            return Observation::OutOfRange;
        }
        if self.period_ns == 0 {
            self.period_ns = gap;
            self.run = 1;
            return Observation::Learning;
        }
        let err = self.period_ns.abs_diff(gap);
        let tolerance = 3 * self.dev_ns.max(self.period_ns / 16);
        if self.run >= REGULAR_AFTER && err > tolerance {
            self.forget();
            self.period_ns = gap;
            self.run = 1;
            return Observation::Dropped;
        }
        self.period_ns = ewma(self.period_ns, gap);
        self.dev_ns = ewma(self.dev_ns, err);
        self.run = self.run.saturating_add(1);
        self.regular = self.run >= REGULAR_AFTER
            && self.dev_ns.saturating_mul(8) <= self.period_ns
            && self.dev_ns.saturating_mul(4) <= MAX_WINDOW_NS;
        if self.regular {
            Observation::Regular
        } else {
            self.early_ns = 0;
            Observation::Learning
        }
    }

    /// The predicted message was seen late: before its window opened, or before the park's first check. The next
    /// window opens earlier by a widening that doubles per such arrival, from [`MIN_WINDOW_NS`] up to what keeps the
    /// window within [`MAX_PARK_NS`]. No effect unless the topic is regular.
    pub fn arrived_before_window(&mut self) {
        if !self.regular {
            return;
        }
        let room = MAX_PARK_NS.saturating_sub(self.window_ns().saturating_mul(2));
        self.early_ns = self.early_ns.saturating_mul(2).max(MIN_WINDOW_NS).min(room);
    }

    /// The park caught the predicted message, so the last arrival is the ring time: the early widening is dropped.
    pub fn caught_in_window(&mut self) {
        self.early_ns = 0;
    }

    /// The prediction, or `None` unless the topic is regular.
    pub fn predict(&self) -> Option<Prediction> {
        if !self.regular {
            return None;
        }
        let last = self.last_ns?;
        let window_ns = self.window_ns();
        Some(Prediction {
            next_ns: last.saturating_add(self.period_ns),
            window_ns,
            early_ns: self
                .early_ns
                .min(MAX_PARK_NS.saturating_sub(window_ns.saturating_mul(2))),
        })
    }
}

/// How a predicted block ended, applied to the predicted topic's learner by the next [`TopicLearners::note`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WindowEnd {
    /// The predicted message was seen before the park could catch its ring: a wake cut the kernel
    /// lead, or the predicted bell had already rung when the park made its first check. The arrival
    /// is noted late, so the next window widens on its early side.
    BeforePark,
    /// The park armed on the predicted bell saw that bell ring, so the noted arrival is the ring time.
    Caught,
}

/// One learner per topic of a wait set, matched by position to the wait set's mapped bells.
#[derive(Debug, Default)]
pub struct TopicLearners {
    /// (topic, last bell seq seen, learner).
    learners: Vec<(String, u64, PeriodLearner)>,
    /// Learner index for each mapped bell, valid for `generation`.
    map: Vec<usize>,
    /// The bell set generation `map` was built for.
    generation: Option<u64>,
    /// The last predicted block's bell index and how it ended.
    pending: Option<(usize, WindowEnd)>,
}

impl TopicLearners {
    /// Is `map` built for bell set `generation`?
    pub fn is_mapped(&self, generation: u64) -> bool {
        self.generation == Some(generation)
    }

    /// Match learners to the bells of `generation`, given as (topic, current seq) in bell order.
    /// A topic seen for the first time gets a learner seeded with its seq (that arrival is not observed).
    /// When the learners outnumber `bells` by more than [`LEARNER_SLACK`] before the match, those of topics outside `bells` are dropped.
    pub fn remap(&mut self, generation: u64, bells: &[(&str, u64)]) {
        if self.learners.len() > bells.len() + LEARNER_SLACK {
            self.learners
                .retain(|(t, _, _)| bells.iter().any(|(b, _)| b == t));
        }
        self.map.clear();
        for &(topic, seq) in bells {
            let idx = match self.learners.iter().position(|(t, _, _)| t == topic) {
                Some(i) => i,
                None => {
                    self.learners
                        .push((topic.to_owned(), seq, PeriodLearner::new()));
                    self.learners.len() - 1
                }
            };
            self.map.push(idx);
        }
        self.generation = Some(generation);
        self.pending = None;
    }

    /// Record how the predicted block on bell `bell` ended (`None`: no predicted wake to apply).
    pub fn end_window(&mut self, ended: Option<(usize, WindowEnd)>) {
        self.pending = ended;
    }

    /// Observe an arrival at `now_ns` for every bell whose seq (`seqs`, in bell order) moved since the last note,
    /// then apply the pending predicted block's end to its bell's learner if that bell moved.
    pub fn note(&mut self, now_ns: u64, seqs: impl IntoIterator<Item = u64>) {
        let pending = self.pending.take();
        for (bell, seq) in seqs.into_iter().enumerate() {
            let Some(&idx) = self.map.get(bell) else {
                break;
            };
            let (_, last, learner) = &mut self.learners[idx];
            if seq == *last {
                continue;
            }
            *last = seq;
            learner.observe(now_ns);
            match pending {
                Some((b, WindowEnd::BeforePark)) if b == bell => learner.arrived_before_window(),
                Some((b, WindowEnd::Caught)) if b == bell => learner.caught_in_window(),
                _ => {}
            }
        }
    }

    /// Record each mapped bell's seq (`seqs`, in bell order) as seen without observing an arrival, and drop the
    /// pending predicted block's end. For seqs read before a readiness probe that serves every message behind them.
    pub fn sync(&mut self, seqs: impl IntoIterator<Item = u64>) {
        self.pending = None;
        for (bell, seq) in seqs.into_iter().enumerate() {
            let Some(&idx) = self.map.get(bell) else {
                break;
            };
            self.learners[idx].1 = seq;
        }
    }

    /// Each mapped bell's seq at the last [`Self::note`] or [`Self::sync`] (the seed for a topic not yet noted), in
    /// bell order. A bell whose seq differs from this has rung since the learners last looked.
    pub fn noted_seqs(&self) -> impl Iterator<Item = u64> + '_ {
        self.map.iter().map(|&idx| self.learners[idx].1)
    }

    /// The window that opens first among the mapped bells' predictions that close after `now_ns`, with its bell index.
    pub fn next(&self, now_ns: u64) -> Option<(usize, Prediction)> {
        self.map
            .iter()
            .enumerate()
            .filter_map(|(bell, &idx)| self.learners[idx].2.predict().map(|p| (bell, p)))
            .filter(|(_, p)| p.close_ns() > now_ns)
            .min_by_key(|(_, p)| p.open_ns())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(l: &mut PeriodLearner, start: u64, gap: u64, n: u64) -> u64 {
        let mut t = start;
        for _ in 0..n {
            t += gap;
            l.observe(t);
        }
        t
    }

    #[test]
    fn a_steady_topic_becomes_regular_and_predicts_one_period_ahead() {
        let mut l = PeriodLearner::new();
        assert_eq!(l.observe(1_000_000), Observation::First);
        let last = feed(&mut l, 1_000_000, 1_000_000, 8);
        assert_eq!(l.observe(last + 1_000_000), Observation::Regular);
        let p = l.predict().expect("regular topic predicts");
        assert_eq!(p.next_ns, last + 2_000_000);
        assert_eq!(p.window_ns, MIN_WINDOW_NS);
        assert_eq!(p.early_ns, 0);
    }

    #[test]
    fn no_prediction_before_enough_gaps() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        feed(&mut l, 0, 1_000_000, 4);
        assert_eq!(l.predict(), None);
    }

    #[test]
    fn a_gap_at_each_range_edge_is_learned_and_just_outside_is_not() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        assert_eq!(l.observe(MIN_PERIOD_NS), Observation::Learning);
        let mut m = PeriodLearner::new();
        m.observe(0);
        assert_eq!(m.observe(MAX_PERIOD_NS), Observation::Learning);
        let mut o = PeriodLearner::new();
        o.observe(0);
        assert_eq!(o.observe(MIN_PERIOD_NS - 1), Observation::OutOfRange);
        let mut q = PeriodLearner::new();
        q.observe(0);
        assert_eq!(q.observe(MAX_PERIOD_NS + 1), Observation::OutOfRange);
    }

    #[test]
    fn non_advancing_time_resets_the_estimate() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        let last = feed(&mut l, 0, 1_000_000, 9);
        assert!(l.predict().is_some());
        assert_eq!(l.observe(last), Observation::NotMonotonic);
        assert_eq!(l.predict(), None);
        assert_eq!(l.observe(last - 5), Observation::NotMonotonic);
    }

    #[test]
    fn a_missed_beat_drops_a_regular_topic_to_relearning() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        let last = feed(&mut l, 0, 1_000_000, 10);
        assert!(l.predict().is_some());
        assert_eq!(l.observe(last + 10_000_000), Observation::Dropped);
        assert_eq!(l.predict(), None);
    }

    #[test]
    fn a_jittery_topic_never_becomes_regular() {
        let mut l = PeriodLearner::new();
        let mut t = 0;
        l.observe(t);
        for i in 0..200u64 {
            t += if i % 2 == 0 { 700_000 } else { 1_300_000 };
            l.observe(t);
            assert_eq!(l.predict(), None, "step {i}");
        }
    }

    #[test]
    fn a_burst_of_tiny_gaps_never_predicts() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        feed(&mut l, 0, 1_000, 100);
        assert_eq!(l.predict(), None);
    }

    #[test]
    fn a_clock_near_u64_max_saturates_instead_of_wrapping() {
        let mut l = PeriodLearner::new();
        let start = u64::MAX - 20_000_000;
        l.observe(start);
        let last = feed(&mut l, start, 1_000_000, 9);
        let p = l.predict().expect("regular");
        assert_eq!(p.next_ns, last + 1_000_000);
        let mut e = PeriodLearner::new();
        let begin = u64::MAX - 950_000_000;
        e.observe(begin);
        feed(&mut e, begin, 100_000_000, 9);
        let q = e.predict().expect("regular");
        assert_eq!(q.next_ns, u64::MAX);
        assert_eq!(q.close_ns(), u64::MAX);
    }

    #[test]
    fn the_same_arrivals_give_the_same_state() {
        let gaps = [
            1_000_000u64,
            1_010_000,
            990_000,
            1_000_000,
            1_020_000,
            980_000,
            1_000_000,
            1_005_000,
            995_000,
            1_000_000,
        ];
        let (mut a, mut b) = (PeriodLearner::new(), PeriodLearner::new());
        let (mut ta, mut tb) = (0u64, 0u64);
        a.observe(0);
        b.observe(0);
        for g in gaps {
            ta += g;
            tb += g;
            assert_eq!(a.observe(ta), b.observe(tb));
        }
        assert_eq!(a, b);
        assert_eq!(a.predict(), b.predict());
    }

    #[test]
    fn the_window_never_exceeds_half_a_period() {
        let mut l = PeriodLearner::new();
        l.observe(0);
        feed(&mut l, 0, MIN_PERIOD_NS, 12);
        let p = l.predict().expect("regular at the shortest period");
        assert!(p.window_ns <= MIN_PERIOD_NS / 2);
    }

    /// Alternating gaps of period +- `jitter`, 400 arrivals; returns (arrivals that ended Regular, widest window seen).
    fn alternate(period: u64, jitter: u64) -> (usize, u64) {
        let mut l = PeriodLearner::new();
        let mut t = 0u64;
        l.observe(t);
        let (mut regular, mut widest) = (0, 0);
        for i in 0..400u64 {
            t += if i % 2 == 0 {
                period + jitter
            } else {
                period - jitter
            };
            if l.observe(t) == Observation::Regular {
                regular += 1;
            }
            if let Some(p) = l.predict() {
                widest = widest.max(p.close_ns() - p.open_ns());
            }
        }
        (regular, widest)
    }

    #[test]
    fn a_topic_whose_window_would_pass_the_cap_is_not_regular() {
        let (regular, widest) = alternate(10_000_000, 1_100_000);
        assert_eq!((regular, widest), (0, 0), "+-1.1 ms at 100 Hz");
        let (regular, widest) = alternate(10_000_000, 500_000);
        assert_eq!((regular, widest), (0, 0), "+-500 us at 100 Hz");
        let (regular, widest) = alternate(10_000_000, 40_000);
        assert_eq!(
            (regular, widest),
            (0, 0),
            "+-40 us needs a 160 us half-width"
        );
        let (regular, widest) = alternate(10_000_000, 10_000);
        assert!(regular > 300, "+-10 us at 100 Hz is regular ({regular})");
        assert!(widest <= MAX_PARK_NS, "window {widest} ns");
        assert!(
            widest > 2 * MIN_WINDOW_NS,
            "the window tracks the deviation ({widest} ns)"
        );
    }

    #[test]
    fn the_early_widening_doubles_to_the_park_cap_and_a_catch_drops_it() {
        let mut l = PeriodLearner::new();
        l.arrived_before_window();
        assert_eq!(
            l,
            PeriodLearner::new(),
            "no widening before the topic is regular"
        );
        l.observe(0);
        feed(&mut l, 0, 1_000_000, 9);
        let base = l.predict().expect("regular");
        assert_eq!((base.window_ns, base.early_ns), (MIN_WINDOW_NS, 0));
        let mut seen = Vec::new();
        for _ in 0..6 {
            l.arrived_before_window();
            let p = l.predict().expect("still regular");
            assert_eq!(p.next_ns, base.next_ns);
            assert!(p.close_ns() - p.open_ns() <= MAX_PARK_NS);
            seen.push(p.early_ns);
        }
        assert_eq!(seen, [20_000, 40_000, 80_000, 160_000, 160_000, 160_000]);
        l.caught_in_window();
        assert_eq!(l.predict(), Some(base));
        l.arrived_before_window();
        assert_eq!(l.observe(base.next_ns + 10_000_000), Observation::Dropped);
        assert_eq!(l.early_ns, 0);
        l.observe(base.next_ns + 11_000_000);
        assert_eq!(
            l.predict(),
            None,
            "a reset forgets the widening with the period"
        );
    }

    /// Arrivals exactly every `period`, each observed `late` ns after it arrives unless a park caught it.
    /// Returns, per arrival, whether the park caught it.
    fn late_observer(period: u64, late: u64, n: u64) -> Vec<bool> {
        let mut l = PeriodLearner::new();
        let mut caught = Vec::new();
        for k in 1..=n {
            let arrival = k * period;
            match l.predict() {
                Some(p) if arrival < p.open_ns() => {
                    l.observe(arrival + late);
                    l.arrived_before_window();
                    caught.push(false);
                }
                Some(p) if arrival < p.close_ns() => {
                    l.observe(arrival);
                    l.caught_in_window();
                    caught.push(true);
                }
                _ => {
                    l.observe(arrival + late);
                    caught.push(false);
                }
            }
        }
        caught
    }

    #[test]
    fn a_late_observed_anchor_widens_until_the_park_catches_and_then_holds() {
        let caught = late_observer(1_000_000, 60_000, 40);
        let first = caught
            .iter()
            .position(|&c| c)
            .expect("the park catches within 40 arrivals");
        assert!(
            first <= REGULAR_AFTER as usize + 4,
            "first catch at arrival {first}"
        );
        assert!(
            caught[first..].iter().all(|&c| c),
            "every arrival after the first catch is caught: {caught:?}"
        );
        let never = late_observer(1_000_000, 300_000, 40);
        assert!(
            never.iter().all(|&c| !c),
            "a lag past the park cap is never caught: {never:?}"
        );
    }

    fn regular_set(topics: &[&str], period: u64, last_ns: u64) -> TopicLearners {
        let mut set = TopicLearners::default();
        let bells: Vec<(&str, u64)> = topics.iter().map(|t| (*t, 0)).collect();
        set.remap(1, &bells);
        for k in 0..=10u64 {
            let t = last_ns - (10 - k) * period;
            let seqs: Vec<u64> = topics.iter().map(|_| k + 1).collect();
            set.note(t, seqs);
        }
        set
    }

    #[test]
    fn the_next_window_names_the_predicted_topics_bell() {
        let mut set = TopicLearners::default();
        set.remap(1, &[("/quiet", 0), ("/beat", 0)]);
        for k in 1..=10u64 {
            set.note(k * 1_000_000, [0, k]);
        }
        let t = 10_000_000;
        let (bell, p) = set.next(t).expect("the regular topic predicts");
        assert_eq!(bell, 1, "the window is /beat's, bell 1");
        assert_eq!(p.next_ns, t + 1_000_000);
        set.remap(2, &[("/beat", 10), ("/quiet", 0)]);
        assert_eq!(
            set.next(t).map(|(b, _)| b),
            Some(0),
            "a remap follows /beat to bell 0"
        );
        assert_eq!(set.learners.len(), 2, "the remap kept both learners");
    }

    #[test]
    fn a_closed_window_or_an_unmapped_topic_never_drives_the_next_window() {
        let mut set = regular_set(&["/slow", "/fast"], 1_000_000, 10_000_000);
        let (_, slow) = set.next(10_000_000).expect("both regular");
        let after = slow.close_ns();
        set.note(after, [11, 12]);
        let (bell, p) = set.next(after).expect("/fast predicts again");
        assert!(p.close_ns() > after);
        assert_eq!(bell, 1);
        let mut slow_only = regular_set(&["/slow", "/fast"], 1_000_000, 10_000_000);
        let closed = slow_only.next(10_000_000).expect("regular").1.close_ns();
        assert_eq!(
            slow_only.next(closed),
            None,
            "every window closed: no prediction"
        );
        slow_only.remap(2, &[("/other", 0)]);
        assert_eq!(
            slow_only.next(10_000_000),
            None,
            "learners of topics outside the bell set do not predict"
        );
        assert!(slow_only.is_mapped(2) && !slow_only.is_mapped(1));
    }

    #[test]
    fn a_bell_that_did_not_move_is_not_an_arrival_and_a_new_topic_is_only_seeded() {
        let mut set = TopicLearners::default();
        set.remap(1, &[("/a", 5)]);
        set.note(1_000_000, [5]);
        set.note(2_000_000, [5]);
        assert_eq!(
            set.learners[0].2,
            PeriodLearner::new(),
            "seq 5 was the seed: nothing observed"
        );
        set.note(3_000_000, [6]);
        assert_eq!(set.learners[0].2.last_ns, Some(3_000_000));
        set.note(4_000_000, [6, 9]);
        assert_eq!(
            set.learners.len(),
            1,
            "a seq past the mapped bells is ignored"
        );
    }

    #[test]
    fn the_noted_seqs_are_the_last_note_per_bell_in_bell_order() {
        let mut set = TopicLearners::default();
        set.remap(1, &[("/a", 5), ("/b", 9)]);
        assert_eq!(
            set.noted_seqs().collect::<Vec<_>>(),
            [5, 9],
            "a topic not yet noted reports its seed"
        );
        set.note(1_000_000, [7, 9]);
        assert_eq!(set.noted_seqs().collect::<Vec<_>>(), [7, 9]);
        set.note(2_000_000, [7, 9]);
        assert_eq!(
            set.noted_seqs().collect::<Vec<_>>(),
            [7, 9],
            "a note with no ring keeps the seqs"
        );
        set.remap(2, &[("/b", 40), ("/a", 8)]);
        assert_eq!(
            set.noted_seqs().collect::<Vec<_>>(),
            [9, 7],
            "a remap reorders by bell and does not re-seed a known topic"
        );
    }

    #[test]
    fn a_sync_marks_rings_seen_without_an_arrival_and_drops_the_pending_end() {
        // Seqs 1 to 11 noted at 0 to 10 ms: a regular 1 ms topic per bell, last seq 11.
        let mut set = regular_set(&["/a", "/b"], 1_000_000, 10_000_000);
        let before = set.learners[1].2;
        set.end_window(Some((1, WindowEnd::BeforePark)));
        set.sync([12, 12]);
        assert_eq!(set.noted_seqs().collect::<Vec<_>>(), [12, 12]);
        assert_eq!(set.learners[1].2, before, "a sync observes no arrival");
        set.note(11_000_000, [12, 13]);
        assert_eq!(
            set.learners[1].2.early_ns, 0,
            "the sync dropped the pending end"
        );
        assert_eq!(
            set.learners[1].2.predict().map(|p| p.next_ns),
            Some(12_000_000),
            "the ring to 13 at 11 ms is the arrival observed"
        );
        let at_11 = set.learners[1].2;
        set.sync([12, 13]);
        set.note(11_400_000, [12, 13]);
        assert_eq!(
            set.learners[1].2, at_11,
            "a note at the synced seqs sees no ring"
        );
        let mut unsynced = regular_set(&["/a", "/b"], 1_000_000, 10_000_000);
        unsynced.end_window(Some((1, WindowEnd::BeforePark)));
        unsynced.note(11_000_000, [12, 12]);
        assert_eq!(
            unsynced.learners[1].2.early_ns, MIN_WINDOW_NS,
            "without the sync the pending end widens the window"
        );
    }

    #[test]
    fn the_pending_window_end_reaches_only_its_own_bell_and_only_once() {
        let mut set = regular_set(&["/a", "/b"], 1_000_000, 10_000_000);
        set.end_window(Some((1, WindowEnd::BeforePark)));
        set.note(11_000_000, [12, 12]);
        assert_eq!(
            set.learners[0].2.early_ns, 0,
            "/a was not the predicted bell"
        );
        assert_eq!(set.learners[1].2.early_ns, MIN_WINDOW_NS);
        set.note(12_000_000, [13, 13]);
        assert_eq!(
            set.learners[1].2.early_ns, MIN_WINDOW_NS,
            "a pending end applies once"
        );
        set.end_window(Some((1, WindowEnd::BeforePark)));
        set.note(12_500_000, [14, 13]);
        assert_eq!(
            set.learners[1].2.early_ns, MIN_WINDOW_NS,
            "the predicted bell did not move: no widening"
        );
        set.end_window(Some((1, WindowEnd::Caught)));
        set.note(13_000_000, [14, 14]);
        assert_eq!(set.learners[1].2.early_ns, 0, "a catch drops the widening");
    }

    #[test]
    fn a_remap_drops_departed_learners_only_past_the_slack() {
        let mut set = TopicLearners::default();
        let names: Vec<String> = (0..=LEARNER_SLACK).map(|i| format!("/t{i}")).collect();
        for (g, name) in names.iter().enumerate() {
            set.remap(g as u64, &[(name.as_str(), 0)]);
        }
        assert_eq!(
            set.learners.len(),
            LEARNER_SLACK + 1,
            "16 departed plus 1 mapped: all kept"
        );
        set.remap(100, &[("/new", 0)]);
        assert_eq!(
            set.learners.len(),
            LEARNER_SLACK + 2,
            "the count is checked before the new topic is added"
        );
        set.remap(101, &[("/new", 0)]);
        assert_eq!(
            set.learners.len(),
            1,
            "17 departed is past the slack: only the mapped topic stays"
        );
        assert_eq!(set.learners[0].0, "/new");
        assert_eq!(set.map, [0]);
    }
}

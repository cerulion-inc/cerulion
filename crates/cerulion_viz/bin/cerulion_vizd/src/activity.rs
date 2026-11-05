// SPDX-License-Identifier: AGPL-3.0-only
//! The `activity` FOLD: one followed run's fire pages, kept and aged.
//!
//! # Why this module is PURE
//!
//! The pages come from `cerulion_netd::NetdClient::query_fires`, which needs a
//! robot hosting a run. The decisions (dedup, ageing, the window statistics and
//! the loss accounting) live here, where hand-written pages and a fake
//! [`Instant`] drive every one of them; the daemon's follower keeps only the
//! polling (the `runs.rs` precedent).
//!
//! # Ages need no clock sync
//!
//! A served page carries the robot's clock at the serve (`now_ns`) and each
//! fire's start on the same clock, so `now_ns - at_ns` is the fire's age when it
//! was served. The log stores that age with the desk [`Instant`] the page
//! arrived at, and an answer adds the desk time since. The robot's clock is
//! never compared with the desk's.
//!
//! # The cursor is inclusive
//!
//! A page's `next` is the last sequence it accounts for, and the follower passes
//! it unchanged as the next GET's cursor. An answer keeps the same rule for its
//! controller: the fires it carries plus its `lost` account for every sequence
//! in `(after, next]` the follower followed.

use std::collections::{BTreeMap, VecDeque};
use std::time::{Duration, Instant};

use cerulion_core::fire_tap::{FireFlags, FireRecord};
use cerulion_core::transport::cerulion_q::{FiresOutcome, FiresReply};

use crate::protocol::{ActivityResponse, ActivityState, FireSeen, InTick, NodeActivity};

/// The most fires one log holds; the oldest is dropped first.
pub const ACTIVITY_LOG_CAPACITY: usize = 2048;

/// The most fires one `activity` answer carries (the newest ones).
pub const MAX_ACTIVITY_FIRES: usize = 256;

/// One fire as the log keeps it: the robot's age of it at the serve, and the
/// desk instant the page arrived.
#[derive(Debug, Clone, Copy)]
struct Held {
    record: FireRecord,
    age_at_fetch_ns: u64,
    fetched_at: Instant,
}

/// What the newest served page said about the robot's tap, aged like a fire.
#[derive(Debug, Clone)]
struct Served {
    armed_age_ns: u64,
    in_tick: Vec<(u16, u64)>,
    fetched_at: Instant,
}

/// One followed run's fires and state. See the module docs.
#[derive(Debug, Clone)]
pub struct ActivityLog {
    robot: String,
    run_id: String,
    ring: VecDeque<Held>,
    state: ActivityState,
    served: Option<Served>,
    origin: Option<u64>,
    cursor: Option<u64>,
}

impl ActivityLog {
    /// An empty log for `(robot, run_id)`, in the `starting` state.
    pub fn new(robot: &str, run_id: &str) -> Self {
        Self {
            robot: robot.to_string(),
            run_id: run_id.to_string(),
            ring: VecDeque::new(),
            state: ActivityState::Starting,
            served: None,
            origin: None,
            cursor: None,
        }
    }

    /// Fold one reply that arrived at `fetched_at`, and return the cursor to
    /// ask with next: the newest `next` any served page carried.
    ///
    /// A served page adds the fires after the cursor (a fire it already holds,
    /// or one the cursor already accounts for, is skipped) and makes the state
    /// `live`. Any other outcome sets its state and keeps the fires and the
    /// cursor.
    pub fn absorb(&mut self, reply: &FiresReply, fetched_at: Instant) -> Option<u64> {
        match &reply.outcome {
            FiresOutcome::Served {
                now_ns,
                armed_since_ns,
                next,
                lost,
                in_tick,
                fires,
            } => {
                if self.origin.is_none() {
                    let span = (fires.len() as u64).saturating_add(*lost);
                    self.origin = Some(next.saturating_sub(span));
                }
                for record in fires {
                    if self.cursor.is_some_and(|cursor| record.seq <= cursor) {
                        continue;
                    }
                    if self.ring.len() == ACTIVITY_LOG_CAPACITY {
                        self.ring.pop_front();
                    }
                    self.ring.push_back(Held {
                        record: *record,
                        age_at_fetch_ns: now_ns.saturating_sub(record.at_ns),
                        fetched_at,
                    });
                    self.cursor = Some(record.seq);
                }
                self.cursor = Some(self.cursor.map_or(*next, |cursor| cursor.max(*next)));
                self.served = Some(Served {
                    armed_age_ns: now_ns.saturating_sub(*armed_since_ns),
                    in_tick: in_tick
                        .iter()
                        .map(|&(node, since_ns)| (node, now_ns.saturating_sub(since_ns)))
                        .collect(),
                    fetched_at,
                });
                self.state = ActivityState::Live {
                    armed_since_ms_ago: 0.0,
                };
            }
            FiresOutcome::UnknownRun => self.mark(ActivityState::UnknownRun),
            FiresOutcome::NotServed => self.mark(ActivityState::NotServed),
            FiresOutcome::Refused { reason } => self.mark(ActivityState::Refused {
                reason: reason.clone(),
            }),
        }
        self.cursor
    }

    /// Record a state the follower learned without a reply to fold (silence,
    /// an unusable reply, netd failing). Keeps the fires and the cursor, and
    /// drops the `in_tick` snapshot, which only a served page can refresh.
    pub fn mark(&mut self, state: ActivityState) {
        if let Some(served) = &mut self.served {
            served.in_tick.clear();
        }
        self.state = state;
    }

    /// The answer for a controller holding cursor `after`, at desk time `now`,
    /// with per-node statistics over the fires at most `window` old.
    pub fn answer(&self, after: Option<u64>, now: Instant, window: Duration) -> ActivityResponse {
        let state = match (&self.state, &self.served) {
            (ActivityState::Live { .. }, Some(served)) => ActivityState::Live {
                armed_since_ms_ago: ms(aged(served.armed_age_ns, served.fetched_at, now)),
            },
            (state, _) => state.clone(),
        };
        let asked = after.unwrap_or(0);
        let (next, lost, fires) = match (self.cursor, self.origin) {
            (Some(cursor), Some(origin)) if asked.max(origin) < cursor => {
                let floor = asked.max(origin);
                let fires: Vec<FireSeen> = self
                    .ring
                    .iter()
                    .rev()
                    .take_while(|held| held.record.seq > floor)
                    .take(MAX_ACTIVITY_FIRES)
                    .map(|held| seen(held, now))
                    .collect();
                (cursor, cursor - floor - fires.len() as u64, fires)
            }
            (Some(cursor), _) => (cursor.max(asked), 0, Vec::new()),
            (None, _) => (asked, 0, Vec::new()),
        };
        let in_tick: Vec<InTick> = self
            .served
            .iter()
            .flat_map(|served| {
                served.in_tick.iter().map(|&(node, age_ns)| InTick {
                    node,
                    ms: ms(aged(age_ns, served.fetched_at, now)),
                })
            })
            .collect();
        ActivityResponse {
            robot: self.robot.clone(),
            run_id: self.run_id.clone(),
            state,
            next,
            lost,
            window_ms: u32::try_from(window.as_millis()).unwrap_or(u32::MAX),
            nodes: self.nodes(&in_tick, now, window),
            in_tick,
            fires,
        }
    }

    /// One row per node with a held fire or inside a tick, ascending by node.
    fn nodes(&self, in_tick: &[InTick], now: Instant, window: Duration) -> Vec<NodeActivity> {
        let window_ns = u64::try_from(window.as_nanos()).unwrap_or(u64::MAX);
        let mut nodes: BTreeMap<u16, (Option<&Held>, Vec<&Held>)> = in_tick
            .iter()
            .map(|tick| (tick.node, (None, Vec::new())))
            .collect();
        for held in &self.ring {
            let (last, recent) = nodes.entry(held.record.node).or_default();
            *last = Some(held);
            if aged(held.age_at_fetch_ns, held.fetched_at, now) <= window_ns {
                recent.push(held);
            }
        }
        nodes
            .into_iter()
            .map(|(node, (last, recent))| {
                let mut durations: Vec<u32> = recent.iter().map(|h| h.record.dur_ns).collect();
                durations.sort_unstable();
                let count_flag = |flag: u8| {
                    recent
                        .iter()
                        .filter(|h| h.record.flags.0 & flag != 0)
                        .count() as u32
                };
                NodeActivity {
                    node,
                    fires: recent.len() as u32,
                    last_ms_ago: last.map(|h| ms(aged(h.age_at_fetch_ns, h.fetched_at, now))),
                    dur_p50_ns: durations
                        .get(durations.len().saturating_sub(1) / 2)
                        .copied(),
                    dur_max_ns: durations.last().copied(),
                    panics: count_flag(FireFlags::PANICKED),
                    errors: count_flag(FireFlags::TICK_ERROR),
                    last_cause: last.map(|h| h.record.cause as u8),
                    last_wrote: last.map(|h| h.record.wrote),
                    last_flags: last.map_or(0, |h| h.record.flags.0),
                }
            })
            .collect()
    }
}

/// An age the robot reported at `fetched_at`, carried forward to `now`.
fn aged(age_at_fetch_ns: u64, fetched_at: Instant, now: Instant) -> u64 {
    let since = now.saturating_duration_since(fetched_at).as_nanos();
    age_at_fetch_ns.saturating_add(u64::try_from(since).unwrap_or(u64::MAX))
}

fn ms(ns: u64) -> f64 {
    ns as f64 / 1e6
}

fn seen(held: &Held, now: Instant) -> FireSeen {
    let record = &held.record;
    FireSeen {
        seq: record.seq,
        node: record.node,
        cause: record.cause as u8,
        flags: record.flags.0,
        ms_ago: ms(aged(held.age_at_fetch_ns, held.fetched_at, now)),
        dur_ns: record.dur_ns,
        wrote: record.wrote,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::fire_tap::FireCause;

    const RUN: u128 = 0x5d1c_0000_0000_0000_0000_0000_000f_9e02;
    const RUN_TEXT: &str = "0x5d1c00000000000000000000000f9e02";
    const WINDOW: Duration = Duration::from_secs(5);
    const MS: u64 = 1_000_000;

    fn fire(seq: u64, at_ns: u64, dur_ns: u32, node: u16, cause: FireCause) -> FireRecord {
        FireRecord {
            seq,
            at_ns,
            dur_ns,
            node,
            cause,
            flags: FireFlags(0),
            wrote: 0,
        }
    }

    fn flagged(mut record: FireRecord, flags: u8, wrote: u64) -> FireRecord {
        record.flags = FireFlags(flags);
        record.wrote = wrote;
        record
    }

    fn served(
        now_ns: u64,
        armed_since_ns: u64,
        next: u64,
        lost: u64,
        in_tick: Vec<(u16, u64)>,
        fires: Vec<FireRecord>,
    ) -> FiresReply {
        FiresReply::new(
            "temi",
            RUN,
            FiresOutcome::Served {
                now_ns,
                armed_since_ns,
                next,
                lost,
                in_tick,
                fires,
            },
        )
    }

    fn outcome(outcome: FiresOutcome) -> FiresReply {
        FiresReply::new("temi", RUN, outcome)
    }

    fn seqs(answer: &ActivityResponse) -> Vec<u64> {
        answer.fires.iter().map(|f| f.seq).collect()
    }

    /// The robot's first answer to a cursor-less GET: nothing yet, `next` at
    /// the head. Its clock is the robot's (seconds since its boot), unrelated
    /// to the desk's.
    fn from_now(log: &mut ActivityLog, at: Instant) -> Option<u64> {
        log.absorb(
            &served(88_000 * MS, 87_900 * MS, 5119, 0, vec![], vec![]),
            at,
        )
    }

    #[test]
    fn a_new_log_is_starting_and_echoes_the_callers_cursor() {
        let log = ActivityLog::new("temi", RUN_TEXT);
        let now = Instant::now();
        let answer = log.answer(None, now, WINDOW);
        assert_eq!(
            answer,
            ActivityResponse {
                robot: "temi".into(),
                run_id: RUN_TEXT.into(),
                state: ActivityState::Starting,
                next: 0,
                lost: 0,
                window_ms: 5000,
                in_tick: vec![],
                nodes: vec![],
                fires: vec![],
            }
        );
        assert_eq!(log.answer(Some(41), now, WINDOW).next, 41);
    }

    #[test]
    fn fires_age_from_the_robots_serve_plus_desk_time_since() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        assert_eq!(from_now(&mut log, t0), Some(5119));
        let page = served(
            88_500 * MS,
            87_900 * MS,
            5121,
            0,
            vec![],
            vec![
                fire(5120, 88_100 * MS, 38_211, 0, FireCause::Period),
                fire(5121, 88_400 * MS, 412_003, 1, FireCause::Data),
            ],
        );
        let fetched = t0 + Duration::from_millis(250);
        assert_eq!(log.absorb(&page, fetched), Some(5121));

        let answer = log.answer(Some(5119), fetched + Duration::from_millis(250), WINDOW);
        assert_eq!(
            answer.state,
            ActivityState::Live {
                armed_since_ms_ago: 850.0
            }
        );
        assert_eq!(answer.next, 5121);
        assert_eq!(answer.lost, 0);
        assert_eq!(
            answer.fires,
            vec![
                FireSeen {
                    seq: 5121,
                    node: 1,
                    cause: 3,
                    flags: 0,
                    ms_ago: 350.0,
                    dur_ns: 412_003,
                    wrote: 0,
                },
                FireSeen {
                    seq: 5120,
                    node: 0,
                    cause: 1,
                    flags: 0,
                    ms_ago: 650.0,
                    dur_ns: 38_211,
                    wrote: 0,
                },
            ]
        );
        let later = log.answer(Some(5119), fetched + Duration::from_millis(1250), WINDOW);
        assert_eq!(later.fires[0].ms_ago, 1350.0);
        assert_eq!(
            later.state,
            ActivityState::Live {
                armed_since_ms_ago: 1850.0
            }
        );
        assert_eq!(
            log.answer(Some(5119), fetched + Duration::from_millis(250), WINDOW),
            answer,
            "the same log and instant answer the same bytes"
        );
    }

    #[test]
    fn a_fire_already_held_or_accounted_for_is_absorbed_once() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        from_now(&mut log, t0);
        let page = served(
            88_500 * MS,
            87_900 * MS,
            5123,
            0,
            vec![],
            (5120..=5123)
                .map(|seq| fire(seq, 88_400 * MS, 10, 0, FireCause::Period))
                .collect(),
        );
        assert_eq!(log.absorb(&page, t0), Some(5123));
        assert_eq!(log.absorb(&page, t0), Some(5123), "a repeated page");
        let overlap = served(
            88_600 * MS,
            87_900 * MS,
            5125,
            0,
            vec![],
            (5122..=5125)
                .map(|seq| fire(seq, 88_500 * MS, 10, 0, FireCause::Period))
                .collect(),
        );
        assert_eq!(log.absorb(&overlap, t0), Some(5125));
        let answer = log.answer(Some(5119), t0, WINDOW);
        assert_eq!(seqs(&answer), vec![5125, 5124, 5123, 5122, 5121, 5120]);
        assert_eq!(answer.lost, 0);
        assert_eq!(answer.nodes[0].fires, 6);

        let behind = served(88_700 * MS, 87_900 * MS, 5124, 0, vec![], vec![]);
        assert_eq!(
            log.absorb(&behind, t0),
            Some(5125),
            "a page behind the cursor never moves it back"
        );
    }

    #[test]
    fn the_robots_lost_fires_pass_through_to_the_answer() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        from_now(&mut log, t0);
        let page = served(
            88_500 * MS,
            87_900 * MS,
            5124,
            3,
            vec![],
            vec![
                fire(5120, 88_100 * MS, 10, 0, FireCause::Period),
                fire(5124, 88_400 * MS, 10, 0, FireCause::Period),
            ],
        );
        log.absorb(&page, t0);
        let answer = log.answer(Some(5119), t0, WINDOW);
        assert_eq!((seqs(&answer), answer.lost), (vec![5124, 5120], 3));
        let resumed = log.answer(Some(5120), t0, WINDOW);
        assert_eq!((seqs(&resumed), resumed.lost), (vec![5124], 3));
        let current = log.answer(Some(5124), t0, WINDOW);
        assert_eq!(
            (seqs(&current), current.lost, current.next),
            (vec![], 0, 5124)
        );
    }

    #[test]
    fn lost_is_counted_only_over_the_span_the_follower_followed() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        from_now(&mut log, t0);
        log.absorb(
            &served(
                88_500 * MS,
                87_900 * MS,
                5120,
                0,
                vec![],
                vec![fire(5120, 88_100 * MS, 10, 0, FireCause::Period)],
            ),
            t0,
        );
        for after in [None, Some(0), Some(100), Some(5119)] {
            let answer = log.answer(after, t0, WINDOW);
            assert_eq!((seqs(&answer), answer.lost), (vec![5120], 0), "{after:?}");
        }
        let ahead = log.answer(Some(9000), t0, WINDOW);
        assert_eq!(
            (seqs(&ahead), ahead.lost, ahead.next),
            (vec![], 0, 9000),
            "a cursor ahead of the log is never moved back"
        );
    }

    #[test]
    fn per_node_statistics_cover_the_window_and_last_covers_the_log() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        from_now(&mut log, t0);
        let now_ns = 100_000 * MS;
        let page = served(
            now_ns,
            90_000 * MS,
            5126,
            0,
            vec![],
            vec![
                flagged(
                    fire(5120, now_ns - 6000 * MS, 1000, 1, FireCause::Data),
                    FireFlags::TICK_ERROR,
                    0,
                ),
                flagged(
                    fire(5121, now_ns - 5500 * MS, 9000, 0, FireCause::Period),
                    FireFlags::TICK_ERROR,
                    0,
                ),
                fire(5122, now_ns - 4000 * MS, 30, 0, FireCause::Period),
                fire(5123, now_ns - 3000 * MS, 10, 0, FireCause::Period),
                flagged(
                    fire(5124, now_ns - 2000 * MS, 20, 0, FireCause::Period),
                    FireFlags::PANICKED,
                    0,
                ),
                flagged(
                    fire(5125, now_ns - 1000 * MS, 40, 0, FireCause::Period),
                    FireFlags::TICK_ERROR,
                    0b01,
                ),
                flagged(
                    fire(5126, now_ns - 500 * MS, 7, 2, FireCause::Sync),
                    FireFlags::WROTE_UNKNOWN,
                    0,
                ),
            ],
        );
        log.absorb(&page, t0);
        let answer = log.answer(None, t0, WINDOW);
        assert_eq!(
            answer.nodes,
            vec![
                NodeActivity {
                    node: 0,
                    fires: 4,
                    last_ms_ago: Some(1000.0),
                    dur_p50_ns: Some(20),
                    dur_max_ns: Some(40),
                    panics: 1,
                    errors: 1,
                    last_cause: Some(1),
                    last_wrote: Some(0b01),
                    last_flags: FireFlags::TICK_ERROR,
                },
                NodeActivity {
                    node: 1,
                    fires: 0,
                    last_ms_ago: Some(6000.0),
                    dur_p50_ns: None,
                    dur_max_ns: None,
                    panics: 0,
                    errors: 0,
                    last_cause: Some(3),
                    last_wrote: Some(0),
                    last_flags: FireFlags::TICK_ERROR,
                },
                NodeActivity {
                    node: 2,
                    fires: 1,
                    last_ms_ago: Some(500.0),
                    dur_p50_ns: Some(7),
                    dur_max_ns: Some(7),
                    panics: 0,
                    errors: 0,
                    last_cause: Some(4),
                    last_wrote: Some(0),
                    last_flags: FireFlags::WROTE_UNKNOWN,
                },
            ]
        );
        let aged = log.answer(None, t0 + Duration::from_millis(2500), WINDOW);
        assert_eq!(
            aged.nodes[0].fires, 2,
            "the window moves with desk time: 5122 and 5123 aged out"
        );
        assert_eq!(aged.nodes[0].dur_p50_ns, Some(20));
        assert_eq!(aged.nodes[0].panics, 1);
    }

    #[test]
    fn in_tick_ages_with_desk_time_and_lists_a_node_with_no_fire() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        log.absorb(
            &served(
                88_500 * MS,
                87_900 * MS,
                5119,
                0,
                vec![(2, 88_400 * MS)],
                vec![],
            ),
            t0,
        );
        let answer = log.answer(None, t0 + Duration::from_millis(250), WINDOW);
        assert_eq!(answer.in_tick, vec![InTick { node: 2, ms: 350.0 }]);
        assert_eq!(
            answer.nodes,
            vec![NodeActivity {
                node: 2,
                fires: 0,
                last_ms_ago: None,
                dur_p50_ns: None,
                dur_max_ns: None,
                panics: 0,
                errors: 0,
                last_cause: None,
                last_wrote: None,
                last_flags: 0,
            }]
        );
        log.absorb(
            &served(88_750 * MS, 87_900 * MS, 5119, 0, vec![], vec![]),
            t0,
        );
        assert_eq!(log.answer(None, t0, WINDOW).in_tick, vec![]);
        assert_eq!(log.answer(None, t0, WINDOW).nodes, vec![]);
    }

    #[test]
    fn the_log_holds_at_most_its_capacity_and_an_answer_its_row_cap() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        assert_eq!(from_now(&mut log, t0), Some(5119));
        let total: u64 = 2100;
        let mut first = 5120;
        while first < 5120 + total {
            let last = (first + 511).min(5119 + total);
            let page = served(
                88_500 * MS,
                87_900 * MS,
                last,
                0,
                vec![],
                (first..=last)
                    .map(|seq| fire(seq, 88_400 * MS, 10, 0, FireCause::Period))
                    .collect(),
            );
            assert_eq!(log.absorb(&page, t0), Some(last));
            first = last + 1;
        }
        let answer = log.answer(Some(5119), t0, WINDOW);
        assert_eq!(answer.next, 5119 + total);
        assert_eq!(answer.fires.len(), MAX_ACTIVITY_FIRES);
        assert_eq!(answer.fires[0].seq, 5119 + total);
        assert_eq!(
            answer.fires[MAX_ACTIVITY_FIRES - 1].seq,
            5119 + total - MAX_ACTIVITY_FIRES as u64 + 1
        );
        assert_eq!(answer.lost, total - MAX_ACTIVITY_FIRES as u64);
        assert_eq!(answer.nodes[0].fires, ACTIVITY_LOG_CAPACITY as u32);
    }

    #[test]
    fn a_non_served_outcome_sets_its_state_and_keeps_fires_and_cursor() {
        let t0 = Instant::now();
        let mut log = ActivityLog::new("temi", RUN_TEXT);
        from_now(&mut log, t0);
        log.absorb(
            &served(
                88_500 * MS,
                87_900 * MS,
                5120,
                0,
                vec![(0, 88_400 * MS)],
                vec![fire(5120, 88_100 * MS, 10, 0, FireCause::Period)],
            ),
            t0,
        );
        let cases = [
            (FiresOutcome::UnknownRun, ActivityState::UnknownRun),
            (FiresOutcome::NotServed, ActivityState::NotServed),
            (
                FiresOutcome::Refused {
                    reason: "paired".into(),
                },
                ActivityState::Refused {
                    reason: "paired".into(),
                },
            ),
        ];
        for (wire, state) in cases {
            assert_eq!(log.absorb(&outcome(wire), t0), Some(5120));
            let answer = log.answer(Some(5119), t0, WINDOW);
            assert_eq!(answer.state, state);
            assert_eq!((seqs(&answer), answer.next), (vec![5120], 5120));
        }
        for state in [
            ActivityState::Silent,
            ActivityState::Unusable {
                reason: "skew".into(),
            },
            ActivityState::Degraded {
                reason: "netd down".into(),
            },
        ] {
            log.mark(state.clone());
            let answer = log.answer(Some(5119), t0, WINDOW);
            assert_eq!(answer.state, state);
            assert_eq!((seqs(&answer), answer.next), (vec![5120], 5120));
            assert_eq!(answer.in_tick, vec![], "only a served page knows in_tick");
        }
        assert_eq!(from_now(&mut log, t0), Some(5120));
        assert!(matches!(
            log.answer(None, t0, WINDOW).state,
            ActivityState::Live { .. }
        ));
    }
}

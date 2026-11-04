// SPDX-License-Identifier: AGPL-3.0-only
//! The replay gate: what each `rmw_wait` reports and which thread may run.
//!
//! # Serving units
//!
//! A recorded window (one `rmw_wait` return) is served as one or more UNITS,
//! one unit per replayed `rmw_wait` call:
//!
//! - a timeout, an error or an empty ready set is one unit with that return;
//! - otherwise each entity the executor TOOK from after that wait, in the
//!   recorded take order, is its own unit; ready entities that were never
//!   taken from (guard conditions, timers' guards) ride the LAST unit, after
//!   the takes, because both rclcpp and rclpy order guard-condition and
//!   waitable callbacks after subscriptions;
//! - a window in which a `/clock` take precedes a take from another entity is
//!   served as its recorded SET, in one unit. Splitting it would put an extra
//!   rcl timer check after the time jump that live rcl never made (rcl decides
//!   a ROS-time timer's readiness from the clock at `rcl_wait`, not from the
//!   rmw's report), and rclpy would then run a timer callback the live run
//!   ran one window later.
//!
//! Units are ordered by the order key of their take (a unit without a take by
//! its window's key), which is the order the recorded callbacks ran in, also
//! across threads.
//!
//! # The process token
//!
//! A thread that is served a unit holds the token until it calls `rmw_wait`
//! again, so the callbacks that unit releases run before any other thread is
//! served. A waiting thread is served only when the token is free and the head
//! unit is its own. With one executor thread this costs nothing; with several
//! single-threaded executors on several threads (the rclcpp clock thread, a
//! tf2 listener) it makes their callbacks interleave in the recorded order.
//!
//! # Takes in order across threads
//!
//! A served take also waits for its turn: it runs once every unit and every
//! other thread's waiting take with a smaller order key is served and the
//! token is free, and then holds the token until its thread's next rmw call.
//! So a thread's window that the recording returned between another thread's
//! wait return and that thread's take is served in between, as recorded. This
//! is what keeps an rclcpp clock thread's `/clock` take (and the ROS time its
//! callback sets) on the recorded side of the executor thread's rcl timer
//! check, which runs right after that thread's `rmw_wait` returns, while it
//! holds the token.
//!
//! # The watchdog
//!
//! A thread that has waited [`WATCHDOG`] for its turn with no rmw progress
//! anywhere in the process reports who holds the token: a callback is blocked
//! on something the replay cannot serve. The watchdog names the holder; it
//! cannot rescue the run.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

use super::log::{EntityId, GraphQuery, GraphRec, TakeKind, TakeRec, WaitRec};
use crate::ffi::RMW_RET_OK;

/// How long a waiting thread waits with no rmw progress before it reports the
/// token holder.
pub(crate) const WATCHDOG: Duration = Duration::from_secs(5);

/// One served `rmw_wait` return.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Unit {
    /// Ordering key (the take's `k`, or the window's for a unit without one).
    pub key: u64,
    /// The recorded thread the unit belongs to.
    pub thread: u32,
    /// Index of the unit's window in [`Schedule::windows`].
    pub window: usize,
    /// The first unit of its window (the window's wait return opens a
    /// segment for the oracle; later units continue it).
    pub first: bool,
    pub ret: i32,
    pub ready: Vec<EntityId>,
}

/// The whole recorded wait schedule of a process.
#[derive(Debug, Default)]
pub(crate) struct Schedule {
    pub units: Vec<Unit>,
    /// Every recorded window, in `k` order.
    pub windows: Vec<WaitRec>,
    /// Windows served as their recorded set because a `/clock` take preceded
    /// another take.
    pub set_served_windows: usize,
    /// Windows served as more than one unit.
    pub split_windows: usize,
}

impl Schedule {
    /// Build the schedule from the recorded waits and takes (each in `k`
    /// order). `clock_subs` are the `/clock` subscriptions.
    pub(crate) fn build(
        waits: &[WaitRec],
        takes: &[TakeRec],
        clock_subs: &HashSet<EntityId>,
    ) -> Schedule {
        let mut s = Schedule {
            windows: waits.to_vec(),
            ..Schedule::default()
        };
        let mut takes_by_thread: HashMap<u32, Vec<&TakeRec>> = HashMap::new();
        for t in takes {
            takes_by_thread.entry(t.thread).or_default().push(t);
        }
        let mut windows_by_thread: HashMap<u32, Vec<usize>> = HashMap::new();
        for (i, w) in s.windows.iter().enumerate() {
            windows_by_thread.entry(w.thread).or_default().push(i);
        }
        let mut units = Vec::new();
        for (thread, idxs) in &windows_by_thread {
            let thread_takes = takes_by_thread
                .get(thread)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            let mut tp = 0usize;
            for (n, &wi) in idxs.iter().enumerate() {
                let w = &s.windows[wi];
                let end_k = idxs
                    .get(n + 1)
                    .map(|&next| s.windows[next].k)
                    .unwrap_or(u64::MAX);
                while tp < thread_takes.len() && thread_takes[tp].k < w.k {
                    tp += 1;
                }
                let start = tp;
                while tp < thread_takes.len() && thread_takes[tp].k < end_k {
                    tp += 1;
                }
                let in_window = &thread_takes[start..tp];
                let mut push = |key: u64, first: bool, ready: Vec<EntityId>, ret: i32| {
                    units.push(Unit {
                        key,
                        thread: *thread,
                        window: wi,
                        first,
                        ret,
                        ready,
                    })
                };
                if w.ret != RMW_RET_OK || w.ready.is_empty() {
                    push(w.k, true, w.ready.clone(), w.ret);
                    continue;
                }
                let clock_split_hazard = in_window.iter().enumerate().any(|(i, t)| {
                    t.taken
                        && clock_subs.contains(&t.entity)
                        && in_window[i + 1..]
                            .iter()
                            .any(|later| later.entity != t.entity)
                });
                if clock_split_hazard {
                    s.set_served_windows += 1;
                    push(w.k, true, w.ready.clone(), w.ret);
                    continue;
                }
                // The distinct ready entities taken from, in take order.
                let mut order: Vec<(EntityId, u64)> = Vec::new();
                for t in in_window {
                    if w.ready.contains(&t.entity) && !order.iter().any(|(e, _)| *e == t.entity) {
                        order.push((t.entity, t.k));
                    }
                }
                if order.len() <= 1 {
                    push(w.k, true, w.ready.clone(), w.ret);
                    continue;
                }
                s.split_windows += 1;
                let rest: Vec<EntityId> = w
                    .ready
                    .iter()
                    .copied()
                    .filter(|e| !order.iter().any(|(o, _)| o == e))
                    .collect();
                let last = order.len() - 1;
                for (i, (entity, take_k)) in order.iter().enumerate() {
                    let mut ready = vec![*entity];
                    if i == last {
                        ready.extend_from_slice(&rest);
                    }
                    // The first unit keeps the window's key so its wait return
                    // orders before anything its take precedes.
                    let key = if i == 0 { w.k } else { *take_k };
                    push(key, i == 0, ready, w.ret);
                }
            }
        }
        units.sort_by_key(|u| u.key);
        s.units = units;
        s
    }
}

/// What a waiting thread is told.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Turn {
    /// Serve the unit at this index of [`Schedule::units`].
    Serve(usize),
    /// This thread has no recorded window left (others may).
    NoMore,
    /// The recorded schedule is fully served and no thread holds the token
    /// or waits for a take's turn: every served unit's callbacks are done.
    Exhausted,
}

/// The token holder at a watchdog report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Stall {
    /// The recorded thread holding the token, or the owner of the head unit
    /// when no thread holds it.
    pub holder_thread: u32,
    /// The unit the holder was last served (or the head unit).
    pub unit: usize,
    pub waiting_thread: u32,
}

struct TokenState {
    next: usize,
    holder: Option<(u32, usize)>,
    remaining: HashMap<u32, usize>,
    /// Order keys of the served takes waiting for their turn.
    waiting_takes: std::collections::BTreeSet<u64>,
    /// The schedule is used up and the last holder stalled (reported once).
    stalled_at_end: bool,
    /// The schedule is used up and a thread still holds the token: since
    /// when, and the work counter then.
    end_watch: Option<(Instant, u64)>,
}

/// The token and the unit cursor.
pub(crate) struct Gate {
    units_thread: Vec<u32>,
    units_key: Vec<u64>,
    state: Mutex<TokenState>,
    cv: Condvar,
    /// Bumped on every rmw event of the replaying process: the watchdog's
    /// "no progress" is this counter standing still.
    progress: AtomicU64,
    /// Bumped on every take, output and graph query (not on waits): once the
    /// schedule is used up, idle threads keep waiting, so the last holder's
    /// stall is this counter standing still.
    work: AtomicU64,
}

impl Gate {
    pub(crate) fn new(schedule: &Schedule) -> Gate {
        let mut remaining: HashMap<u32, usize> = HashMap::new();
        for u in &schedule.units {
            *remaining.entry(u.thread).or_default() += 1;
        }
        Gate {
            units_thread: schedule.units.iter().map(|u| u.thread).collect(),
            units_key: schedule.units.iter().map(|u| u.key).collect(),
            state: Mutex::new(TokenState {
                next: 0,
                holder: None,
                remaining,
                waiting_takes: std::collections::BTreeSet::new(),
                stalled_at_end: false,
                end_watch: None,
            }),
            cv: Condvar::new(),
            progress: AtomicU64::new(0),
            work: AtomicU64::new(0),
        }
    }

    /// Record rmw progress (a wait).
    pub(crate) fn note_progress(&self) {
        self.progress.fetch_add(1, Ordering::Relaxed);
    }

    /// Record rmw work (a take, an output or a graph query): progress too.
    pub(crate) fn note_work(&self) {
        self.work.fetch_add(1, Ordering::Relaxed);
        self.progress.fetch_add(1, Ordering::Relaxed);
    }

    /// The number of units served so far.
    pub(crate) fn served(&self) -> usize {
        self.state.lock().unwrap_or_else(|e| e.into_inner()).next
    }

    /// Block until the served take with order key `k` on `thread` may run
    /// (see "Takes in order across threads"), then hold the token. Releases
    /// the token first if `thread` holds it. `on_stall` is called once per
    /// stall that lasts [`WATCHDOG`] with no progress; the take then runs.
    pub(crate) fn take_turn(&self, thread: u32, k: u64, mut on_stall: impl FnMut(Stall)) {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(st.holder, Some((h, _)) if h == thread) {
            st.holder = None;
            self.cv.notify_all();
        }
        st.waiting_takes.insert(k);
        let mut last_progress = self.progress.load(Ordering::Relaxed);
        let mut since = Instant::now();
        loop {
            let units_before = st.next < self.units_key.len() && self.units_key[st.next] < k;
            let takes_before = st.waiting_takes.first().is_some_and(|&first| first < k);
            if st.holder.is_none() && !units_before && !takes_before {
                break;
            }
            let (guard, _) = self
                .cv
                .wait_timeout(st, Duration::from_millis(100))
                .unwrap_or_else(|e| e.into_inner());
            st = guard;
            let now_progress = self.progress.load(Ordering::Relaxed);
            if now_progress != last_progress {
                last_progress = now_progress;
                since = Instant::now();
            } else if since.elapsed() >= WATCHDOG {
                let (holder_thread, unit) = match st.holder {
                    Some(h) => h,
                    None => (
                        self.units_thread.get(st.next).copied().unwrap_or(thread),
                        st.next,
                    ),
                };
                st.waiting_takes.remove(&k);
                self.cv.notify_all();
                drop(st);
                on_stall(Stall {
                    holder_thread,
                    unit,
                    waiting_thread: thread,
                });
                return;
            }
        }
        st.waiting_takes.remove(&k);
        st.holder = Some((thread, st.next.saturating_sub(1)));
        self.progress.fetch_add(1, Ordering::Relaxed);
        self.cv.notify_all();
    }

    /// Undo the [`Turn::Serve`] of unit `idx` to `thread` (its wait could not
    /// take it, see `Player::wait`): the unit is the next one again, still
    /// `thread`'s, and the token is free.
    pub(crate) fn give_back(&self, thread: u32, idx: usize) {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        debug_assert_eq!(st.next, idx + 1);
        debug_assert_eq!(st.holder, Some((thread, idx)));
        if st.next != idx + 1 || st.holder != Some((thread, idx)) {
            return;
        }
        st.next = idx;
        st.holder = None;
        *st.remaining.entry(thread).or_default() += 1;
        self.cv.notify_all();
    }

    /// Block until it is `thread`'s turn. Releases the token if `thread`
    /// holds it (it is back in `rmw_wait`). `on_stall` is called once per
    /// stall that lasts [`WATCHDOG`] with no progress.
    pub(crate) fn wait_turn(&self, thread: u32, mut on_stall: impl FnMut(Stall)) -> Turn {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(st.holder, Some((h, _)) if h == thread) {
            st.holder = None;
            self.cv.notify_all();
        }
        let mut last_progress = self.progress.load(Ordering::Relaxed);
        let mut since = Instant::now();
        let mut reported = false;
        loop {
            if st.next >= self.units_thread.len() {
                if st.holder.is_none() && st.waiting_takes.is_empty() {
                    return Turn::Exhausted;
                }
                // Another thread is inside the callbacks of the last units:
                // their outputs belong to recorded segments still open, so
                // exhaustion (which closes every segment) waits until the
                // holder is back in rmw_wait (or the process ends). This
                // thread idles meanwhile (`NoMore`): a thread that must leave
                // its executor, such as rclcpp's clock thread at shutdown, is
                // not held. A holder that does no rmw work (take, output or
                // graph query) for [`WATCHDOG`] is reported once as a stall;
                // its open segment is never compared half done.
                if !st.stalled_at_end {
                    let work = self.work.load(Ordering::Relaxed);
                    match st.end_watch {
                        Some((at, w)) if w == work => {
                            if at.elapsed() >= WATCHDOG {
                                st.stalled_at_end = true;
                                let last = self.units_thread.len().saturating_sub(1);
                                let (holder_thread, unit) = st.holder.unwrap_or((
                                    self.units_thread.get(last).copied().unwrap_or(thread),
                                    last,
                                ));
                                drop(st);
                                on_stall(Stall {
                                    holder_thread,
                                    unit,
                                    waiting_thread: thread,
                                });
                                return Turn::NoMore;
                            }
                        }
                        _ => st.end_watch = Some((Instant::now(), work)),
                    }
                }
                return Turn::NoMore;
            }
            if st.remaining.get(&thread).copied().unwrap_or(0) == 0 {
                return Turn::NoMore;
            }
            let take_before = st
                .waiting_takes
                .first()
                .is_some_and(|&t| t < self.units_key[st.next]);
            if st.holder.is_none() && self.units_thread[st.next] == thread && !take_before {
                let idx = st.next;
                st.next += 1;
                st.holder = Some((thread, idx));
                if let Some(r) = st.remaining.get_mut(&thread) {
                    *r -= 1;
                }
                self.progress.fetch_add(1, Ordering::Relaxed);
                self.cv.notify_all();
                return Turn::Serve(idx);
            }
            let (guard, _) = self
                .cv
                .wait_timeout(st, Duration::from_millis(100))
                .unwrap_or_else(|e| e.into_inner());
            st = guard;
            let now_progress = self.progress.load(Ordering::Relaxed);
            if now_progress != last_progress {
                last_progress = now_progress;
                since = Instant::now();
                reported = false;
            } else if !reported && since.elapsed() >= WATCHDOG && st.next < self.units_thread.len()
            {
                reported = true;
                let (holder_thread, unit) = match st.holder {
                    Some(h) => h,
                    None => (self.units_thread[st.next], st.next),
                };
                let stall = Stall {
                    holder_thread,
                    unit,
                    waiting_thread: thread,
                };
                // Report with the lock released: the caller logs and writes
                // the verdict.
                drop(st);
                on_stall(stall);
                st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            }
        }
    }
}

/// The recorded takes, served per entity, with each thread's recorded take
/// order kept to check the replayed order against.
#[derive(Default)]
pub(crate) struct TakeQueues {
    per_entity: HashMap<(EntityId, TakeKind), VecDeque<TakeRec>>,
    /// Each recorded thread's takes not yet served: (entity, kind, order key).
    per_thread: HashMap<u32, VecDeque<(EntityId, TakeKind, u64)>>,
    /// The log was torn (see [`TakeQueues::new`]).
    torn: bool,
}

/// A take served from the log.
#[derive(Debug)]
pub(crate) struct Served {
    /// The recorded take, or `None` past the end of the entity's recording.
    pub rec: Option<TakeRec>,
    /// Set when the replayed take differs from the recording: the replayed
    /// process took from an entity in an order the live run did not.
    pub mismatch: Option<String>,
    /// A take past this thread's last recorded take in a torn log: the
    /// recording may have made it after its last surviving record. Served as
    /// not taken; not a mismatch.
    pub past_end: bool,
}

impl TakeQueues {
    /// `torn`: the log did not close cleanly, so a take past a thread's last
    /// recorded take may be one the recording made after the log's end.
    pub(crate) fn new(takes: &[TakeRec], torn: bool) -> TakeQueues {
        let mut q = TakeQueues {
            torn,
            ..TakeQueues::default()
        };
        for t in takes {
            q.per_entity
                .entry((t.entity, t.kind))
                .or_default()
                .push_back(t.clone());
            q.per_thread
                .entry(t.thread)
                .or_default()
                .push_back((t.entity, t.kind, t.k));
        }
        q
    }

    /// The recorded take with the smallest order key that the replay has not
    /// made, as (thread, entity, order key).
    #[cfg(test)]
    pub(crate) fn first_unserved(&self) -> Option<(u32, EntityId, u64)> {
        self.first_unserved_where(|_, _| true)
    }

    /// [`TakeQueues::first_unserved`] among the takes `keep(thread, k)` accepts.
    pub(crate) fn first_unserved_where(
        &self,
        keep: impl Fn(u32, u64) -> bool,
    ) -> Option<(u32, EntityId, u64)> {
        self.per_thread
            .iter()
            .flat_map(|(t, q)| q.iter().map(move |&(e, _, k)| (*t, e, k)))
            .filter(|&(t, _, k)| keep(t, k))
            .min_by_key(|&(_, _, k)| k)
    }

    /// The number of recorded takes the replay has not made.
    pub(crate) fn unserved(&self) -> usize {
        self.per_thread.values().map(VecDeque::len).sum()
    }

    /// Serve the next recorded take of `entity` to the recorded thread
    /// `thread` (`None` when the replaying thread maps to no recorded one).
    pub(crate) fn serve(
        &mut self,
        thread: Option<u32>,
        entity: EntityId,
        kind: TakeKind,
        entity_name: &str,
    ) -> Served {
        let mut mismatch = None;
        let order = match thread {
            Some(t) if self.torn => Some(self.per_thread.entry(t).or_default()),
            Some(t) => self.per_thread.get_mut(&t),
            None => None,
        };
        match order {
            Some(order) => match order.front() {
                Some(&(e, k, _)) if (e, k) == (entity, kind) => {
                    order.pop_front();
                }
                Some(_) => {
                    mismatch = Some(format!(
                        "took from {entity_name} where the recording took from another entity \
                         next on this thread"
                    ));
                    if let Some(pos) = order.iter().position(|&(e, k, _)| (e, k) == (entity, kind))
                    {
                        order.remove(pos);
                    }
                }
                None if self.torn => {
                    return Served {
                        rec: None,
                        mismatch: None,
                        past_end: true,
                    };
                }
                None => {
                    mismatch = Some(format!(
                        "took from {entity_name} after this thread's last recorded take"
                    ));
                }
            },
            None => {
                mismatch = Some(format!(
                    "took from {entity_name} on a thread the recording does not have"
                ));
            }
        }
        let rec = self
            .per_entity
            .get_mut(&(entity, kind))
            .and_then(VecDeque::pop_front);
        if rec.is_none() && mismatch.is_none() {
            mismatch = Some(format!("took from {entity_name} beyond its recording"));
        }
        Served {
            rec,
            mismatch,
            past_end: false,
        }
    }
}

/// The recorded graph-query answers, served per (query, name) in order.
#[derive(Default)]
pub(crate) struct GraphAnswers {
    queues: HashMap<(GraphQuery, String), VecDeque<GraphRec>>,
}

impl GraphAnswers {
    pub(crate) fn new(recs: &[GraphRec]) -> GraphAnswers {
        let mut g = GraphAnswers::default();
        for r in recs {
            g.queues
                .entry((r.query, r.name.clone()))
                .or_default()
                .push_back(r.clone());
        }
        g
    }

    /// The next recorded answer, or `None` when the recording has no more.
    pub(crate) fn next(&mut self, query: GraphQuery, name: &str) -> Option<GraphRec> {
        self.queues
            .get_mut(&(query, name.to_string()))
            .and_then(VecDeque::pop_front)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi::RMW_RET_TIMEOUT;
    use std::sync::Arc;

    fn wait(k: u64, thread: u32, ret: i32, ready: &[EntityId]) -> WaitRec {
        WaitRec {
            k,
            thread,
            ret,
            timeout_ns: -1,
            attached: vec![0, 1, 2, 3, 9],
            ready: ready.to_vec(),
        }
    }

    fn take(k: u64, thread: u32, entity: EntityId) -> TakeRec {
        TakeRec {
            k,
            thread,
            entity,
            kind: TakeKind::Message,
            ret: 0,
            taken: true,
            seq: k as i64,
            source_ts: 0,
            received_ts: 0,
            guid: [0; 16],
            body: vec![k as u8],
        }
    }

    const CLOCK: EntityId = 0;
    const A: EntityId = 1;
    const B: EntityId = 2;
    const GUARD: EntityId = 9;

    fn clock_set() -> HashSet<EntityId> {
        [CLOCK].into_iter().collect()
    }

    #[test]
    fn a_window_is_split_in_take_order_with_untaken_guards_on_the_last_unit() {
        let waits = [wait(10, 0, RMW_RET_OK, &[A, B, GUARD])];
        // The executor took B first, then A.
        let takes = [take(11, 0, B), take(12, 0, A)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        let readies: Vec<_> = s.units.iter().map(|u| u.ready.clone()).collect();
        assert_eq!(readies, vec![vec![B], vec![A, GUARD]]);
        assert!(s.units[0].first && !s.units[1].first);
        assert_eq!(s.split_windows, 1);
        assert_eq!(s.set_served_windows, 0);
    }

    #[test]
    fn a_clock_take_before_another_take_serves_the_window_as_its_set() {
        let waits = [wait(10, 0, RMW_RET_OK, &[CLOCK, A])];
        let takes = [take(11, 0, CLOCK), take(12, 0, A)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        assert_eq!(s.units.len(), 1);
        assert_eq!(s.units[0].ready, vec![CLOCK, A]);
        assert_eq!(s.set_served_windows, 1);
        // A clock take that comes LAST is split like any other window.
        let takes = [take(11, 0, A), take(12, 0, CLOCK)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        assert_eq!(s.units.len(), 2);
        assert_eq!(s.set_served_windows, 0);
    }

    #[test]
    fn timeouts_and_empty_windows_are_one_unit_with_their_return() {
        let waits = [
            wait(10, 0, RMW_RET_TIMEOUT, &[]),
            wait(20, 0, RMW_RET_OK, &[GUARD]),
        ];
        let s = Schedule::build(&waits, &[], &clock_set());
        assert_eq!(s.units.len(), 2);
        assert_eq!(s.units[0].ret, RMW_RET_TIMEOUT);
        assert_eq!(s.units[1].ready, vec![GUARD]);
    }

    #[test]
    fn units_of_two_threads_interleave_in_recorded_take_order() {
        // Thread 0's window at 10 takes A (11) and B (15); thread 1's window
        // at 12 takes the clock (13). The callbacks ran A, clock, B.
        let waits = [
            wait(10, 0, RMW_RET_OK, &[A, B]),
            wait(12, 1, RMW_RET_OK, &[CLOCK]),
        ];
        let takes = [take(11, 0, A), take(13, 1, CLOCK), take(15, 0, B)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        let order: Vec<(u32, Vec<EntityId>)> = s
            .units
            .iter()
            .map(|u| (u.thread, u.ready.clone()))
            .collect();
        assert_eq!(order, vec![(0, vec![A]), (1, vec![CLOCK]), (0, vec![B])]);
    }

    #[test]
    fn the_token_serves_two_threads_in_schedule_order() {
        let waits = [
            wait(10, 0, RMW_RET_OK, &[A, B]),
            wait(12, 1, RMW_RET_OK, &[CLOCK]),
        ];
        let takes = [take(11, 0, A), take(13, 1, CLOCK), take(15, 0, B)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        let gate = Arc::new(Gate::new(&s));
        let log = Arc::new(Mutex::new(Vec::new()));
        let spawn = |thread: u32| {
            let gate = Arc::clone(&gate);
            let log = Arc::clone(&log);
            std::thread::spawn(move || loop {
                match gate.wait_turn(thread, |_| panic!("no stall expected")) {
                    Turn::Serve(i) => log.lock().unwrap().push(i),
                    Turn::NoMore | Turn::Exhausted => {
                        // Release the token by calling once more.
                        return;
                    }
                }
            })
        };
        // Start thread 1 first: it must still wait for thread 0's unit.
        let t1 = spawn(1);
        std::thread::sleep(Duration::from_millis(50));
        let t0 = spawn(0);
        t0.join().unwrap();
        t1.join().unwrap();
        assert_eq!(*log.lock().unwrap(), vec![0, 1, 2]);
        assert_eq!(gate.served(), 3);
    }

    #[test]
    fn a_take_waits_for_another_threads_window_recorded_before_it() {
        // The rclcpp clock thread (1) returns from its wait at k 10 and takes
        // /clock at k 12; the executor thread (0) returned from its wait at
        // k 11 in between, and checked its timers before the time moved.
        let waits = [
            wait(10, 1, RMW_RET_OK, &[CLOCK]),
            wait(11, 0, RMW_RET_OK, &[A]),
        ];
        let takes = [take(12, 1, CLOCK), take(13, 0, A)];
        let s = Schedule::build(&waits, &takes, &clock_set());
        let gate = Arc::new(Gate::new(&s));
        let order = Arc::new(Mutex::new(Vec::new()));
        // Thread 1 is served its window and goes to take at once.
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Serve(0));
        let (g, o) = (Arc::clone(&gate), Arc::clone(&order));
        let clock = std::thread::spawn(move || {
            g.take_turn(1, 12, |_| panic!("no stall"));
            o.lock().unwrap().push("take clock k12");
            // Back in rmw_wait: the token passes on. Thread 0 may hold it by
            // now (its take at k 13), so this thread idles or sees the end.
            let t = g.wait_turn(1, |_| panic!("no stall"));
            assert!(matches!(t, Turn::NoMore | Turn::Exhausted), "{t:?}");
        });
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            order.lock().unwrap().is_empty(),
            "the /clock take waits for thread 0's window at k 11"
        );
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Serve(1));
        order.lock().unwrap().push("serve window k11");
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(
            order.lock().unwrap().len(),
            1,
            "thread 0 holds the token until its next rmw call (its timer check)"
        );
        // Thread 0's take at k 13 waits for the clock take at k 12.
        gate.take_turn(0, 13, |_| panic!("no stall"));
        order.lock().unwrap().push("take a k13");
        clock.join().unwrap();
        assert_eq!(
            *order.lock().unwrap(),
            vec!["serve window k11", "take clock k12", "take a k13"]
        );
        // Thread 0 back in rmw_wait: the schedule is used up.
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Exhausted);
    }

    #[test]
    fn a_thread_holding_the_token_without_progress_trips_the_watchdog() {
        let waits = [wait(10, 0, RMW_RET_OK, &[A]), wait(20, 1, RMW_RET_OK, &[B])];
        let s = Schedule::build(&waits, &[take(11, 0, A), take(21, 1, B)], &clock_set());
        let gate = Arc::new(Gate::new(&s));
        // Thread 0 is served its unit and does not come back (a blocked
        // callback): it holds the token.
        assert_eq!(gate.wait_turn(0, |_| {}), Turn::Serve(0));
        let started = Instant::now();
        let (tx, rx) = std::sync::mpsc::channel();
        let g = Arc::clone(&gate);
        let waiter = std::thread::spawn(move || {
            g.wait_turn(1, move |stall| {
                let _ = tx.send(stall);
            })
        });
        let stall = rx
            .recv_timeout(WATCHDOG * 3)
            .expect("the watchdog reported the stall");
        assert!(started.elapsed() >= WATCHDOG);
        assert_eq!(
            stall,
            Stall {
                holder_thread: 0,
                unit: 0,
                waiting_thread: 1
            }
        );
        // Thread 0 comes back: the token passes and thread 1 is served.
        assert_eq!(gate.wait_turn(0, |_| {}), Turn::NoMore);
        assert_eq!(waiter.join().unwrap(), Turn::Serve(1));
    }

    #[test]
    fn a_unit_given_back_is_served_again_to_its_thread_and_keeps_the_order() {
        let waits = [wait(10, 1, RMW_RET_OK, &[B]), wait(12, 0, RMW_RET_OK, &[A])];
        let s = Schedule::build(&waits, &[take(11, 1, B), take(13, 0, A)], &clock_set());
        let gate = Gate::new(&s);
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Serve(0));
        // Thread 1's wait could not take unit 0 (a stale wait): given back,
        // the token is free and unit 0 is still the next one, so thread 0
        // is not served unit 1 ahead of it.
        gate.give_back(1, 0);
        assert_eq!(gate.served(), 0);
        let g = std::sync::Arc::new(gate);
        let g0 = g.clone();
        let t0 = std::thread::spawn(move || g0.wait_turn(0, |_| panic!("no stall")));
        std::thread::sleep(Duration::from_millis(150));
        assert!(
            !t0.is_finished(),
            "thread 0 waits for unit 0 to be served first"
        );
        assert_eq!(g.wait_turn(1, |_| panic!("no stall")), Turn::Serve(0));
        assert_eq!(g.wait_turn(1, |_| panic!("no stall")), Turn::NoMore);
        assert_eq!(t0.join().unwrap(), Turn::Serve(1));
        assert_eq!(g.wait_turn(0, |_| panic!("no stall")), Turn::Exhausted);
    }

    #[test]
    fn exhaustion_waits_until_the_last_holder_is_back_in_rmw_wait() {
        let waits = [wait(10, 0, RMW_RET_OK, &[A]), wait(12, 1, RMW_RET_OK, &[B])];
        let s = Schedule::build(&waits, &[take(11, 0, A), take(13, 1, B)], &clock_set());
        let gate = Gate::new(&s);
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Serve(0));
        // Thread 0 is back in rmw_wait with nothing left of its own.
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::NoMore);
        // Thread 1 is served the last unit and holds the token inside its
        // callback.
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Serve(1));
        // Thread 0 idles, never told Exhausted while thread 1 holds the
        // token, and is not held either (it returns at once).
        for _ in 0..3 {
            let t = Instant::now();
            assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::NoMore);
            assert!(t.elapsed() < Duration::from_secs(1));
        }
        // Thread 1 is back in rmw_wait: the schedule is exhausted for both.
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Exhausted);
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Exhausted);
    }

    #[test]
    fn a_last_holder_that_stalls_is_reported_and_its_segment_is_never_closed() {
        let waits = [wait(10, 0, RMW_RET_OK, &[A]), wait(12, 1, RMW_RET_OK, &[B])];
        let s = Schedule::build(&waits, &[take(11, 0, A), take(13, 1, B)], &clock_set());
        let gate = Gate::new(&s);
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Serve(0));
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::NoMore);
        // Thread 1 takes the last unit and never comes back to rmw_wait.
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Serve(1));
        let started = Instant::now();
        let mut stalls = Vec::new();
        // Thread 0 polls the way an idle executor thread does. Its own waits
        // do not count as the holder's work; the holder's outputs do.
        let mut worked = false;
        while stalls.is_empty() {
            gate.note_progress();
            assert_eq!(gate.wait_turn(0, |st| stalls.push(st)), Turn::NoMore);
            if !worked && started.elapsed() >= Duration::from_secs(1) {
                // The holder publishes once: the stall clock restarts.
                gate.note_work();
                worked = true;
            }
            assert!(started.elapsed() < WATCHDOG * 3, "no stall reported");
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(started.elapsed() >= WATCHDOG + Duration::from_secs(1));
        assert_eq!(
            stalls,
            vec![Stall {
                holder_thread: 1,
                unit: 1,
                waiting_thread: 0
            }],
            "the stall names the holder and its unit"
        );
        // Reported once; still not Exhausted (which would close the
        // holder's open segment).
        assert_eq!(gate.wait_turn(0, |_| panic!("reported once")), Turn::NoMore);
        // The holder comes back: the schedule is exhausted.
        assert_eq!(gate.wait_turn(1, |_| panic!("no stall")), Turn::Exhausted);
        assert_eq!(gate.wait_turn(0, |_| panic!("no stall")), Turn::Exhausted);
    }

    #[test]
    fn a_torn_logs_takes_past_a_threads_last_record_are_past_the_end() {
        let takes = [take(11, 0, A), take(12, 0, B)];
        let mut q = TakeQueues::new(&takes, true);
        assert_eq!(q.unserved(), 2);
        assert_eq!(q.first_unserved(), Some((0, A, 11)));
        assert!(q
            .serve(Some(0), A, TakeKind::Message, "sub:/a#0")
            .rec
            .is_some());
        assert!(q
            .serve(Some(0), B, TakeKind::Message, "sub:/b#0")
            .rec
            .is_some());
        assert_eq!((q.unserved(), q.first_unserved()), (0, None));
        let s = q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        assert!(
            s.past_end && s.rec.is_none() && s.mismatch.is_none(),
            "{s:?}"
        );
        // A thread with no recorded take at all, in a torn log.
        let s = q.serve(Some(3), A, TakeKind::Message, "sub:/a#0");
        assert!(s.past_end && s.mismatch.is_none(), "{s:?}");
        // The same take in a log that closed cleanly is a mismatch.
        let mut q = TakeQueues::new(&takes, false);
        q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        q.serve(Some(0), B, TakeKind::Message, "sub:/b#0");
        let s = q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        assert!(!s.past_end && s.mismatch.is_some(), "{s:?}");
    }

    #[test]
    fn a_recorded_take_never_made_stays_unserved() {
        let takes = [take(11, 0, A), take(12, 0, B), take(14, 1, A)];
        let mut q = TakeQueues::new(&takes, false);
        q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        assert_eq!(q.unserved(), 2);
        assert_eq!(q.first_unserved(), Some((0, B, 12)));
        assert_eq!(
            q.first_unserved_where(|t, k| (t, k) != (0, 12)),
            Some((1, A, 14))
        );
    }

    #[test]
    fn takes_are_served_per_entity_and_out_of_order_takes_are_named() {
        let takes = [take(11, 0, A), take(12, 0, B), take(13, 0, A)];
        let mut q = TakeQueues::new(&takes, false);
        let s = q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        assert_eq!(s.rec.unwrap().k, 11);
        assert!(s.mismatch.is_none());
        // The recording took B next; the replay takes A.
        let s = q.serve(Some(0), A, TakeKind::Message, "sub:/a#0");
        assert_eq!(s.rec.unwrap().k, 13);
        assert!(s.mismatch.unwrap().contains("sub:/a#0"));
        let s = q.serve(Some(0), B, TakeKind::Message, "sub:/b#0");
        assert_eq!(s.rec.unwrap().k, 12);
        let s = q.serve(Some(0), B, TakeKind::Message, "sub:/b#0");
        assert!(s.rec.is_none());
        assert!(s.mismatch.is_some());
    }

    #[test]
    fn graph_answers_are_served_in_recorded_order_per_query() {
        let rec = |k, answer| GraphRec {
            k,
            thread: 0,
            query: GraphQuery::CountSubscribers,
            name: "/clock".into(),
            ret: 0,
            answer,
        };
        let mut g = GraphAnswers::new(&[rec(1, 0), rec(2, 2)]);
        assert_eq!(
            g.next(GraphQuery::CountSubscribers, "/clock")
                .unwrap()
                .answer,
            0
        );
        assert_eq!(
            g.next(GraphQuery::CountSubscribers, "/clock")
                .unwrap()
                .answer,
            2
        );
        assert!(g.next(GraphQuery::CountSubscribers, "/clock").is_none());
        assert!(g.next(GraphQuery::CountPublishers, "/clock").is_none());
    }
}

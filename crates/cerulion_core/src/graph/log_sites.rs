// SPDX-License-Identifier: AGPL-3.0-only
//! Throttled and once-only log macros for node code.
//!
//! Each level has two variants on top of the plain `tracing` macros:
//!
//! | Macro | Emits | ROS 2 name |
//! |---|---|---|
//! | `warn_throttle!(self, period, ..)` | at most once per `period` of graph time, per call site | `RCLCPP_WARN_THROTTLE` |
//! | `warn_once!(self, ..)` | the first time the call site runs | `RCLCPP_WARN_ONCE` |
//!
//! and the same pair for `error_`, `info_`, `debug_` and `trace_` (ROS 2 has
//! no trace level).
//!
//! The first argument is the node (`self` inside `tick`, `init`, `shutdown`
//! or an `#[on_event]` handler). A throttle takes a [`Duration`] next. The
//! rest is exactly what the plain macro takes (`target:`, fields, a format
//! string), and it is forwarded unchanged, so an emitted line has the same
//! level, target and fields as `tracing::warn!` written at that line.
//!
//! # Graph time
//!
//! A throttle reads the node's own clock, the one `self.now_ns()` reads, never
//! the wall clock. Under `--time-source virtual`, and when a recording is
//! re-executed from its first step, the emit and suppress pattern is a pure
//! function of graph time, so two runs print the same lines. A re-execution
//! from a mid-run checkpoint builds fresh nodes, and this state is not part
//! of a node's captured state, so it starts fresh at the anchor: the first
//! call at each site emits there. The rule, in graph nanoseconds:
//!
//! - the first call at a site emits;
//! - a later call emits when `now >= last_emit + period` (a call exactly at
//!   the period end emits, one nanosecond before does not);
//! - a call whose `now` is EARLIER than the last emit (the clock went back)
//!   does not emit, and the period restarts from that earlier time;
//! - a zero period applies the same rule, so on a clock that only moves
//!   forward every call emits;
//! - a clock reading of `u64::MAX` counts as `u64::MAX - 1`, so the state
//!   word can also say "never emitted".
//!
//! A runtime subscriber filter (`RUST_LOG`) only decides whether a permitted
//! line is printed; the state follows graph time whatever the filter says, so
//! the pattern never depends on the filter.
//!
//! # State
//!
//! Each call site owns one `static` [`LogSite`]: a slot number it takes the
//! first time it runs. The state itself is one word per slot in a fixed table
//! inside each node instance ([`LogSiteTable`], [`LOG_SITE_SLOTS`] slots), so
//! a `once` site emits once PER NODE INSTANCE, and two instances of one node
//! type keep separate throttles. Two call sites never share a slot; the same
//! call site inside a loop is one site. Nothing grows: no map, no
//! allocation.
//!
//! Slot numbers are counted per linked binary: each node cdylib counts its
//! own sites, and an in-process host counts every site of every node type
//! linked into it, together. Past [`LOG_SITE_SLOTS`] sites in one binary, the
//! extra sites keep their state in the site's `static`, which every instance
//! of that node type shares, and the first such site says so once with a
//! `warn` that names its file, line and module.
//!
//! # Cost
//!
//! A suppressed call is one atomic load of the slot number, one clock
//! read (throttle only) and one relaxed atomic load of the slot. It does not
//! format or evaluate its message arguments, and it allocates nothing. A
//! level the build compiles out (a release build removes `debug` and
//! `trace`) costs nothing: the call does not read the clock, keeps no state
//! and takes no slot.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::Duration;

use crate::graph::node::CerNodeRuntimeFields;

/// Slots in each node instance's [`LogSiteTable`]: the number of throttle and
/// once call sites one linked binary (a node cdylib, or an in-process host
/// with all its node types) can hold per instance.
pub const LOG_SITE_SLOTS: usize = 64;

/// Slot word meaning "this site has not emitted yet".
const NEVER: u64 = 0;

/// Slot number a [`LogSite`] holds before its first call.
const UNASSIGNED: u32 = u32::MAX;

/// Slot number a [`LogSite`] holds while its first caller takes a number.
const RESERVING: u32 = u32::MAX - 1;

/// The next free slot number in this linked copy of the framework. Each node
/// cdylib links its own copy, so each node library numbers its own sites; an
/// in-process host numbers the sites of all its node types from one counter.
static NEXT_SLOT: AtomicU32 = AtomicU32::new(0);

/// Set once the first site past [`LOG_SITE_SLOTS`] has said so.
static OVERFLOW_WARNED: AtomicBool = AtomicBool::new(false);

/// The per-instance state table, one word per call site. Lives in the hidden
/// runtime fields every `#[cerulion_node]` struct carries.
#[doc(hidden)]
pub struct LogSiteTable {
    slots: [AtomicU64; LOG_SITE_SLOTS],
}

impl Default for LogSiteTable {
    fn default() -> Self {
        Self {
            slots: std::array::from_fn(|_| AtomicU64::new(NEVER)),
        }
    }
}

/// A clone is a NEW node instance for log state: it starts with no once line
/// printed and no throttle window open, as a freshly built instance does.
/// Copying the words would make a clone skip lines its original printed.
impl Clone for LogSiteTable {
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl std::fmt::Debug for LogSiteTable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let used = self
            .slots
            .iter()
            .filter(|s| s.load(Ordering::Relaxed) != NEVER)
            .count();
        f.debug_struct("LogSiteTable")
            .field("slots", &LOG_SITE_SLOTS)
            .field("used", &used)
            .finish()
    }
}

/// One throttle or once call site. The macros declare one as a `static` at
/// each call.
#[doc(hidden)]
pub struct LogSite {
    slot: AtomicU32,
    /// Where the site is, for the one overflow `warn`.
    file: &'static str,
    line: u32,
    module: &'static str,
    /// The state word for a site past [`LOG_SITE_SLOTS`]: shared by every
    /// instance of the node type.
    overflow: AtomicU64,
}

impl LogSite {
    /// A site at `file:line` in `module` that has not run yet.
    pub const fn new(file: &'static str, line: u32, module: &'static str) -> Self {
        Self {
            slot: AtomicU32::new(UNASSIGNED),
            file,
            line,
            module,
            overflow: AtomicU64::new(NEVER),
        }
    }

    /// The slot number, taken on the first call.
    ///
    /// Exactly one caller takes a number from [`NEXT_SLOT`]: the first to move
    /// the site from [`UNASSIGNED`] to [`RESERVING`]. Callers that race it wait
    /// for its number instead of taking one of their own, so concurrent first
    /// calls never spend a slot the site does not keep. The wait happens once
    /// per site, on its first calls only.
    fn slot(&self) -> u32 {
        let slot = self.slot.load(Ordering::Acquire);
        if slot != UNASSIGNED && slot != RESERVING {
            return slot;
        }
        if self
            .slot
            .compare_exchange(UNASSIGNED, RESERVING, Ordering::Acquire, Ordering::Acquire)
            .is_ok()
        {
            let fresh = NEXT_SLOT.fetch_add(1, Ordering::Relaxed);
            self.slot.store(fresh, Ordering::Release);
            return fresh;
        }
        loop {
            let slot = self.slot.load(Ordering::Acquire);
            if slot != RESERVING {
                return slot;
            }
            std::thread::yield_now();
        }
    }

    /// The state word this site uses in `table`.
    fn word<'a>(&'a self, table: &'a LogSiteTable) -> &'a AtomicU64 {
        match table.slots.get(self.slot() as usize) {
            Some(word) => word,
            None => {
                if !OVERFLOW_WARNED.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        slots = LOG_SITE_SLOTS,
                        site = %self.file,
                        line = self.line,
                        module = %self.module,
                        "this binary has more throttle and once log call sites than the \
                         per-instance table holds; this site and every later one share \
                         their state across every instance of the node type"
                    );
                }
                &self.overflow
            }
        }
    }
}

/// What a throttle does with one call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ThrottleStep {
    /// Emit, and the period starts at `now`.
    Emit,
    /// Suppress; the state does not change.
    Suppress,
    /// The clock went back: suppress, and the period restarts at `now`.
    Rewind,
}

/// The throttle rule over graph nanoseconds. `last_emit` is `None` before the
/// site's first emit. Pure: the module doc states the rule this implements.
pub(crate) fn throttle_step(last_emit: Option<u64>, now: u64, period_ns: u64) -> ThrottleStep {
    match last_emit {
        None => ThrottleStep::Emit,
        Some(last) if now < last => ThrottleStep::Rewind,
        Some(last) if now - last >= period_ns => ThrottleStep::Emit,
        Some(_) => ThrottleStep::Suppress,
    }
}

/// The latest graph time a throttle tells apart. One word holds every time
/// from 0 to this value plus [`NEVER`], so a clock reading of `u64::MAX`
/// counts as this value. That keeps the stored time and the time the rule
/// compares the same at the top of the range: repeated calls at `u64::MAX`
/// read as the same instant, inside the period.
const LATEST: u64 = u64::MAX - 1;

/// The state word for a time at or before [`LATEST`]: `now + 1`, so a word is
/// never [`NEVER`].
fn encode(now: u64) -> u64 {
    debug_assert!(now <= LATEST);
    now + 1
}

fn decode(word: u64) -> Option<u64> {
    word.checked_sub(1)
}

/// `true` when a throttle call at `site` emits now. The macros call this.
#[doc(hidden)]
#[inline]
pub fn throttle_permits(rt: &CerNodeRuntimeFields, site: &LogSite, period: Duration) -> bool {
    let word = site.word(&rt.log_sites);
    let now = rt.clock.now_ns().min(LATEST);
    let period_ns = u64::try_from(period.as_nanos()).unwrap_or(u64::MAX);
    let old = word.load(Ordering::Relaxed);
    match throttle_step(decode(old), now, period_ns) {
        ThrottleStep::Suppress => false,
        step => {
            // A lost race means another thread of this instance took this
            // window first: it emits, this call does not.
            let won = word
                .compare_exchange(old, encode(now), Ordering::Relaxed, Ordering::Relaxed)
                .is_ok();
            won && step == ThrottleStep::Emit
        }
    }
}

/// `true` the first time a once call at `site` runs on this node instance.
/// The macros call this.
#[doc(hidden)]
#[inline]
pub fn once_permits(rt: &CerNodeRuntimeFields, site: &LogSite) -> bool {
    let word = site.word(&rt.log_sites);
    word.load(Ordering::Relaxed) == NEVER && word.swap(1, Ordering::Relaxed) == NEVER
}

/// A throttle period. Implemented for [`Duration`] only, so a bare number
/// (which unit?) is a compile error.
///
/// This is the CI-gated half of that pin: a bare number does not build. The
/// rendered message is pinned by the `#[ignore]`d
/// `tests/ui/type_error/log_throttle_period_not_duration.rs` fixture.
///
/// ```compile_fail
/// use cerulion_core::graph::node::CerNodeRuntimeFields;
/// use cerulion_core::warn_throttle;
/// struct Probe { __cer_rt: CerNodeRuntimeFields }
/// let probe = Probe { __cer_rt: CerNodeRuntimeFields::default() };
/// warn_throttle!(probe, 500, "x");
/// ```
///
/// The twin: the same call with a `Duration` builds, so the refusal above is
/// about the period and not about the probe.
///
/// ```no_run
/// use std::time::Duration;
/// use cerulion_core::graph::node::CerNodeRuntimeFields;
/// use cerulion_core::warn_throttle;
/// struct Probe { __cer_rt: CerNodeRuntimeFields }
/// let probe = Probe { __cer_rt: CerNodeRuntimeFields::default() };
/// warn_throttle!(probe, Duration::from_millis(500), "x");
/// ```
#[doc(hidden)]
#[diagnostic::on_unimplemented(
    message = "a throttle period must be a `std::time::Duration`, not `{Self}`",
    label = "not a `Duration`",
    note = "write the period as a `Duration`, for example `Duration::from_millis(500)`, \
            then the message: `warn_throttle!(self, Duration::from_millis(500), \"text\")`"
)]
pub trait ThrottlePeriod {
    /// The period as a [`Duration`].
    fn into_period(self) -> Duration;
}

impl ThrottlePeriod for Duration {
    #[inline]
    fn into_period(self) -> Duration {
        self
    }
}

/// The shared body of the `*_throttle!` macros.
#[doc(hidden)]
#[macro_export]
macro_rules! __cer_log_throttle {
    ($level:ident, $LEVEL:ident, $name:literal, $node:expr, $period:expr, $($arg:tt)+) => {{
        static __CER_LOG_SITE: $crate::graph::log_sites::LogSite =
            $crate::graph::log_sites::LogSite::new(
                ::core::file!(),
                ::core::line!(),
                ::core::module_path!(),
            );
        // A level this build compiles out (`release_max_level_info` removes
        // debug and trace) does no work at all: no clock read, no state, no
        // slot. The comparison is a constant, so the branch compiles away.
        if $crate::tracing::Level::$LEVEL <= $crate::tracing::level_filters::STATIC_MAX_LEVEL
            && $crate::graph::log_sites::throttle_permits(
            &$node.__cer_rt,
            &__CER_LOG_SITE,
            $crate::graph::log_sites::ThrottlePeriod::into_period($period),
        ) {
            $crate::tracing::$level!($($arg)+);
        }
    }};
    ($level:ident, $LEVEL:ident, $name:literal, $node:expr, $period:expr $(,)?) => {
        ::core::compile_error!(::core::concat!(
            "`", $name, "!` needs the node, a period and a message, \
             for example `", $name, "!(self, Duration::from_millis(500), \"text\")`"
        ))
    };
    ($level:ident, $LEVEL:ident, $name:literal $($rest:tt)*) => {
        ::core::compile_error!(::core::concat!(
            "`", $name, "!` needs the node, a period and a message, \
             for example `", $name, "!(self, Duration::from_millis(500), \"text\")`"
        ))
    };
}

/// The shared body of the `*_once!` macros.
#[doc(hidden)]
#[macro_export]
macro_rules! __cer_log_once {
    ($level:ident, $LEVEL:ident, $name:literal, $node:expr, $($arg:tt)+) => {{
        static __CER_LOG_SITE: $crate::graph::log_sites::LogSite =
            $crate::graph::log_sites::LogSite::new(
                ::core::file!(),
                ::core::line!(),
                ::core::module_path!(),
            );
        // As for a throttle: a compiled-out level does no work and takes no
        // slot.
        if $crate::tracing::Level::$LEVEL <= $crate::tracing::level_filters::STATIC_MAX_LEVEL
            && $crate::graph::log_sites::once_permits(&$node.__cer_rt, &__CER_LOG_SITE)
        {
            $crate::tracing::$level!($($arg)+);
        }
    }};
    ($level:ident, $LEVEL:ident, $name:literal $($rest:tt)*) => {
        ::core::compile_error!(::core::concat!(
            "`", $name, "!` needs the node and a message, for example `",
            $name, "!(self, \"text\")`"
        ))
    };
}

/// `tracing::error!` at most once per `period` of graph time at this call
/// site: `error_throttle!(self, Duration::from_secs(1), "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! error_throttle {
    ($($t:tt)*) => { $crate::__cer_log_throttle!(error, ERROR, "error_throttle", $($t)*) };
}

/// `tracing::warn!` at most once per `period` of graph time at this call
/// site: `warn_throttle!(self, Duration::from_secs(1), "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! warn_throttle {
    ($($t:tt)*) => { $crate::__cer_log_throttle!(warn, WARN, "warn_throttle", $($t)*) };
}

/// `tracing::info!` at most once per `period` of graph time at this call
/// site: `info_throttle!(self, Duration::from_secs(1), "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! info_throttle {
    ($($t:tt)*) => { $crate::__cer_log_throttle!(info, INFO, "info_throttle", $($t)*) };
}

/// `tracing::debug!` at most once per `period` of graph time at this call
/// site: `debug_throttle!(self, Duration::from_secs(1), "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! debug_throttle {
    ($($t:tt)*) => { $crate::__cer_log_throttle!(debug, DEBUG, "debug_throttle", $($t)*) };
}

/// `tracing::trace!` at most once per `period` of graph time at this call
/// site: `trace_throttle!(self, Duration::from_secs(1), "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! trace_throttle {
    ($($t:tt)*) => { $crate::__cer_log_throttle!(trace, TRACE, "trace_throttle", $($t)*) };
}

/// `tracing::error!` the first time this call site runs on this node
/// instance: `error_once!(self, "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! error_once {
    ($($t:tt)*) => { $crate::__cer_log_once!(error, ERROR, "error_once", $($t)*) };
}

/// `tracing::warn!` the first time this call site runs on this node
/// instance: `warn_once!(self, "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! warn_once {
    ($($t:tt)*) => { $crate::__cer_log_once!(warn, WARN, "warn_once", $($t)*) };
}

/// `tracing::info!` the first time this call site runs on this node
/// instance: `info_once!(self, "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! info_once {
    ($($t:tt)*) => { $crate::__cer_log_once!(info, INFO, "info_once", $($t)*) };
}

/// `tracing::debug!` the first time this call site runs on this node
/// instance: `debug_once!(self, "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! debug_once {
    ($($t:tt)*) => { $crate::__cer_log_once!(debug, DEBUG, "debug_once", $($t)*) };
}

/// `tracing::trace!` the first time this call site runs on this node
/// instance: `trace_once!(self, "text")`. See
/// [`graph::log_sites`](crate::graph::log_sites).
#[macro_export]
macro_rules! trace_once {
    ($($t:tt)*) => { $crate::__cer_log_once!(trace, TRACE, "trace_once", $($t)*) };
}

#[cfg(test)]
mod tests {
    use super::{throttle_step, ThrottleStep::*};

    const MS: u64 = 1_000_000;

    /// Hand-written vectors over a 100 ms period: (last emit, now, expected).
    #[test]
    fn the_throttle_rule_matches_hand_written_clock_vectors() {
        let period = 100 * MS;
        let vectors: &[(Option<u64>, u64, super::ThrottleStep)] = &[
            // The first call emits, at any time, including graph time zero.
            (None, 0, Emit),
            (None, 5 * MS, Emit),
            // Exactly at the period end emits; one nanosecond before does not.
            (Some(5 * MS), 105 * MS, Emit),
            (Some(5 * MS), 105 * MS - 1, Suppress),
            // The same instant as the last emit is inside the period.
            (Some(5 * MS), 5 * MS, Suppress),
            // Long after the period still emits once.
            (Some(5 * MS), 10_000 * MS, Emit),
            // The clock went back: no emit, the period restarts.
            (Some(500 * MS), 499 * MS, Rewind),
            (Some(500 * MS), 0, Rewind),
            // Near the top of the clock range: no overflow, no panic.
            (Some(u64::MAX - 1), u64::MAX, Suppress),
            (Some(0), u64::MAX, Emit),
        ];
        for &(last, now, want) in vectors {
            assert_eq!(
                throttle_step(last, now, period),
                want,
                "last {last:?}, now {now}"
            );
        }
    }

    #[test]
    fn a_zero_period_emits_every_call_while_the_clock_moves_forward_or_stands() {
        assert_eq!(throttle_step(None, 7, 0), Emit);
        assert_eq!(throttle_step(Some(7), 7, 0), Emit);
        assert_eq!(throttle_step(Some(7), 8, 0), Emit);
        // Going back is still a rewind, as for any period.
        assert_eq!(throttle_step(Some(7), 6, 0), Rewind);
    }

    #[test]
    fn a_maximal_period_never_emits_twice() {
        assert_eq!(throttle_step(Some(0), u64::MAX - 1, u64::MAX), Suppress);
        assert_eq!(throttle_step(Some(1), u64::MAX, u64::MAX), Suppress);
    }

    #[test]
    fn the_state_word_round_trips_and_is_never_the_unset_word() {
        assert_eq!(super::decode(super::encode(0)), Some(0));
        assert_eq!(super::decode(super::encode(42)), Some(42));
        assert_ne!(super::encode(0), super::NEVER);
        // The top of the range round-trips exactly too: no time reads back as
        // another one.
        assert_eq!(super::LATEST, u64::MAX - 1);
        assert_eq!(
            super::decode(super::encode(super::LATEST)),
            Some(u64::MAX - 1)
        );
        assert_ne!(super::encode(super::LATEST), super::NEVER);
        assert_eq!(super::decode(super::NEVER), None);
    }

    /// Sixteen threads make the FIRST call at one site at the same moment. The
    /// site keeps one number and the counter moves by exactly one: no racer
    /// spends a slot of its own. No other code in this library takes slot
    /// numbers, so the counter is this test's to read.
    #[test]
    fn concurrent_first_calls_at_one_site_take_exactly_one_slot() {
        use std::sync::atomic::Ordering;
        use std::sync::Barrier;

        static RACED: super::LogSite = super::LogSite::new(file!(), line!(), module_path!());
        static NEXT: super::LogSite = super::LogSite::new(file!(), line!(), module_path!());
        const RACERS: usize = 16;

        let before = super::NEXT_SLOT.load(Ordering::SeqCst);
        let barrier = Barrier::new(RACERS);
        let slots: Vec<u32> = std::thread::scope(|s| {
            let handles: Vec<_> = (0..RACERS)
                .map(|_| {
                    s.spawn(|| {
                        barrier.wait();
                        RACED.slot()
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });
        assert!(
            slots.iter().all(|&n| n == slots[0]),
            "every racer sees the one number the site keeps: {slots:?}"
        );
        assert_eq!(slots[0], before, "the site took the next free number");
        assert_eq!(
            super::NEXT_SLOT.load(Ordering::SeqCst),
            before + 1,
            "the race spent one number, not one per racer"
        );
        // The next site gets the very next number: nothing was skipped.
        assert_eq!(NEXT.slot(), before + 1);
        assert_eq!(RACED.slot(), before, "a later call reads the same number");
    }
}

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
//! re-executed, the emit and suppress pattern is a pure function of graph
//! time, so two runs emit at the same calls and suppress the same calls.
//! What an emitted line says is what its arguments say in that run: the
//! macros decide only whether a call emits. The state is part of the node's
//! captured state, so a re-execution from a mid-run checkpoint continues each
//! site's once and throttle state from that checkpoint: a once line that
//! printed before the checkpoint does not print again, and an open throttle
//! window stays open. The rule, in graph nanoseconds:
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
//! Each call site has a stable id, fixed when the node is compiled: a hash of
//! its module path, file name, line, column, macro name and arguments. The
//! same source gives the same id in every run and every build, so a
//! checkpoint names each site by id. Only the file NAME is part of the id,
//! not its directory: a build that remaps its source paths
//! (`--remap-path-prefix`, a different checkout directory) gives the same
//! ids, so a checkpoint from one build restores into the other, and two
//! files that `include!` puts in one module still get their own ids. Two
//! such files share an id only when they also share their file name, line,
//! column and arguments. Two log calls that one invocation of a user's own
//! macro produces with the same arguments get the same id, so they count as
//! one site.
//!
//! The state itself is one entry per site in a fixed table inside each node
//! instance ([`LogSiteTable`]), keyed by the site's id, so a `once` site
//! emits once PER NODE INSTANCE, and two instances of one node type keep
//! separate throttles. The same call site inside a loop is one site. Nothing
//! grows: no map, no allocation.
//!
//! An instance keeps state for [`LOG_SITE_SLOTS`] distinct sites. A site past
//! that has no entry: it keeps no state at all and every call emits, as the
//! plain macro would. The first time in the process that a site finds no
//! entry, it logs one `error` that names its file, line and module; later
//! calls of that site that find no entry, on any instance, log nothing. No
//! site ever shares state with another site or another instance.
//!
//! A restore keeps the recorded entries apart until their sites run: a
//! recorded entry counts toward [`LOG_SITE_SLOTS`] only from its site's first
//! call after the restore. A checkpoint from a build whose sites an edit has
//! since moved or removed therefore never takes the room of the sites this
//! build has. The one cost is on an instance that runs more than
//! [`LOG_SITE_SLOTS`] distinct sites: after a restore, which sites keep state
//! follows the order they run in after the restore.
//!
//! # Cost
//!
//! A suppressed call is a few atomic loads (acquire for the table lookup,
//! relaxed for the state word) and one clock read (throttle only). It does
//! not format or evaluate its message arguments, and it allocates nothing. A
//! level the build compiles out (a release build removes `debug` and `trace`)
//! costs nothing: the call does not read the clock, keeps no state and takes
//! no entry.

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::time::Duration;

use crate::graph::node::CerNodeRuntimeFields;
use crate::state::{StateCursor, StateError, StateSink};

/// The number of distinct throttle and once call sites one node instance
/// keeps state for.
pub const LOG_SITE_SLOTS: usize = 64;

/// Entries in each [`LogSiteTable`]. Twice [`LOG_SITE_SLOTS`]: a restore puts
/// at most [`LOG_SITE_SLOTS`] recorded entries in, and the sites of this
/// build take at most [`LOG_SITE_SLOTS`] more, so a probe always finds the
/// site's id or a free entry.
const TABLE_ENTRIES: usize = 2 * LOG_SITE_SLOTS;

/// State word meaning "this site has not emitted yet".
const NEVER: u64 = 0;

/// Entry id meaning "this entry holds no site". [`site_id`] never returns it.
const EMPTY: u64 = 0;

/// The first word of a captured log site section ("CLOG", little-endian).
/// It tells the section apart from bytes a changed field list left behind.
const SECTION_TAG: u32 = u32::from_le_bytes(*b"CLOG");

/// FNV-1a over `bytes`, continuing from `hash`.
const fn fnv1a(mut hash: u64, bytes: &[u8]) -> u64 {
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        i += 1;
    }
    hash
}

/// The last component of `path`: the bytes after its last `/` or `\`.
const fn file_name(path: &str) -> &[u8] {
    let bytes = path.as_bytes();
    let mut start = 0;
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'/' || bytes[i] == b'\\' {
            start = i + 1;
        }
        i += 1;
    }
    bytes.split_at(start).1
}

/// The stable id of the call site at `line:column` in `module`, in the file
/// `file`, where `call` is the macro name and its arguments as written. A
/// pure function of the source, computed at compile time. Never [`EMPTY`].
///
/// Only the file NAME counts, not its directory: the directory changes with
/// the build directory and `--remap-path-prefix`, and an id must not. The
/// name tells apart two files that `include!` puts in one module.
///
/// A `0xff` byte ends the module path and the file name: it never occurs in
/// UTF-8, so no byte moves between one part and the next.
#[doc(hidden)]
pub const fn site_id(file: &str, module: &str, line: u32, column: u32, call: &str) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325;
    hash = fnv1a(hash, module.as_bytes());
    hash = fnv1a(hash, &[0xff]);
    hash = fnv1a(hash, file_name(file));
    hash = fnv1a(hash, &[0xff]);
    hash = fnv1a(hash, &line.to_le_bytes());
    hash = fnv1a(hash, &column.to_le_bytes());
    hash = fnv1a(hash, call.as_bytes());
    if hash == EMPTY {
        1
    } else {
        hash
    }
}

/// One table entry: the id of the site that owns it, its state word, and
/// whether a site of this build has claimed it.
struct Entry {
    id: AtomicU64,
    word: AtomicU64,
    /// `false` only for an entry a restore wrote that no site has run since:
    /// it may belong to a site an edit removed, so it does not count toward
    /// [`LOG_SITE_SLOTS`].
    claimed: AtomicBool,
}

/// The per-instance state table, one entry per call site, keyed by the
/// site's stable id. Lives in the hidden runtime fields every
/// `#[cerulion_node]` struct carries, and rides the node's captured state.
#[doc(hidden)]
pub struct LogSiteTable {
    entries: [Entry; TABLE_ENTRIES],
    /// Claimed entries, at most [`LOG_SITE_SLOTS`].
    claimed_count: AtomicUsize,
}

impl Default for LogSiteTable {
    fn default() -> Self {
        Self {
            entries: std::array::from_fn(|_| Entry {
                id: AtomicU64::new(EMPTY),
                word: AtomicU64::new(NEVER),
                claimed: AtomicBool::new(false),
            }),
            claimed_count: AtomicUsize::new(0),
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
        f.debug_struct("LogSiteTable")
            .field("slots", &LOG_SITE_SLOTS)
            .field("used", &self.used())
            .finish()
    }
}

impl LogSiteTable {
    /// The state word of the site `id`. The site's first call claims an
    /// entry: the restored one that holds `id`, or a free one. `None` when
    /// [`LOG_SITE_SLOTS`] other sites have claimed entries.
    ///
    /// A claim counts itself before it takes an entry, and a claim that loses
    /// a race gives its count back. A full count can therefore hold a claim
    /// that is still in flight: the first call of a site on another thread
    /// that will give its count back, or the first call of this same site
    /// that has not published its id yet. So a full count is checked against
    /// the claimed entries, which only ever grow, and while fewer than
    /// [`LOG_SITE_SLOTS`] are claimed the lookup waits for the claim in
    /// flight and tries again. Once [`LOG_SITE_SLOTS`] are claimed, one more
    /// lookup finds the site if a racing first call of it was among them. A
    /// site is past the limit only when [`LOG_SITE_SLOTS`] other sites hold
    /// claimed entries.
    fn word(&self, id: u64) -> Option<&AtomicU64> {
        let mut spins: u32 = 0;
        loop {
            if let Some(word) = self.probe(id) {
                return Some(word);
            }
            if self.claimed_entries() >= LOG_SITE_SLOTS {
                // The limit is reached, but a racing first call of this same
                // site may be one of the claims that reached it: look again,
                // now that every claimed entry's id is visible.
                return self.probe(id);
            }
            // A claim in flight takes a few atomic steps to settle; give its
            // thread the core if it was preempted in the middle of them.
            if spins < 64 {
                spins += 1;
                std::hint::spin_loop();
            } else {
                std::thread::yield_now();
            }
        }
    }

    /// Claimed entries. Each one holds a count no claim gives back, and an
    /// entry never stops being claimed while the table is shared, so this
    /// never reads more than the true number. 128 loads, on the overflow
    /// path only.
    fn claimed_entries(&self) -> usize {
        self.entries
            .iter()
            .filter(|e| e.claimed.load(Ordering::Acquire))
            .count()
    }

    /// One lookup of [`Self::word`]. `None` when the count is full, which may
    /// still hold a claim in flight.
    ///
    /// Open addressing with linear probing from `id % TABLE_ENTRIES`. An
    /// entry, once it holds an id, keeps it for the life of the table (only
    /// [`Self::restore`], which holds the table exclusively, rewrites ids),
    /// so a probe that passes an entry never has to look at it again.
    fn probe(&self, id: u64) -> Option<&AtomicU64> {
        let start = (id % TABLE_ENTRIES as u64) as usize;
        for step in 0..TABLE_ENTRIES {
            let entry = &self.entries[(start + step) % TABLE_ENTRIES];
            let held = entry.id.load(Ordering::Acquire);
            if held == id {
                return self.claim(entry).then_some(&entry.word);
            }
            if held == EMPTY {
                // Count the site first, so no entry is taken past the limit.
                if !self.try_count() {
                    return None;
                }
                match entry
                    .id
                    .compare_exchange(EMPTY, id, Ordering::AcqRel, Ordering::Acquire)
                {
                    Ok(_) => {
                        if entry
                            .claimed
                            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                            .is_err()
                        {
                            // A racing first call of this site saw the id
                            // and claimed it, counting itself.
                            self.uncount();
                        }
                        return Some(&entry.word);
                    }
                    // A racing first call of the same site took it a moment
                    // earlier and counts it.
                    Err(won) if won == id => {
                        self.uncount();
                        return self.claim(entry).then_some(&entry.word);
                    }
                    // Another site took it: keep probing.
                    Err(_) => self.uncount(),
                }
            }
        }
        // Only while a claim is in flight: at most `LOG_SITE_SLOTS` restored
        // entries plus `LOG_SITE_SLOTS` claimed ones fill `TABLE_ENTRIES`,
        // and a claim is counted before it takes an entry.
        None
    }

    /// Mark `entry`, which holds this site's id, as claimed. `false` when it
    /// was not claimed yet and [`LOG_SITE_SLOTS`] other sites have claimed
    /// entries.
    fn claim(&self, entry: &Entry) -> bool {
        if entry.claimed.load(Ordering::Acquire) {
            return true;
        }
        if !self.try_count() {
            return false;
        }
        if entry
            .claimed
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            self.uncount();
        }
        true
    }

    /// Count one more claimed entry, unless [`LOG_SITE_SLOTS`] are claimed.
    fn try_count(&self) -> bool {
        let mut n = self.claimed_count.load(Ordering::Acquire);
        while n < LOG_SITE_SLOTS {
            match self.claimed_count.compare_exchange_weak(
                n,
                n + 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return true,
                Err(now) => n = now,
            }
        }
        false
    }

    /// Give back a count [`Self::try_count`] took.
    fn uncount(&self) {
        self.claimed_count.fetch_sub(1, Ordering::AcqRel);
    }

    /// Entries that hold a site.
    fn used(&self) -> usize {
        self.entries
            .iter()
            .filter(|e| e.id.load(Ordering::Relaxed) != EMPTY)
            .count()
    }

    /// `true` when the capture writes `entry`: every claimed entry, and a
    /// restored one no site has run since when its id is at or below `cut`.
    fn kept(entry: &Entry, cut: Option<u64>) -> bool {
        let id = entry.id.load(Ordering::Acquire);
        id != EMPTY && (entry.claimed.load(Ordering::Acquire) || cut.is_some_and(|c| id <= c))
    }

    /// The smallest id above `floor` among the entries `pick` accepts.
    fn next_above(
        &self,
        floor: Option<u64>,
        pick: impl Fn(&Entry) -> bool,
    ) -> Option<(u64, &Entry)> {
        self.entries
            .iter()
            .filter_map(|e| {
                let id = e.id.load(Ordering::Acquire);
                let above = floor.is_none_or(|f| id > f);
                (id != EMPTY && above && pick(e)).then_some((id, e))
            })
            .min_by_key(|(id, _)| *id)
    }

    /// Write this table into a node's captured state. A table no site has
    /// used writes nothing, so a node without throttle or once calls captures
    /// exactly the bytes of its fields and accumulate inputs. The node's
    /// generated `capture_state` appends this last.
    ///
    /// The capture holds every claimed entry, then the restored entries no
    /// site has run since, smallest id first, up to [`LOG_SITE_SLOTS`] in
    /// all. A restored entry is dropped only when the instance has seen more
    /// than [`LOG_SITE_SLOTS`] distinct sites since the restore.
    ///
    /// Layout (little-endian): `u32` tag `CLOG`, `u32` entry count, then per
    /// entry `u64` site id and `u64` state word, in ascending id order, so
    /// one table state has one encoding whatever order its sites first ran
    /// in.
    ///
    /// # Errors
    ///
    /// What the sink refuses.
    pub fn capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        let claimed = self
            .entries
            .iter()
            .filter(|e| e.id.load(Ordering::Relaxed) != EMPTY && e.claimed.load(Ordering::Acquire))
            .count();
        // The largest restored id that still fits, walking them in ascending
        // id order without a buffer. 128 entries, at a checkpoint only.
        let unclaimed = |e: &Entry| !e.claimed.load(Ordering::Acquire);
        let mut cut: Option<u64> = None;
        let mut written = claimed;
        while written < LOG_SITE_SLOTS {
            let Some((id, _)) = self.next_above(cut, unclaimed) else {
                break;
            };
            cut = Some(id);
            written += 1;
        }
        if written == 0 {
            return Ok(());
        }
        out.write(&SECTION_TAG.to_le_bytes())?;
        // At most `LOG_SITE_SLOTS` (64), so the cast is exact.
        out.write(&(written as u32).to_le_bytes())?;
        // Ascending id order without a buffer: each pass writes the smallest
        // kept id above the last one written.
        let mut floor: Option<u64> = None;
        for _ in 0..written {
            let Some((id, entry)) = self.next_above(floor, |e| Self::kept(e, cut)) else {
                break;
            };
            out.write(&id.to_le_bytes())?;
            out.write(&entry.word.load(Ordering::Relaxed).to_le_bytes())?;
            floor = Some(id);
        }
        Ok(())
    }

    /// Replace this table with what [`Self::capture`] wrote. The table ends
    /// up holding exactly the recorded entries: a site the recording does not
    /// name starts fresh, whatever this instance did before the restore. No
    /// bytes left in `src` means the recorded table was empty. The node's
    /// generated `restore_state` calls this after its accumulate inputs,
    /// before the cursor's trailing-byte check.
    ///
    /// A restored entry counts toward [`LOG_SITE_SLOTS`] only once its site
    /// runs, so entries of sites an edit removed never take the room of the
    /// sites this build has.
    ///
    /// # Errors
    ///
    /// [`StateError::TrailingBytes`] when the bytes left do not open with the
    /// section tag (the node's field list changed since the recording);
    /// [`StateError::LogSiteRestore`] when the section holds no entry (a
    /// capture writes no section for an empty table) or more entries than
    /// the table, or repeats or misorders an id; the cursor's own errors for
    /// truncated bytes.
    // hot-path-alloc-ok-fn: a state restore runs once, before the first step;
    // it allocates only to name a refusal
    pub fn restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        *self = Self::default();
        if src.is_empty() {
            return Ok(());
        }
        // Bytes that do not open with the tag are not a log site section:
        // the node's field list disagrees with the recorded one, the drift
        // the cursor's trailing-byte check reports, so report it the same way.
        let remaining = src.remaining();
        if remaining < 4 || src.take_array::<4>()? != SECTION_TAG.to_le_bytes() {
            return Err(StateError::TrailingBytes { remaining });
        }
        let refuse = |reason: String| StateError::LogSiteRestore { reason };
        let count = src.read_element_count()?;
        if count == 0 {
            return Err(refuse(String::from(
                "the recording holds a log site section with no entries; a capture \
                 writes no section for an empty table",
            )));
        }
        if count > LOG_SITE_SLOTS {
            return Err(refuse(format!(
                "the recording holds {count} log sites, a node instance holds at most \
                 {LOG_SITE_SLOTS}"
            )));
        }
        let mut last: Option<u64> = None;
        for _ in 0..count {
            let id = u64::from_le_bytes(src.take_array::<8>()?);
            let word = u64::from_le_bytes(src.take_array::<8>()?);
            if id == EMPTY {
                return Err(refuse(String::from(
                    "the recording holds a log site with id 0",
                )));
            }
            if last.is_some_and(|l| id <= l) {
                return Err(refuse(format!(
                    "the recording lists log site 0x{id:016x} out of ascending order; a \
                     capture never does"
                )));
            }
            self.place_restored(id, word);
            last = Some(id);
        }
        Ok(())
    }

    /// Put a recorded entry in the first free entry of its probe, unclaimed.
    /// `count` is at most [`LOG_SITE_SLOTS`] and every id is distinct, so a
    /// free entry always exists and no probe meets `id` already there.
    fn place_restored(&mut self, id: u64, word: u64) {
        let start = (id % TABLE_ENTRIES as u64) as usize;
        for step in 0..TABLE_ENTRIES {
            let entry = &mut self.entries[(start + step) % TABLE_ENTRIES];
            if *entry.id.get_mut() == EMPTY {
                *entry.id.get_mut() = id;
                *entry.word.get_mut() = word;
                return;
            }
        }
    }
}

/// One throttle or once call site. The macros declare one as a `static` at
/// each call.
#[doc(hidden)]
pub struct LogSite {
    /// The stable id ([`site_id`]).
    id: u64,
    /// Where the site is, for the error a site with no table entry prints.
    file: &'static str,
    line: u32,
    module: &'static str,
    /// Set once this site has said it has no table entry.
    unkept_reported: AtomicBool,
}

impl LogSite {
    /// The site at `file:line:column` in `module`, whose macro name and
    /// arguments read `call`.
    pub const fn new(
        file: &'static str,
        line: u32,
        column: u32,
        module: &'static str,
        call: &'static str,
    ) -> Self {
        Self {
            id: site_id(file, module, line, column, call),
            file,
            line,
            module,
            unkept_reported: AtomicBool::new(false),
        }
    }

    /// The site's stable id.
    pub const fn id(&self) -> u64 {
        self.id
    }

    /// The state word this site uses in `table`, or `None` when the table is
    /// full. The first `None` for this site says so with an `error`.
    fn word<'a>(&self, table: &'a LogSiteTable) -> Option<&'a AtomicU64> {
        let word = table.word(self.id);
        if word.is_none() && !self.unkept_reported.swap(true, Ordering::Relaxed) {
            tracing::error!(
                slots = LOG_SITE_SLOTS,
                site = %self.file,
                line = self.line,
                module = %self.module,
                "this node instance runs more distinct throttle and once log call sites \
                 than its table holds; this site keeps no state, so every call to it \
                 prints"
            );
        }
        word
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
    let Some(word) = site.word(&rt.log_sites) else {
        return true;
    };
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
    let Some(word) = site.word(&rt.log_sites) else {
        return true;
    };
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
                ::core::column!(),
                ::core::module_path!(),
                ::core::concat!($name, "(", ::core::stringify!($($arg)+), ")"),
            );
        // A level this build compiles out (`release_max_level_info` removes
        // debug and trace) does no work at all: no clock read, no state, no
        // table entry. The comparison is a constant, so the branch compiles away.
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
                ::core::column!(),
                ::core::module_path!(),
                ::core::concat!($name, "(", ::core::stringify!($($arg)+), ")"),
            );
        // As for a throttle: a compiled-out level does no work and takes no
        // table entry.
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

    /// FNV-1a 64 over the published test vectors: the empty input is the
    /// offset basis, and `"a"` is `0xaf63dc4c8601ec8c`.
    #[test]
    fn the_site_hash_is_fnv1a_over_the_published_vectors() {
        const BASIS: u64 = 0xcbf2_9ce4_8422_2325;
        assert_eq!(super::fnv1a(BASIS, b""), BASIS);
        assert_eq!(super::fnv1a(BASIS, b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(super::fnv1a(BASIS, b"foobar"), 0x8594_4171_f739_67e8);
    }

    #[test]
    fn a_site_id_is_a_pure_function_of_the_source_and_each_part_counts() {
        use super::site_id;
        let base = site_id("a.rs", "m", 10, 5, "warn_once(\"x\")");
        assert_eq!(base, site_id("a.rs", "m", 10, 5, "warn_once(\"x\")"));
        for other in [
            site_id("b.rs", "m", 10, 5, "warn_once(\"x\")"),
            site_id("a.rs", "m2", 10, 5, "warn_once(\"x\")"),
            site_id("a.rs", "m", 11, 5, "warn_once(\"x\")"),
            site_id("a.rs", "m", 10, 6, "warn_once(\"x\")"),
            site_id("a.rs", "m", 10, 5, "warn_once(\"y\")"),
            // The macro name counts: another macro family at the same
            // location with the same arguments is another site.
            site_id("a.rs", "m", 10, 5, "info_once(\"x\")"),
        ] {
            assert_ne!(base, other);
        }
        // Computed at compile time, as the macros use it.
        const AT_COMPILE_TIME: u64 = super::site_id("a.rs", "m", 10, 5, "warn_once(\"x\")");
        assert_eq!(AT_COMPILE_TIME, base);
        assert_ne!(base, super::EMPTY);
    }

    /// HAND ORACLE: the id is FNV-1a over module, `0xff`, file name, `0xff`,
    /// line and column (little-endian), then the call text.
    #[test]
    fn a_site_id_hashes_its_parts_in_the_documented_order() {
        const BASIS: u64 = 0xcbf2_9ce4_8422_2325;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"m");
        bytes.push(0xff);
        bytes.extend_from_slice(b"lib.rs");
        bytes.push(0xff);
        bytes.extend_from_slice(&[10, 0, 0, 0, 5, 0, 0, 0]);
        bytes.extend_from_slice(b"warn_once(\"x\")");
        assert_eq!(
            super::site_id("src/lib.rs", "m", 10, 5, "warn_once(\"x\")"),
            super::fnv1a(BASIS, &bytes)
        );
    }

    /// The last path component, for both separators and for a bare name.
    #[test]
    fn the_file_name_is_the_part_after_the_last_separator() {
        assert_eq!(super::file_name("node/src/lib.rs"), b"lib.rs");
        assert_eq!(super::file_name("C:\\build\\src\\part.rs"), b"part.rs");
        assert_eq!(super::file_name("lib.rs"), b"lib.rs");
        assert_eq!(super::file_name("dir/"), b"");
        assert_eq!(super::file_name(""), b"");
    }

    /// A build that remaps its source paths sees a different `file!()` for
    /// the same site. The id does not change, so a checkpoint from one build
    /// names the same sites in the other.
    #[test]
    fn a_site_keeps_its_id_when_the_build_remaps_its_file_path() {
        use super::LogSite;
        let call = "warn_once(\"x\")";
        let here = LogSite::new("node/src/lib.rs", 10, 5, "node", call);
        let remapped = LogSite::new("/build/src/lib.rs", 10, 5, "node", call);
        assert_eq!(here.id(), remapped.id());
        assert_eq!(here.id(), super::site_id("lib.rs", "node", 10, 5, call));
        let elsewhere = LogSite::new("/build/src/lib.rs", 10, 5, "node::inner", call);
        assert_ne!(here.id(), elsewhere.id());
    }

    /// Two files that `include!` puts in one module have one module path.
    /// The same call at the same line and column in each is two sites,
    /// because their file names differ.
    #[test]
    fn the_same_call_in_two_included_files_of_one_module_is_two_sites() {
        use super::LogSite;
        let call = "warn_once(\"x\")";
        let first = LogSite::new("node/src/part_a.rs", 3, 9, "node", call);
        let second = LogSite::new("node/src/part_b.rs", 3, 9, "node", call);
        assert_ne!(first.id(), second.id());
        let table = super::LogSiteTable::default();
        let a = std::ptr::from_ref(table.word(first.id()).unwrap());
        let b = std::ptr::from_ref(table.word(second.id()).unwrap());
        assert_ne!(a, b, "each file's site has its own state word");
    }

    /// Sixteen threads make the FIRST call at one site on one table at the
    /// same moment: the site takes exactly one entry, and every racer gets
    /// that entry's word.
    #[test]
    fn concurrent_first_calls_at_one_site_take_exactly_one_entry() {
        use std::sync::Barrier;

        const RACERS: usize = 16;
        let table = super::LogSiteTable::default();
        let barrier = Barrier::new(RACERS);
        let words: Vec<usize> = std::thread::scope(|s| {
            let handles: Vec<_> = (0..RACERS)
                .map(|_| {
                    s.spawn(|| {
                        barrier.wait();
                        let word = table.word(0x1234).expect("an empty table has room");
                        std::ptr::from_ref(word) as usize
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });
        assert!(
            words.iter().all(|&w| w == words[0]),
            "every racer sees the one entry the site took"
        );
        assert_eq!(
            table.used(),
            1,
            "the race took one entry, not one per racer"
        );
    }

    /// Claim `n` distinct sites (ids `0x100 + k`) on `table`.
    fn claim_sites(table: &super::LogSiteTable, n: usize) {
        for k in 0..n as u64 {
            table.word(0x100 + k).expect("room below the limit");
        }
    }

    /// With 63 sites claimed, another thread holds a count it will give back
    /// (a first call that lost its entry to a racer). A first call at the
    /// 64th site meanwhile must not read the full count as overflow: it waits
    /// for the claim in flight, then gets its own entry.
    #[test]
    fn a_claim_in_flight_never_reads_as_overflow() {
        let table = super::LogSiteTable::default();
        claim_sites(&table, super::LOG_SITE_SLOTS - 1);
        assert!(
            table.try_count(),
            "the in-flight claim takes the last count"
        );
        std::thread::scope(|s| {
            let late = s.spawn(|| table.word(0x9999).is_some());
            std::thread::sleep(std::time::Duration::from_millis(50));
            assert!(
                !late.is_finished(),
                "the 64th site waits instead of reporting overflow"
            );
            table.uncount();
            assert!(
                late.join().unwrap(),
                "the 64th site gets an entry once the claim in flight settles"
            );
        });
        assert!(table.word(0x8888).is_none(), "the 65th site has none");
    }

    /// With 63 sites claimed, many threads make the first call at the 64th
    /// site at the same moment, over many rounds: every racer gets the one
    /// entry the site took, none sees overflow, and the 65th site gets none.
    #[test]
    fn concurrent_first_calls_at_the_last_free_site_all_get_its_entry() {
        use std::sync::Barrier;

        const RACERS: usize = 8;
        for round in 0..200u64 {
            let table = super::LogSiteTable::default();
            claim_sites(&table, super::LOG_SITE_SLOTS - 1);
            let barrier = Barrier::new(RACERS);
            let id = 0x10_0000 + round;
            let words: Vec<Option<usize>> = std::thread::scope(|s| {
                let handles: Vec<_> = (0..RACERS)
                    .map(|_| {
                        s.spawn(|| {
                            barrier.wait();
                            table.word(id).map(|w| std::ptr::from_ref(w) as usize)
                        })
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().unwrap()).collect()
            });
            assert!(
                words.iter().all(|w| w.is_some() && *w == words[0]),
                "round {round}: every racer gets the 64th site's one entry"
            );
            assert_eq!(table.used(), super::LOG_SITE_SLOTS, "round {round}");
            assert!(
                table.word(id + 0x1000).is_none(),
                "round {round}: 65th site"
            );
        }
    }

    /// Sites whose ids all start their probe at entry 0 still each get an
    /// entry of their own until the table is full; the next site gets none.
    #[test]
    fn a_full_table_has_no_entry_for_one_more_site_and_keeps_every_other() {
        let table = super::LogSiteTable::default();
        let slots = super::LOG_SITE_SLOTS as u64;
        // Ids k * TABLE_ENTRIES all hash to entry 0 and probe from there.
        let entries = super::TABLE_ENTRIES as u64;
        let ids: Vec<u64> = (1..=slots).map(|k| k * entries).collect();
        for (n, &id) in ids.iter().enumerate() {
            table
                .word(id)
                .expect("room")
                .store(n as u64 + 1, Ordering::Relaxed);
        }
        assert!(table.word(9 * entries + 1).is_none(), "65th site: no entry");
        for (n, &id) in ids.iter().enumerate() {
            assert_eq!(
                table.word(id).unwrap().load(Ordering::Relaxed),
                n as u64 + 1,
                "site {id} keeps its own word"
            );
        }
    }

    use std::sync::atomic::Ordering;

    fn captured(table: &super::LogSiteTable) -> Vec<u8> {
        let mut sink = crate::state::VecSink::new();
        table.capture(&mut sink).expect("unbounded capture");
        sink.into_inner()
    }

    #[test]
    fn an_unused_table_captures_nothing_and_restores_from_nothing() {
        let table = super::LogSiteTable::default();
        assert!(captured(&table).is_empty());
        let mut target = super::LogSiteTable::default();
        target.word(7).unwrap().store(3, Ordering::Relaxed);
        let mut cursor = crate::state::StateCursor::new(&[]);
        target
            .restore(&mut cursor)
            .expect("an empty section is an empty table");
        assert_eq!(
            target.used(),
            0,
            "the live entry did not survive the restore"
        );
    }

    /// HAND ORACLE: tag, count, then (id, word) pairs in ascending id order,
    /// whatever order the sites first ran in.
    #[test]
    fn a_table_captures_in_ascending_id_order_and_restores_to_the_same_bytes() {
        let table = super::LogSiteTable::default();
        table.word(0x30).unwrap().store(1, Ordering::Relaxed);
        table.word(0x10).unwrap().store(0x0506, Ordering::Relaxed);
        let mut oracle = Vec::new();
        oracle.extend_from_slice(b"CLOG");
        oracle.extend_from_slice(&2u32.to_le_bytes());
        oracle.extend_from_slice(&0x10u64.to_le_bytes());
        oracle.extend_from_slice(&0x0506u64.to_le_bytes());
        oracle.extend_from_slice(&0x30u64.to_le_bytes());
        oracle.extend_from_slice(&1u64.to_le_bytes());
        assert_eq!(captured(&table), oracle);

        let mut target = super::LogSiteTable::default();
        target.word(0x99).unwrap().store(1, Ordering::Relaxed);
        let mut cursor = crate::state::StateCursor::new(&oracle);
        target.restore(&mut cursor).expect("restore");
        assert!(cursor.is_empty());
        assert_eq!(captured(&target), oracle, "exactly the recorded entries");
        assert_eq!(target.word(0x10).unwrap().load(Ordering::Relaxed), 0x0506);
    }

    fn refusal(bytes: &[u8]) -> String {
        let mut table = super::LogSiteTable::default();
        let mut cursor = crate::state::StateCursor::new(bytes);
        table.restore(&mut cursor).expect_err("refused").to_string()
    }

    fn section(count: u32, ids: &[u64]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"CLOG");
        out.extend_from_slice(&count.to_le_bytes());
        for id in ids {
            out.extend_from_slice(&id.to_le_bytes());
            out.extend_from_slice(&1u64.to_le_bytes());
        }
        out
    }

    #[test]
    fn a_restore_refuses_bytes_that_are_not_a_canonical_log_site_section() {
        // Bytes a changed field list left behind: the trailing-byte drift
        // signal, whether shorter than the tag or not.
        assert!(refusal(&[1, 2, 3, 4, 5]).contains("5 trailing byte(s)"));
        assert!(refusal(&[0xab]).contains("1 trailing byte(s)"));
        // Repeated and misordered ids.
        assert!(refusal(&section(2, &[5, 5])).contains("out of ascending order"));
        assert!(refusal(&section(2, &[6, 5])).contains("out of ascending order"));
        // The reserved id.
        assert!(refusal(&section(1, &[0])).contains("id 0"));
        // A section with no entries: a capture writes none for an empty
        // table, so these bytes are not one.
        assert!(refusal(&section(0, &[])).contains("no entries"));
        // More entries than a table holds.
        let ids: Vec<u64> = (1..=65).collect();
        assert!(refusal(&section(65, &ids)).contains("at most 64"));
        // Truncated.
        let mut short = section(1, &[5]);
        short.pop();
        assert!(refusal(&short).contains("truncated"));
    }

    /// A checkpoint whose 64 entries all belong to sites an edit removed:
    /// every current site still gets an entry, and the old entries ride the
    /// next capture only in the room the current sites leave.
    #[test]
    fn obsolete_checkpoint_entries_never_take_the_room_of_current_sites() {
        let slots = super::LOG_SITE_SLOTS as u64;
        let old: Vec<u64> = (1..=slots).collect();
        let mut table = super::LogSiteTable::default();
        let bytes = section(slots as u32, &old);
        let mut cursor = crate::state::StateCursor::new(&bytes);
        table
            .restore(&mut cursor)
            .expect("a full checkpoint restores");
        // 64 current sites, none of them in the checkpoint.
        let current: Vec<u64> = (1..=slots).map(|k| 1000 + k).collect();
        for &id in &current {
            let word = table.word(id).expect("a current site gets an entry");
            assert_eq!(word.load(Ordering::Relaxed), super::NEVER, "fresh");
            word.store(7, Ordering::Relaxed);
        }
        // The 65th current site is past the limit, as without a restore.
        assert!(table.word(5000).is_none());
        // The capture holds the 64 current sites and none of the old ones.
        let mut oracle = Vec::new();
        oracle.extend_from_slice(b"CLOG");
        oracle.extend_from_slice(&64u32.to_le_bytes());
        for &id in &current {
            oracle.extend_from_slice(&id.to_le_bytes());
            oracle.extend_from_slice(&7u64.to_le_bytes());
        }
        assert_eq!(captured(&table), oracle);
    }

    /// A restored entry a site claims keeps its recorded word; one no site
    /// has run since still rides the next capture when there is room, so a
    /// capture right after a restore writes the same bytes.
    #[test]
    fn a_restored_entry_keeps_its_word_and_rides_the_next_capture_until_claimed() {
        let bytes = section(2, &[0x10, 0x20]);
        let mut table = super::LogSiteTable::default();
        table
            .restore(&mut crate::state::StateCursor::new(&bytes))
            .expect("restore");
        assert_eq!(captured(&table), bytes, "an untouched restore round-trips");
        // Site 0x10 runs: it gets its recorded word, not a fresh one.
        assert_eq!(table.word(0x10).unwrap().load(Ordering::Relaxed), 1);
        // 63 more current sites fill the limit with 0x10; 0x20 is not run.
        for k in 1..super::LOG_SITE_SLOTS as u64 {
            table.word(0x1000 + k).unwrap().store(2, Ordering::Relaxed);
        }
        let capture = captured(&table);
        assert_eq!(&capture[4..8], &64u32.to_le_bytes(), "64 entries at most");
        assert!(
            !capture
                .chunks(8)
                .skip(1)
                .step_by(2)
                .any(|id| id == 0x20u64.to_le_bytes()),
            "the unrun restored entry gave its room to a current site"
        );
        // Site 0x20 is the 65th claimed site: past the limit.
        assert!(table.word(0x20).is_none());
    }
}

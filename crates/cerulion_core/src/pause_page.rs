// SPDX-License-Identifier: AGPL-3.0-only
//! The PAUSE PAGE: one word of cross-process run state that lets
//! `cerulion graph pause` hold a live run and stop its clock.
//!
//! # What a pause is
//!
//! A paused run steps no node, and its RUN CLOCK stands still. The run clock is
//! the hardware monotonic clock ([`crate::clock::real_ns`]) minus every
//! nanosecond the run has spent paused, so a `Period` deadline, a message
//! timestamp and a recording all read the same value on the first tick after a
//! resume that they read on the last tick before the pause. Nothing is skipped
//! and nothing is caught up. Stopping the process instead (`SIGSTOP`) does neither:
//! the hardware clock keeps running, so the timestamps jump by the length of the
//! stop and the timers burst on resume.
//!
//! # The form: a frozen value and an offset
//!
//! The page holds two clock words and a counter, and no lock:
//!
//! * `frozen_ns`: `0` while the run is live; while paused, the run-clock value the
//!   run is frozen at (never `0`, a would-be zero is stored as `1`).
//! * `offset_ns`: the total time the run has spent paused, as of the last resume.
//!   Read only while the run is live.
//! * `epoch`: bumped on every transition, so an observer can tell that a pause
//!   happened inside a window it was waiting through even when the run is live
//!   again by the time it looks.
//!
//! [`PausePage::run_clock_ns`] is `frozen_ns` if it is set, else
//! `real_ns() - offset_ns`.
//!
//! **Every single store leaves a valid state**, which is why no seqlock is needed
//! and a verb killed half way through cannot wedge anything. Pausing is ONE
//! compare-and-swap (`frozen_ns` from `0` to the frozen value); a reader sees
//! either the live state or the paused one. Resuming stores the new offset FIRST,
//! while `frozen_ns` still holds the run frozen (the offset is not read in that
//! state), and clears `frozen_ns` LAST. A resume killed between the two stores
//! leaves a run that still reads as paused, and running the verb again finishes it.
//! The new offset is `real_ns() - frozen_ns`, which makes the clock continue from
//! exactly the frozen value, so it never goes backwards and never jumps forward.
//!
//! The page has no lock of its own, and two resumes racing could move the clock
//! back. `cerulion graph pause` and `resume` therefore take turns: each holds a
//! lock on the run directory across its flip of the page and its mirror into
//! `run.json`. A caller that flips the page without that lock owns the exclusion.
//!
//! # Scope, stated
//!
//! This page is the run's CONTROL word. It does not pause anything by itself: the
//! live loop ([`crate::graph::GraphRuntime::attach_pause`]) holds at its next step
//! boundary and the run's clock ([`crate::clock::PausableClock`]) reads it.
//!
//! It is a page-class object like [`crate::wedge_page`]: 4 KiB, named from the run
//! identity, removed by its owner on a controlled exit. A run that is killed
//! outright leaves the name behind, and the stale-run sweep unlinks it: the name is
//! a function of the dead run's identity, so the sweep can derive it from the run
//! directory it is already reclaiming, and no other run can own it.
//!
//! NOTE: this module is compiled only on Unix; the `#[cfg(unix)]` gate lives on its
//! `pub mod pause_page;` declaration in `lib.rs`.
//!
//! The POSIX mechanics are the shared substrate's (`crate::shm_map`), exactly as
//! for `state_arm`, `credit`, `barrier` and `wedge_page`.

use std::sync::atomic::{AtomicU64, Ordering};

use crate::clock::real_ns;
use crate::shm_map::{create_exclusive, fnv1a64, unlink, unmap, OpenedSegment};

/// Distinctive magic identifying a Cerulion pause page: ASCII `"CERPAUSE"`.
const MAGIC: u64 = 0x4345_5250_4155_5345;

/// On-page format version. Bumped on any layout change.
pub const PAUSE_PAGE_VERSION: u32 = 1;

/// The mapped size of a pause page: one 4 KiB region.
pub const PAUSE_PAGE_BYTES: usize = 4096;

/// The cross-process pause page.
///
/// `#[repr(C)]`, atomics plus two write-once header fields, so it maps identically
/// across processes. Accessed only through a pointer into the mapped segment.
#[repr(C)]
pub struct PausePage {
    /// [`MAGIC`] once fully initialised; `Release`-stored LAST at create and
    /// `Acquire`-loaded FIRST by an opener.
    magic: AtomicU64,
    /// [`PAUSE_PAGE_VERSION`].
    version: u32,
    /// Layout padding; never read.
    _reserved: u32,
    /// `0` while live, else the run-clock value the run is frozen at.
    frozen_ns: AtomicU64,
    /// Total paused time as of the last resume; read only while live.
    offset_ns: AtomicU64,
    /// Bumped on every transition.
    epoch: AtomicU64,
    /// Explicit padding out to the page. Never read; layout only.
    _pad: [u64; (PAUSE_PAGE_BYTES / 8) - 5],
}

const _: () = assert!(std::mem::align_of::<PausePage>() == 8);
const _: () = assert!(std::mem::offset_of!(PausePage, magic) == 0);
const _: () = assert!(std::mem::offset_of!(PausePage, version) == 8);
const _: () = assert!(std::mem::offset_of!(PausePage, frozen_ns) == 16);
const _: () = assert!(std::mem::offset_of!(PausePage, offset_ns) == 24);
const _: () = assert!(std::mem::offset_of!(PausePage, epoch) == 32);
const _: () = assert!(std::mem::size_of::<PausePage>() == PAUSE_PAGE_BYTES);

/// What a [`PausePage::pause`] or [`PausePage::resume`] call did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PauseTransition {
    /// This call moved the run to the requested state.
    Changed,
    /// The run was already in the requested state (or another caller moved it
    /// first); nothing changed.
    Unchanged,
}

impl PausePage {
    /// Initialise a freshly-created, exclusively-owned, zero-filled mapping.
    ///
    /// The magic is stored LAST with `Release`, so a racing opener either fails its
    /// magic check or observes every field below it.
    fn init(&self) {
        // SAFETY: `self` is a freshly-created, exclusively-owned mapping; the two
        // plain fields are written exactly once, before the magic publishes them.
        unsafe {
            let me = self as *const Self as *mut Self;
            (*me).version = PAUSE_PAGE_VERSION;
            (*me)._reserved = 0;
        }
        self.frozen_ns.store(0, Ordering::Relaxed);
        self.offset_ns.store(0, Ordering::Relaxed);
        self.epoch.store(0, Ordering::Relaxed);
        self.magic.store(MAGIC, Ordering::Release);
    }

    /// Validate a mapped page an opener did not create.
    fn validate(&self) -> Result<(), String> {
        if self.magic.load(Ordering::Acquire) != MAGIC {
            return Err("magic is not set: the segment is not a pause page, or its \
                        creator has not finished initialising it"
                .to_string());
        }
        if self.version != PAUSE_PAGE_VERSION {
            return Err(format!(
                "version {} is not {PAUSE_PAGE_VERSION}",
                self.version
            ));
        }
        Ok(())
    }

    /// Whether the run is paused right now.
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.frozen_ns.load(Ordering::Acquire) != 0
    }

    /// How many transitions (pauses plus resumes) this page has seen.
    ///
    /// An observer that waits through a window (a barrier wait, say) snapshots this
    /// at the start and compares it at the end: a different value means a pause
    /// overlapped the window even if the run is live again.
    #[must_use]
    pub fn epoch(&self) -> u64 {
        self.epoch.load(Ordering::Acquire)
    }

    /// The run clock, in nanoseconds: the hardware monotonic clock minus the time
    /// the run has spent paused, held still while it is paused.
    ///
    /// Monotonic across a pause and a resume, and the SAME value in every process
    /// that maps this page (they share the hardware clock and the offset).
    #[must_use]
    pub fn run_clock_ns(&self) -> u64 {
        let frozen = self.frozen_ns.load(Ordering::Acquire);
        if frozen != 0 {
            return frozen;
        }
        real_ns().saturating_sub(self.offset_ns.load(Ordering::Acquire))
    }

    /// The total time the run has spent paused, including a pause still in
    /// progress, in nanoseconds. Never decreases.
    #[must_use]
    pub fn paused_ns(&self) -> u64 {
        let frozen = self.frozen_ns.load(Ordering::Acquire);
        if frozen != 0 {
            return real_ns().saturating_sub(frozen);
        }
        self.offset_ns.load(Ordering::Acquire)
    }

    /// Pause the run: freeze the run clock at its current value.
    ///
    /// One compare-and-swap, so a call that finds the run already paused changes
    /// nothing.
    pub fn pause(&self) -> PauseTransition {
        let offset = self.offset_ns.load(Ordering::Acquire);
        // A frozen value of `0` would read as "live", so the floor is 1.
        let frozen = real_ns().saturating_sub(offset).max(1);
        match self
            .frozen_ns
            .compare_exchange(0, frozen, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => {
                self.epoch.fetch_add(1, Ordering::Release);
                PauseTransition::Changed
            }
            Err(_) => PauseTransition::Unchanged,
        }
    }

    /// Resume the run: the clock continues from the value it was frozen at.
    ///
    /// The new offset is stored while the run is still frozen and the frozen word
    /// is cleared last, so every intermediate state is a valid one (see the module
    /// docs). The epoch is bumped BEFORE the run reads as live, so an observer
    /// never sees a live run whose pause it has no way to detect.
    pub fn resume(&self) -> PauseTransition {
        let frozen = self.frozen_ns.load(Ordering::Acquire);
        if frozen == 0 {
            return PauseTransition::Unchanged;
        }
        self.offset_ns
            .store(real_ns().saturating_sub(frozen), Ordering::Release);
        self.epoch.fetch_add(1, Ordering::Release);
        match self
            .frozen_ns
            .compare_exchange(frozen, 0, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => PauseTransition::Changed,
            Err(_) => PauseTransition::Unchanged,
        }
    }
}

/// Derive the pause page's tag for a run: the run identity as 32 lowercase hex
/// digits, the spelling `run.json` and the run registry already use.
///
/// One function, so the supervisor that creates the page, the worker that opens
/// it and the CLI verb that flips it cannot spell the same run three ways.
#[must_use]
pub fn pause_tag_for_run(run_id: u128) -> String {
    format!("{run_id:032x}")
}

/// Derive the POSIX SHM object name for `tag`: `/cer_pau_<fnv1a64(tag):016x>`.
///
/// Fixed-length hex keeps the name PREFIX-FREE against `/cer_rg_`, `/cer_sta_`,
/// `/cer_bar_`, `/cer_db_` and `/cer_wdg_` (there is a known iceoryx2 hazard with
/// string-prefix collisions) and within macOS's 31-character limit
/// (`/cer_pau_` is 9, plus 16). Pure, so it is testable on any OS.
#[must_use]
pub fn pause_page_shm_name(tag: &str) -> String {
    format!("/cer_pau_{:016x}", fnv1a64(tag.as_bytes()))
}

/// A process-shared SHM mapping of a [`PausePage`].
///
/// Construct via [`create_owned`](MappedPausePage::create_owned) (the run's owner:
/// creates, initialises, and `shm_unlink`s the name on drop) or
/// [`open_unowned`](MappedPausePage::open_unowned) (a worker or the CLI verb:
/// STRICT open-existing, maps only). [`Deref`](std::ops::Deref)s to the page.
///
/// As with the other page classes, `shm_unlink` removes only the NAME, so an owner
/// dropping mid-run cannot invalidate a peer's live mapping.
#[must_use = "the mapping is unmapped (and, if owned, shm_unlink'd) on drop: bind it to a named local"]
pub struct MappedPausePage {
    ptr: *mut PausePage,
    name: std::ffi::CString,
    name_str: String,
    owns_name: bool,
}

// SAFETY: the mapped object is a `PausePage` (atomics plus two fields written once
// before the magic publishes them) in a shared page. All post-init access goes
// through atomic ops, so sending or sharing the handle across threads is sound.
unsafe impl Send for MappedPausePage {}
unsafe impl Sync for MappedPausePage {}

impl MappedPausePage {
    /// Create, map and initialise the pause page for `tag` as its OWNER.
    ///
    /// A pre-existing orphan of the same name is `shm_unlink`ed first so the
    /// `O_EXCL` create yields a fresh, zero-filled object.
    pub fn create_owned(tag: &str) -> std::io::Result<Self> {
        let name_str = pause_page_shm_name(tag);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        let addr = create_exclusive(&name, PAUSE_PAGE_BYTES)?;
        let ptr = addr as *mut PausePage;
        // SAFETY: `ptr` is a freshly-mapped, exclusively-owned (O_EXCL),
        // page-aligned, zero-filled region of exactly `size_of::<PausePage>()`
        // (const-asserted above).
        unsafe { (*ptr).init() };
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: true,
        })
    }

    /// Open and map an EXISTING pause page for `tag` as a peer.
    ///
    /// STRICT open-existing (no `O_CREAT`), so a missing object is an error rather
    /// than a silently-created page nobody initialised. The header is validated
    /// (magic and version) before the handle is returned.
    pub fn open_unowned(tag: &str) -> std::io::Result<Self> {
        let name_str = pause_page_shm_name(tag);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        let seg = OpenedSegment::open(&name)?;
        // SIZE CHECK before the map: a creator that died between `shm_open` and
        // `ftruncate` leaves a zero-length object whose mapping faults on first
        // touch (a SIGBUS on Linux), with no error to attribute.
        let size = seg.size()?;
        if (size as usize) < PAUSE_PAGE_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "pause page '{name_str}' is {size} bytes, short of the \
                     {PAUSE_PAGE_BYTES}-byte page"
                ),
            ));
        }
        let addr = seg.map_shared(PAUSE_PAGE_BYTES)?;
        let ptr = addr as *mut PausePage;
        // SAFETY: `ptr` maps at least PAUSE_PAGE_BYTES of a live object; `validate`
        // reads only initialised-or-zero words.
        if let Err(reason) = unsafe { (*ptr).validate() } {
            // SAFETY: unmap exactly what was just mapped; nothing references it.
            unsafe { unmap(addr, PAUSE_PAGE_BYTES) };
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("pause page '{name_str}' failed validation: {reason}"),
            ));
        }
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: false,
        })
    }

    /// The POSIX SHM object name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name_str
    }
}

impl std::ops::Deref for MappedPausePage {
    type Target = PausePage;
    fn deref(&self) -> &PausePage {
        // SAFETY: `ptr` is a valid mapping of an initialised `PausePage` that stays
        // mapped for as long as `self` lives (unmapped only in `Drop`).
        unsafe { &*self.ptr }
    }
}

impl std::fmt::Debug for MappedPausePage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MappedPausePage")
            .field("name", &self.name_str)
            .field("owns_name", &self.owns_name)
            .finish()
    }
}

impl Drop for MappedPausePage {
    fn drop(&mut self) {
        // SAFETY: unmap exactly the region create_owned/open_unowned mapped;
        // nothing references it after this.
        unsafe {
            unmap(self.ptr as *mut std::ffi::c_void, PAUSE_PAGE_BYTES);
        }
        if self.owns_name {
            // Removes the NAME only: a peer's live mapping survives until its own
            // munmap.
            unlink(&self.name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn tag(what: &str) -> String {
        format!("pause_{what}_{}", std::process::id())
    }

    #[test]
    fn the_name_is_prefix_free_short_enough_for_macos_and_stable() {
        let a = pause_page_shm_name("run_a");
        assert_eq!(a, pause_page_shm_name("run_a"), "the name is stable");
        assert_ne!(a, pause_page_shm_name("run_b"));
        assert_eq!(a.len(), 25, "PSHMNAMLEN is 31 on macOS: {a}");
        assert!(a.starts_with("/cer_pau_"), "{a}");
    }

    #[test]
    fn the_tag_is_the_run_identity_in_the_spelling_run_json_uses() {
        assert_eq!(
            pause_tag_for_run(0x2a),
            "0000000000000000000000000000002a",
            "32 lowercase hex digits, no 0x"
        );
    }

    #[test]
    fn a_fresh_page_is_live_with_a_running_clock_and_no_paused_time() {
        let owner = MappedPausePage::create_owned(&tag("fresh")).expect("create");
        assert!(!owner.is_paused());
        assert_eq!(owner.paused_ns(), 0);
        assert_eq!(owner.epoch(), 0);
        let a = owner.run_clock_ns();
        std::thread::sleep(Duration::from_millis(5));
        assert!(owner.run_clock_ns() > a, "a live run clock advances");
    }

    #[test]
    fn a_pause_freezes_the_clock_and_a_resume_continues_it_without_a_jump() {
        let owner = MappedPausePage::create_owned(&tag("freeze")).expect("create");
        let before = owner.run_clock_ns();
        assert_eq!(owner.pause(), PauseTransition::Changed);
        assert!(owner.is_paused());
        let frozen = owner.run_clock_ns();
        assert!(frozen >= before, "pausing never moves the clock back");
        std::thread::sleep(Duration::from_millis(60));
        assert_eq!(
            owner.run_clock_ns(),
            frozen,
            "the run clock must not advance while paused"
        );
        assert!(
            owner.paused_ns() >= 50_000_000,
            "paused time accrues while paused: {}",
            owner.paused_ns()
        );
        assert_eq!(owner.resume(), PauseTransition::Changed);
        assert!(!owner.is_paused());
        let after = owner.run_clock_ns();
        assert!(after >= frozen, "resuming never moves the clock back");
        assert!(
            after - frozen < 30_000_000,
            "the clock must continue from the frozen value, not jump by the 60 ms pause: \
             frozen={frozen} after={after}"
        );
        assert!(
            owner.paused_ns() >= 50_000_000,
            "the paused time is kept after the resume"
        );
    }

    #[test]
    fn pausing_twice_and_resuming_twice_change_nothing_the_second_time() {
        let owner = MappedPausePage::create_owned(&tag("idem")).expect("create");
        assert_eq!(
            owner.resume(),
            PauseTransition::Unchanged,
            "resuming a live run is a no-op"
        );
        assert_eq!(owner.epoch(), 0);
        assert_eq!(owner.pause(), PauseTransition::Changed);
        let frozen = owner.run_clock_ns();
        assert_eq!(owner.pause(), PauseTransition::Unchanged);
        assert_eq!(
            owner.run_clock_ns(),
            frozen,
            "a second pause must not re-freeze the clock at a later value"
        );
        assert_eq!(owner.resume(), PauseTransition::Changed);
        assert_eq!(owner.resume(), PauseTransition::Unchanged);
    }

    #[test]
    fn every_transition_bumps_the_epoch_and_a_noop_does_not() {
        let owner = MappedPausePage::create_owned(&tag("epoch")).expect("create");
        assert_eq!(owner.epoch(), 0);
        owner.pause();
        assert_eq!(owner.epoch(), 1);
        owner.pause();
        assert_eq!(owner.epoch(), 1, "a no-op pause must not bump the epoch");
        owner.resume();
        assert_eq!(owner.epoch(), 2);
        owner.resume();
        assert_eq!(owner.epoch(), 2);
    }

    #[test]
    fn paused_time_adds_up_across_two_pauses() {
        let owner = MappedPausePage::create_owned(&tag("sum")).expect("create");
        owner.pause();
        std::thread::sleep(Duration::from_millis(30));
        owner.resume();
        let first = owner.paused_ns();
        assert!(first >= 25_000_000, "{first}");
        owner.pause();
        std::thread::sleep(Duration::from_millis(30));
        owner.resume();
        assert!(
            owner.paused_ns() >= first + 25_000_000,
            "the second pause adds to the first: {} then {}",
            first,
            owner.paused_ns()
        );
    }

    #[test]
    fn the_run_clock_never_goes_backwards_across_many_cycles() {
        let owner = MappedPausePage::create_owned(&tag("mono")).expect("create");
        let mut last = owner.run_clock_ns();
        for _ in 0..200 {
            owner.pause();
            let a = owner.run_clock_ns();
            assert!(a >= last, "pause moved the clock back: {last} -> {a}");
            owner.resume();
            let b = owner.run_clock_ns();
            assert!(b >= a, "resume moved the clock back: {a} -> {b}");
            last = b;
        }
    }

    #[test]
    fn a_peer_observes_the_owners_pause_and_the_same_run_clock() {
        let t = tag("peer");
        let owner = MappedPausePage::create_owned(&t).expect("create");
        let peer = MappedPausePage::open_unowned(&t).expect("open");
        assert!(!peer.is_paused());
        assert_eq!(
            peer.pause(),
            PauseTransition::Changed,
            "a peer (the CLI verb) can pause the run the owner created"
        );
        assert!(owner.is_paused(), "the owner must observe the peer's pause");
        assert_eq!(
            owner.run_clock_ns(),
            peer.run_clock_ns(),
            "both mappings read one frozen value"
        );
        assert_eq!(owner.resume(), PauseTransition::Changed);
        assert!(!peer.is_paused());
        assert_eq!(peer.epoch(), owner.epoch());
    }

    #[test]
    fn a_resume_killed_before_it_clears_the_frozen_word_leaves_the_run_paused_and_retryable() {
        let owner = MappedPausePage::create_owned(&tag("crash")).expect("create");
        owner.pause();
        // The first half of a resume, exactly as `resume` performs it, and then the
        // verb dies: the offset has moved but the frozen word has not been cleared.
        let frozen = owner.frozen_ns.load(Ordering::Acquire);
        owner
            .offset_ns
            .store(real_ns().saturating_sub(frozen), Ordering::Release);
        assert!(
            owner.is_paused(),
            "a half-finished resume must still read as paused"
        );
        assert_eq!(owner.run_clock_ns(), frozen, "and the clock stays frozen");
        assert_eq!(
            owner.resume(),
            PauseTransition::Changed,
            "running the verb again finishes the resume"
        );
        assert!(!owner.is_paused());
    }

    #[test]
    fn opening_a_page_nobody_created_errs_rather_than_creating_one() {
        let err = MappedPausePage::open_unowned(&tag("absent")).expect_err("must not create");
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound, "{err}");
    }

    #[test]
    fn dropping_the_owner_removes_the_name_but_not_a_peers_live_mapping() {
        let t = tag("drop");
        let owner = MappedPausePage::create_owned(&t).expect("create");
        let peer = MappedPausePage::open_unowned(&t).expect("open");
        drop(owner);
        assert!(
            MappedPausePage::open_unowned(&t).is_err(),
            "the owner's drop unlinks the name"
        );
        assert_eq!(
            peer.pause(),
            PauseTransition::Changed,
            "the peer still maps it"
        );
    }
}

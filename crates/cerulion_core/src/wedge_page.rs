// SPDX-License-Identifier: AGPL-3.0-only
//! The WEDGE PAGE — the cross-process transport a supervisor watches
//! to learn that one of its workers has entered a tick and not come back.
//!
//! # Why this exists at all
//!
//! `tick_within_ms` times only ticks that RETURN: the scheduler captures an
//! `Instant` before the callback and reads `elapsed()` after it, so a tick that
//! never returns is timed by nobody and counted by nothing. The one thing in the
//! system that noticed a never-returning tick was the level barrier — a peer that
//! stops arriving poisons the runtime after `BARRIER_BOUNDARY_TIMEOUT` — and the
//! free-run arc DELETES that barrier. This page is its replacement, and it is
//! deliberately independent of the substrate switch: it lands on today's lockstep
//! main and keeps working after free-run, because it observes the SCHEDULER's fire
//! wrap rather than any coordination structure.
//!
//! # The form is a SEQ PAIR, and that is the whole design
//!
//! Each node slot holds two monotone counters: `entered_seq`, bumped immediately
//! after the fire's `Instant` is captured, and `exited_seq`, bumped beside the
//! post-callback `elapsed_ns` read. So `entered == exited` means "not inside a
//! tick" and `entered != exited` means "inside one", and neither word carries a
//! TIME.
//!
//! That is not a simplification, it is the correctness argument. The observer is a
//! DIFFERENT PROCESS: under the multi-process default each worker runs on its
//! own `VirtualClock` starting near zero while the supervisor is on wall time, so a
//! stamp written by a worker and compared against the supervisor's clock compares
//! two unrelated number lines — the clock-domain rule; forgetting it makes
//! such a gate silently inert on a deployed robot. What the
//! supervisor CAN do is notice that a pair has not changed between two of ITS OWN
//! observations and accumulate dwell on ITS OWN clock. That is the state-carrier
//! watchdog's rule verbatim: **the reaper thread owns the clock; the rule owns the
//! decision** (`crate::state_carrier::watchdog`), which is also what keeps the
//! deciding half a pure state machine an oracle vector can drive at exact
//! boundaries instead of a test that asserts a wall (the load-fragile class).
//!
//! # The companion word: a rank wedged OUTSIDE any tick
//!
//! The per-slot rule is blind to a worker whose loop stops between ticks — nothing
//! is inside a tick, so nothing dwells. [`WedgePage::step_progress`] is a per-rank
//! counter the scheduler bumps once per step, so a rank whose loop stops entirely
//! alarms too. A producer legitimately PARKED on exhausted block credit keeps
//! advancing it (the park returns on each recheck), so a block-paced graph does not
//! spam the alarm — which is why the companion is a step counter rather than a
//! "the rank is idle" flag.
//!
//! **The composed blind spot, named rather than left to be discovered**: a
//! producer wedged on a DEAD consumer's credit edge is missed by BOTH halves — the
//! defer is a PRE-FIRE gate so the fire never enters a tick, and the park returns
//! each recheck so the progress word keeps moving. That case is not this alarm's,
//! by design: it is indistinguishable here from legitimate slow-consumer pacing,
//! and it is owned by the supervisor's per-edge credit-death warn.
//!
//! # Layout discipline
//!
//! `#[repr(C)]`, atomics-only, ONE 4 KiB page, magic `Release`-stored LAST — the
//! [`crate::state_arm`] template, which is itself [`crate::barrier`]'s. An opener
//! either fails clean or observes a fully initialised page. Offsets are
//! const-asserted, so a layout edit cannot silently re-point a peer's slot.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on
//! its `pub mod wedge_page;` declaration in `lib.rs`.
//!
//! # The POSIX mechanics are the SHARED substrate
//!
//! The `shm_unlink`-first / `O_EXCL` create / `ftruncate`-once / `mmap(MAP_SHARED)`
//! sequence is `shm_map`'s, read through exactly as `state_arm`, `credit`,
//! `barrier` and `shm_ring` read it — one substrate, not a fifth hand-rolled
//! copy of it. What stays
//! HERE is everything the substrate deliberately does not own: the `slot_count`
//! ceiling and its refusal, the SIZE threshold and its diagnostic, and the
//! magic/version/slot-count refusal in `WedgePage::validate` (private, so a prose
//! mention — rustdoc refuses a public→private link) — the `credit.rs` split,
//! **the substrate reports, the site decides**.
//!
//! The substrate carries ONE behavioural property, and it is the one `shm_map`'s own
//! docs call out as the point of sharing it: on a FAILED `mmap` the errno is
//! captured BEFORE the `close(fd)`. POSIX leaves `errno` unspecified after a
//! SUCCESSFUL call, so a hand-rolled order (close, then `last_os_error()`) is
//! unsound BY CONTRACT — an implementation is free to clobber it and report a
//! mapping failure as something else. This module reports the mapping failure
//! the same way the other four sites do.
//!
//! **Stated at its real strength, because it was MEASURED rather than assumed:**
//! on macOS a successful `close(2)` PRESERVES `errno`, so on this platform the two
//! orderings are indistinguishable, and reversing them to close before capturing
//! errno still passes every test in the repo. The ordering is therefore a
//! CONTRACT-correctness property here, not an observed failure, and nothing in this
//! file pins it — see `shm_map`'s module docs for the measurement and for why its
//! own pin cannot see it either.

use std::sync::atomic::{AtomicU64, Ordering};

use crate::shm_map::{create_exclusive, fnv1a64, unlink, unmap, OpenedSegment};

/// Distinctive magic identifying a Cerulion wedge page: ASCII `"CERWEDGE"`.
const MAGIC: u64 = 0x4345_5257_4544_4745;

/// On-page format version. Bumped on any layout change.
pub const WEDGE_PAGE_VERSION: u32 = 1;

/// The mapped size of a wedge page: one 4 KiB region.
///
/// One page per RANK, sized once at create — the same "its own mapping, holding
/// nothing else" discipline [`crate::state_arm`] needs for its `MADV_DONTFORK`
/// sweep, and for the same reason (a capture child must not carry it).
pub const WEDGE_PAGE_BYTES: usize = 4096;

/// Bytes reserved for the header before the slot table starts.
///
/// One cache line, so the header words never false-share with slot 0.
const WEDGE_HEADER_BYTES: usize = 64;

/// One slot's stride, in bytes: ONE cache line.
///
/// The two counters are 16 bytes of state padded to 64 so no two nodes' slots
/// share a line. Without the padding four slots ride one line, and a
/// `PARALLEL_FIRE_THRESHOLD`-wide level firing on rayon pays a cross-core
/// read-for-ownership on every mark — four nodes ping-ponging one line for two
/// counters that are never read together. The cost is slots, and slots are the
/// thing this page has in surplus (see [`WEDGE_MAX_SLOTS`]).
const WEDGE_SLOT_STRIDE: usize = 64;

/// The most node slots one page can hold: `(4096 - 64) / 64`.
///
/// A worker beyond this is REFUSED a page (loudly) rather than silently given a
/// truncated one — under the process-per-node default a worker holds ONE
/// node, and the fused case is bounded by the partitioner's budget, so 63 is far
/// above any shipping split (`humanoid_mp` is 36 nodes for the WHOLE graph, i.e.
/// the whole graph fits in one rank's table with room to spare).
pub const WEDGE_MAX_SLOTS: usize = (WEDGE_PAGE_BYTES - WEDGE_HEADER_BYTES) / WEDGE_SLOT_STRIDE;

/// One node's in-tick seq pair, padded to one cache line.
///
/// (The stride is the private `WEDGE_SLOT_STRIDE`; named in prose rather than
/// linked, because an intra-doc link from a `pub` item to a private one fails
/// CI's `RUSTDOCFLAGS=-D warnings` docs gate.)
///
/// Two counters rather than one flag, because a flag cannot distinguish "still in
/// the tick I saw last time" from "in a DIFFERENT tick now" — and that distinction
/// is the entire discrimination between a wedged node and a busy one. Both are
/// monotone (`fetch_add`), so a repeated pair proves no fire boundary was crossed
/// between two observations.
///
/// The padding is FALSE-SHARING avoidance, not alignment hygiene: these words are
/// written on the fire path of every node in the rank, concurrently under the
/// within-level rayon fire, and unpadded they pack four-to-a-line.
#[repr(C)]
pub struct WedgeSlot {
    /// Bumped immediately after the fire's `Instant` is captured, BEFORE the
    /// `catch_unwind` that runs the node's callback.
    entered_seq: AtomicU64,
    /// Bumped beside the post-callback `elapsed_ns` read. Control reaches that
    /// point even on a CAUGHT PANIC — the panic is contained by the `catch_unwind`
    /// above it, which is the same invariant the replay-suppress clear rests
    /// on three lines earlier in the same function.
    exited_seq: AtomicU64,
    /// Explicit padding out to one cache line. Never read; layout only.
    _pad: [u64; (WEDGE_SLOT_STRIDE / 8) - 2],
}

/// The cross-process wedge page.
///
/// `#[repr(C)]`, atomics + two write-once header fields, so it maps identically
/// across processes. Accessed only through a pointer into the mapped segment.
#[repr(C)]
pub struct WedgePage {
    /// [`MAGIC`] once fully initialised. `Release`-stored as the LAST write at
    /// create and `Acquire`-loaded FIRST by an opener.
    magic: AtomicU64,
    /// [`WEDGE_PAGE_VERSION`].
    version: u32,
    /// How many of [`Self::slots`] this page's creator sized for — so an opener
    /// refuses a page whose table it would index past, and the supervisor's
    /// observer walks exactly the slots the worker writes.
    slot_count: u32,
    /// The per-rank STEP counter — the companion word for a rank wedged OUTSIDE any
    /// tick. Bumped once per scheduler step.
    step_progress: AtomicU64,
    /// Explicit padding so the slot table starts at [`WEDGE_HEADER_BYTES`].
    /// Never read; layout only.
    _pad: [u64; 5],
    /// The per-node slot table.
    slots: [WedgeSlot; WEDGE_MAX_SLOTS],
}

const _: () = assert!(std::mem::size_of::<WedgeSlot>() == WEDGE_SLOT_STRIDE);
const _: () = assert!(std::mem::align_of::<WedgeSlot>() == 8);
const _: () = assert!(std::mem::align_of::<WedgePage>() == 8);
const _: () = assert!(std::mem::offset_of!(WedgePage, magic) == 0);
const _: () = assert!(std::mem::offset_of!(WedgePage, version) == 8);
const _: () = assert!(std::mem::offset_of!(WedgePage, slot_count) == 12);
const _: () = assert!(std::mem::offset_of!(WedgePage, step_progress) == 16);
const _: () = assert!(std::mem::offset_of!(WedgePage, slots) == WEDGE_HEADER_BYTES);
const _: () = assert!(std::mem::size_of::<WedgePage>() <= WEDGE_PAGE_BYTES);
// The table fills the page exactly — a smaller `WEDGE_MAX_SLOTS` would waste
// mapped bytes, a larger one would not fit.
const _: () = assert!(std::mem::size_of::<WedgePage>() == WEDGE_PAGE_BYTES);

/// One observation of a node slot, as the SUPERVISOR reads it.
///
/// A plain value rather than a borrow of the mapped page, for the reason
/// [`crate::state_carrier::watchdog::ProgressReading`] gives: the classifier is
/// pure, and a rule that could re-read the page mid-decision would be deciding
/// against two different observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotReading {
    /// The slot's `entered_seq` at the instant of the read.
    pub entered: u64,
    /// The slot's `exited_seq` at the instant of the read.
    pub exited: u64,
}

impl SlotReading {
    /// Whether this reading says the node is INSIDE a tick.
    ///
    /// `entered > exited` only. A reading where `exited` EXCEEDS `entered` is not
    /// a state the writer can produce (both counters are bumped by one thread, in
    /// order, entry first), so it can only be a torn read across a fire boundary
    /// or a page that is not the one this node writes — and the conservative
    /// answer to both is "not in a tick", which costs at most one observation of
    /// delay and can never fabricate a wedge.
    pub fn in_tick(self) -> bool {
        self.entered > self.exited
    }
}

impl WedgePage {
    /// Initialise a freshly-created, exclusively-owned mapping.
    ///
    /// The magic is stored LAST with `Release`, so a racing opener either fails
    /// its magic check or observes every field below it.
    ///
    /// # Safety
    ///
    /// `self` must be a freshly-created, exclusively-owned mapping of at least
    /// [`WEDGE_PAGE_BYTES`]. `slot_count` must be `<= WEDGE_MAX_SLOTS`.
    fn init(&self, slot_count: u32) {
        // SAFETY: `self` is a freshly-created, exclusively-owned mapping; these
        // two plain fields are written exactly once, before the magic publishes
        // them, so no peer can observe the interior mutation.
        unsafe {
            let me = self as *const Self as *mut Self;
            (*me).version = WEDGE_PAGE_VERSION;
            (*me).slot_count = slot_count;
        }
        self.step_progress.store(0, Ordering::Relaxed);
        for slot in &self.slots {
            slot.entered_seq.store(0, Ordering::Relaxed);
            slot.exited_seq.store(0, Ordering::Relaxed);
        }
        self.magic.store(MAGIC, Ordering::Release);
    }

    /// Validate a mapped page an opener did not create.
    fn validate(&self) -> Result<(), String> {
        if self.magic.load(Ordering::Acquire) != MAGIC {
            return Err(
                "magic is not set — the segment is not a wedge page, or its \
                        creator has not finished initialising it"
                    .to_string(),
            );
        }
        if self.version != WEDGE_PAGE_VERSION {
            return Err(format!(
                "version {} is not {WEDGE_PAGE_VERSION}",
                self.version
            ));
        }
        if self.slot_count as usize > WEDGE_MAX_SLOTS {
            return Err(format!(
                "slot_count {} exceeds the {WEDGE_MAX_SLOTS}-slot table — this build \
                 would index past the page",
                self.slot_count
            ));
        }
        Ok(())
    }

    /// How many node slots this page was sized for.
    pub fn slot_count(&self) -> usize {
        self.slot_count as usize
    }

    /// ENTER a tick on `slot` — one `Release` `fetch_add`.
    ///
    /// Out-of-range slots are IGNORED rather than panicking: this runs on the fire
    /// path of a shipping robot, and a bookkeeping mistake in a diagnostic must
    /// never be able to end a graph (the same posture the discard latch
    /// and the notify latch take at their own hot sites).
    pub fn enter(&self, slot: usize) {
        if let Some(s) = self.slot(slot) {
            s.entered_seq.fetch_add(1, Ordering::Release);
        }
    }

    /// EXIT the tick on `slot` — the matching `Release` `fetch_add`.
    pub fn exit(&self, slot: usize) {
        if let Some(s) = self.slot(slot) {
            s.exited_seq.fetch_add(1, Ordering::Release);
        }
    }

    /// Read `slot`'s pair, or `None` if the index is past this page's table.
    ///
    /// `exited` is read FIRST. The two loads cannot be atomic together, so a
    /// concurrent fire boundary can land between them; reading the EXIT side first
    /// means the worst interleaving yields an `entered` from a LATER fire than the
    /// `exited`, i.e. `entered > exited`, i.e. "in a tick" — which the classifier
    /// then resolves on the NEXT observation because the pair will have changed.
    /// The other order can yield `exited > entered`, a state no writer produces.
    pub fn read_slot(&self, slot: usize) -> Option<SlotReading> {
        let s = self.slot(slot)?;
        let exited = s.exited_seq.load(Ordering::Acquire);
        let entered = s.entered_seq.load(Ordering::Acquire);
        Some(SlotReading { entered, exited })
    }

    /// Bump this rank's step counter — the companion word.
    pub fn advance_step(&self) {
        self.step_progress.fetch_add(1, Ordering::Release);
    }

    /// This rank's step counter.
    pub fn step_progress(&self) -> u64 {
        self.step_progress.load(Ordering::Acquire)
    }

    fn slot(&self, slot: usize) -> Option<&WedgeSlot> {
        if slot >= self.slot_count as usize {
            return None;
        }
        self.slots.get(slot)
    }
}

/// Derive the POSIX SHM object name for `(tag, rank)`:
/// `/cer_wdg_<fnv1a64(tag#rank):016x>`.
///
/// Fixed-length hex keeps the name PREFIX-FREE against `shm_ring`'s `/cer_rg_`,
/// `state_arm`'s `/cer_sta_`, `barrier`'s `/cer_bar_` and `doorbell`'s `/cer_db_`
/// (there is a known iceoryx2 hazard with string-prefix collisions) and ≤ 31 chars
/// for macOS's `PSHMNAMLEN` (`/cer_wdg_` = 9 + 16 = 25). Pure — hermetically
/// testable on any OS.
///
/// The RANK is folded in, not appended, so one deployment's pages are one page per
/// worker: each is sized to ITS OWN node count, and the observer that reads a
/// rank's page is reading exactly the process it is judging.
///
/// The fold is the crate's ONE `fnv1a64` (`shm_map`), not a private copy — it is
/// byte-oriented, hence the `.as_bytes()`.
pub fn wedge_page_shm_name(tag: &str, rank: u32) -> String {
    format!(
        "/cer_wdg_{:016x}",
        fnv1a64(format!("{tag}#{rank}").as_bytes())
    )
}

/// A process-shared SHM mapping of a [`WedgePage`].
///
/// Construct via [`create_owned`](MappedWedgePage::create_owned) (the SUPERVISOR —
/// `O_EXCL`-creates, sizes, initialises, owns the name and `shm_unlink`s it on
/// drop) or [`open_unowned`](MappedWedgePage::open_unowned) (the worker that rank
/// belongs to — STRICT open-existing, maps only). [`Deref`](std::ops::Deref)s to
/// the page.
///
/// As with [`crate::barrier`] and [`crate::state_arm`]: `shm_unlink` removes only
/// the NAME, so an owner dropping mid-run can never invalidate a peer's live
/// mapping — the object persists until the last `munmap`.
#[must_use = "the mapping is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local"]
pub struct MappedWedgePage {
    ptr: *mut WedgePage,
    name: std::ffi::CString,
    name_str: String,
    owns_name: bool,
}

// SAFETY: the mapped object is a `WedgePage` (atomics plus two fields written once
// before the magic publishes them) in a shared page. All post-init access goes
// through atomic ops, so it is sound to send/share the handle across threads; the
// OS keeps the MAP_SHARED page coherent across mappings.
unsafe impl Send for MappedWedgePage {}
unsafe impl Sync for MappedWedgePage {}

impl MappedWedgePage {
    /// Create + map + initialise the wedge page for `(tag, rank)` as its OWNER.
    ///
    /// A pre-existing orphan of the same name (a crashed prior deployment) is
    /// `shm_unlink`ed first so the `O_EXCL` create yields a fresh, zero-filled
    /// object — a crashed run's seq pairs can never be inherited as live ones.
    ///
    /// `slot_count` beyond [`WEDGE_MAX_SLOTS`] is REFUSED rather than truncated: a
    /// truncated table silently drops the tail nodes from the alarm, which is the
    /// failure mode this feature exists to remove.
    pub fn create_owned(tag: &str, rank: u32, slot_count: usize) -> std::io::Result<Self> {
        if slot_count > WEDGE_MAX_SLOTS {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "wedge page for rank {rank} needs {slot_count} node slots, past the \
                     {WEDGE_MAX_SLOTS}-slot page — refusing rather than truncating (a \
                     truncated table would drop the tail nodes from the wedge alarm \
                     silently)"
                ),
            ));
        }
        let name_str = wedge_page_shm_name(tag, rank);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;

        // Unlink-first (so a crashed prior deployment's orphan is cleared and the
        // `O_EXCL` create yields a fresh, ZERO-FILLED object — a dead run's seq
        // pairs can never be inherited as live ones) + create + `ftruncate`
        // EXACTLY ONCE (macOS EINVALs on a re-truncate) + `mmap`, cleanup arms
        // included: `shm_map::create_exclusive`.
        let addr = create_exclusive(&name, WEDGE_PAGE_BYTES)?;
        let ptr = addr as *mut WedgePage;
        // SAFETY: `ptr` is a freshly-mapped, exclusively-owned (O_EXCL),
        // page-aligned (≥ the 8-byte alignment the page needs) zero-filled region
        // of exactly `size_of::<WedgePage>()` (const-asserted above).
        unsafe { (*ptr).init(slot_count as u32) };
        // A capture child must not carry this mapping: it is per-rank live state
        // its peers are writing right now, and a child that inherited it would pay
        // its CoW cost for a page it can never legitimately touch.
        // SAFETY: the region just mapped and exclusively owned here.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                addr,
                WEDGE_PAGE_BYTES,
                crate::state_carrier::ForkExcludedMapping::WedgePage,
            );
        }
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: true,
        })
    }

    /// Open + map an EXISTING wedge page for `(tag, rank)` as a peer.
    ///
    /// STRICT open-existing (`O_RDWR`, no `O_CREAT`), so a missing object is an
    /// error rather than a silently-created page nobody sized. The mapped header is
    /// validated (magic / version / slot count) before the handle is returned.
    pub fn open_unowned(tag: &str, rank: u32) -> std::io::Result<Self> {
        let name_str = wedge_page_shm_name(tag, rank);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        // STRICT open-existing (`O_RDWR`, no `O_CREAT`); `seg`'s Drop closes the
        // descriptor on every early return below.
        let seg = OpenedSegment::open(&name)?;
        // SIZE CHECK before the map, and the THRESHOLD + its diagnostic stay HERE:
        // a creator that died between `shm_open` and `ftruncate` leaves a named,
        // ZERO-LENGTH object under a perfectly good name, and mapping it anyway
        // fails PLATFORM-DEPENDENTLY (Linux: the `mmap` succeeds and the first
        // touch is a SIGBUS process kill with no Rust error to attribute; macOS:
        // refused with an errno that says nothing about what was wrong).
        let size = seg.size()?;
        if (size as usize) < WEDGE_PAGE_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "wedge page '{name_str}' is {size} bytes, short of the \
                     {WEDGE_PAGE_BYTES}-byte page"
                ),
            ));
        }
        // No `ftruncate` — the owner already sized the object. `MAP_SHARED` is what
        // shares the owner's atomics.
        let addr = seg.map_shared(WEDGE_PAGE_BYTES)?;
        let ptr = addr as *mut WedgePage;
        // SAFETY: `ptr` maps ≥ WEDGE_PAGE_BYTES of a live object; `validate` reads
        // only initialised-or-zero words and never indexes the slot table.
        if let Err(reason) = unsafe { (*ptr).validate() } {
            // SAFETY: unmap exactly what we just mapped; nothing references it.
            unsafe { unmap(addr, WEDGE_PAGE_BYTES) };
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("wedge page '{name_str}' failed validation: {reason}"),
            ));
        }
        // The exclusion is per-MAPPING, and this process forks its own capture
        // children regardless of who created the segment.
        // SAFETY: the region just mapped and exclusively owned by this handle.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                addr,
                WEDGE_PAGE_BYTES,
                crate::state_carrier::ForkExcludedMapping::WedgePage,
            );
        }
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: false,
        })
    }

    /// The POSIX SHM object name.
    pub fn name(&self) -> &str {
        &self.name_str
    }

    /// The mapping's base address and length.
    pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
        (self.ptr as *mut std::ffi::c_void, WEDGE_PAGE_BYTES)
    }
}

/// The production [`WedgeMarker`](crate::scheduler::WedgeMarker) — the fire path's
/// only view of a page.
///
/// Both marks are one `Release` `fetch_add` on an already-mapped word: wait-free,
/// allocation-free, syscall-free, which is what makes them affordable on a fire
/// path (the `shm_ring` push contract, held to for the same reason).
impl crate::scheduler::WedgeMarker for MappedWedgePage {
    fn enter(&self, slot: usize) {
        WedgePage::enter(self, slot);
    }
    fn exit(&self, slot: usize) {
        WedgePage::exit(self, slot);
    }
    fn advance_step(&self) {
        WedgePage::advance_step(self);
    }
}

impl std::ops::Deref for MappedWedgePage {
    type Target = WedgePage;
    fn deref(&self) -> &WedgePage {
        // SAFETY: `ptr` is a valid mapping of an initialised `WedgePage` that stays
        // mapped for as long as `self` lives (unmapped only in `Drop`).
        unsafe { &*self.ptr }
    }
}

impl std::fmt::Debug for MappedWedgePage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MappedWedgePage")
            .field("name", &self.name_str)
            .field("owns_name", &self.owns_name)
            .field("slot_count", &self.slot_count())
            .finish()
    }
}

impl Drop for MappedWedgePage {
    fn drop(&mut self) {
        // SAFETY: unmap exactly the region create_owned/open_unowned mapped;
        // nothing references it after this.
        unsafe {
            unmap(self.ptr as *mut std::ffi::c_void, WEDGE_PAGE_BYTES);
        }
        if self.owns_name {
            // Best-effort unlink of the name this owner created. Removes the NAME
            // only — a peer's live mapping survives until its own munmap.
            unlink(&self.name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tag(what: &str) -> String {
        format!("wedge_{what}_{}", std::process::id())
    }

    #[test]
    fn the_slot_table_fills_the_page_and_its_offsets_are_where_a_peer_expects_them() {
        // The const-asserts above already fail the BUILD on a layout edit; this
        // states the derived numbers so a reader can check the arithmetic without
        // running the compiler in their head.
        assert_eq!(WEDGE_MAX_SLOTS, 63);
        assert_eq!(
            std::mem::size_of::<WedgeSlot>(),
            64,
            "one cache line per slot"
        );
        assert_eq!(std::mem::size_of::<WedgePage>(), WEDGE_PAGE_BYTES);
        assert_eq!(std::mem::offset_of!(WedgePage, slots), 64);
        // Every slot starts on a line boundary, which is the whole point of the
        // stride: the table itself begins at 64, so slot `i` begins at `64*(i+1)`.
        assert_eq!(std::mem::offset_of!(WedgePage, slots) % 64, 0);
    }

    #[test]
    fn a_name_is_prefix_free_short_enough_for_macos_and_distinct_per_rank() {
        let a = wedge_page_shm_name("run", 0);
        let b = wedge_page_shm_name("run", 1);
        assert_ne!(a, b, "two ranks of one run must not share a page");
        assert_eq!(a, wedge_page_shm_name("run", 0), "the name is stable");
        for n in [&a, &b] {
            assert_eq!(n.len(), 25, "PSHMNAMLEN is 31 on macOS: {n}");
            assert!(n.starts_with("/cer_wdg_"), "{n}");
        }
    }

    #[test]
    fn a_reading_is_in_a_tick_only_when_entry_leads_exit() {
        assert!(!SlotReading {
            entered: 0,
            exited: 0
        }
        .in_tick());
        assert!(SlotReading {
            entered: 7,
            exited: 6
        }
        .in_tick());
        assert!(!SlotReading {
            entered: 6,
            exited: 6
        }
        .in_tick());
        // A pair no writer can produce reads as NOT in a tick — the conservative
        // direction, which costs an observation of delay and cannot fabricate a
        // wedge.
        assert!(!SlotReading {
            entered: 5,
            exited: 9
        }
        .in_tick());
    }

    #[test]
    fn a_created_page_round_trips_through_a_peers_open_and_the_pairs_are_shared() {
        let t = tag("roundtrip");
        let owner = MappedWedgePage::create_owned(&t, 0, 3).expect("create");
        let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
        assert_eq!(peer.slot_count(), 3);
        assert_eq!(
            peer.read_slot(1),
            Some(SlotReading {
                entered: 0,
                exited: 0
            })
        );
        owner.enter(1);
        assert_eq!(
            peer.read_slot(1),
            Some(SlotReading {
                entered: 1,
                exited: 0
            }),
            "the peer must observe the OWNER's entry — this is the cross-process claim"
        );
        assert!(peer.read_slot(1).expect("slot").in_tick());
        owner.exit(1);
        assert_eq!(
            peer.read_slot(1),
            Some(SlotReading {
                entered: 1,
                exited: 1
            })
        );
        assert!(!peer.read_slot(1).expect("slot").in_tick());
        // A slot past the sized table is `None`, never a wrapped read of slot 0.
        assert_eq!(peer.read_slot(3), None);
        assert_eq!(peer.read_slot(WEDGE_MAX_SLOTS), None);
        // Out-of-range writes are ignored, not panics — a diagnostic must never
        // end a graph.
        owner.enter(3);
        owner.exit(999);
        assert_eq!(
            peer.read_slot(0),
            Some(SlotReading {
                entered: 0,
                exited: 0
            }),
            "an out-of-range write must not land on a live slot"
        );
    }

    #[test]
    fn the_step_progress_word_is_shared_and_independent_of_the_slots() {
        let t = tag("progress");
        let owner = MappedWedgePage::create_owned(&t, 0, 1).expect("create");
        let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
        assert_eq!(peer.step_progress(), 0);
        owner.advance_step();
        owner.advance_step();
        assert_eq!(peer.step_progress(), 2);
        assert_eq!(
            peer.read_slot(0),
            Some(SlotReading {
                entered: 0,
                exited: 0
            }),
            "a step bump must not move a node slot"
        );
    }

    #[test]
    fn opening_a_page_nobody_created_errs_rather_than_creating_one() {
        let err = MappedWedgePage::open_unowned(&tag("absent"), 7).expect_err("must not create");
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound, "{err}");
    }

    #[test]
    fn a_table_larger_than_the_page_is_refused_rather_than_truncated() {
        let t = tag("oversize");
        let err = MappedWedgePage::create_owned(&t, 0, WEDGE_MAX_SLOTS + 1).expect_err("refuse");
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput, "{err}");
        assert!(
            err.to_string().contains("refusing rather than truncating"),
            "the refusal must say WHY a truncated table is worse: {err}"
        );
        // The refusal must not leave a half-made object behind for the next run to
        // open and believe.
        assert!(MappedWedgePage::open_unowned(&t, 0).is_err());
        // Exactly at the boundary is ACCEPTED — the refusal is `>`, not `>=`.
        let ok = MappedWedgePage::create_owned(&t, 0, WEDGE_MAX_SLOTS).expect("at the boundary");
        assert_eq!(ok.slot_count(), WEDGE_MAX_SLOTS);
    }

    #[test]
    fn an_owner_drop_unlinks_the_name_but_a_live_peer_mapping_survives_it() {
        let t = tag("unlink");
        let owner = MappedWedgePage::create_owned(&t, 0, 1).expect("create");
        let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
        owner.enter(0);
        drop(owner);
        // The NAME is gone…
        assert!(MappedWedgePage::open_unowned(&t, 0).is_err());
        // …but the object lives until the last munmap, so a supervisor mid-poll
        // never reads freed memory.
        assert_eq!(
            peer.read_slot(0),
            Some(SlotReading {
                entered: 1,
                exited: 0
            })
        );
    }

    /// The LOAD ORDER in [`WedgePage::read_slot`] is what keeps a torn read
    /// conservative, and only a RACE can see it.
    ///
    /// Both counters are `fetch_add`s on one thread, so a reader that lands
    /// between them observes one of the two counters from a LATER fire than the
    /// other. Reading EXIT first means the stale half is `exited`, i.e. the
    /// reading can only ever over-report `entered > exited` ("in a tick"), which
    /// the classifier resolves on its next observation because the pair will have
    /// changed. Reading ENTRY first yields `exited > entered` — a state no writer
    /// can produce — which [`SlotReading::in_tick`] then has to launder.
    ///
    /// The oracle is therefore the IMPOSSIBLE pair, not the verdict: `in_tick`
    /// answers `false` for it either way, so every boolean assertion in this file
    /// passes under the swapped order. Asserting `exited <= entered` on every
    /// reading is the one claim the swap breaks.
    ///
    /// Deliberately NOT `#[serial]` and NOT wall-timed. It is also not
    /// SCHEDULER-timed: were the writer to run a FIXED
    /// 100_000 fires with the reader spinning only while `done` was clear, a writer
    /// that ran to completion before the reader was first scheduled would leave `reads`
    /// and `caught_in_tick` at zero and fail BOTH anti-vacuity assertions with
    /// no page bug anywhere. Instead the two threads
    /// HANDSHAKE: the writer keeps firing until the reader has published enough
    /// readings AND caught at least one fire in flight, so the interleaving the
    /// arm needs is a POSTCONDITION of the writer rather than a hope about the
    /// scheduler. A loaded machine makes the writer fire longer, never the arm fail.
    ///
    /// The reader samples `done` BEFORE each reading and breaks AFTER it, so its
    /// LAST reading is taken once the writer is provably finished — the quiesced
    /// final state goes through the same impossible-pair oracle as every racing
    /// one, which a break-before-read loop would skip entirely.
    #[test]
    fn a_racing_read_never_observes_a_pair_the_writer_cannot_produce() {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        /// The burst the writer always fires, handshake or not.
        const MIN_FIRES: u64 = 100_000;
        /// Readings the reader must have published before the writer may stop.
        const MIN_READS: u64 = 1_000;
        /// Fires per handshake re-check — the enter/exit loop stays tight.
        const CHUNK: u64 = 1_024;
        /// A GENEROUS liveness ceiling so a broken harness FAILS attributably
        /// instead of spinning forever. Nothing is timed against it: at ~5 ns a
        /// fire this is sub-second, and a reader that is running at all clears
        /// the handshake orders of magnitude earlier.
        const MAX_FIRES: u64 = 100_000_000;

        let t = tag("race");
        let owner = std::sync::Arc::new(MappedWedgePage::create_owned(&t, 0, 1).expect("create"));
        let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");

        let writer_page = std::sync::Arc::clone(&owner);
        let done = std::sync::Arc::new(AtomicBool::new(false));
        let reads_seen = std::sync::Arc::new(AtomicU64::new(0));
        let caught_seen = std::sync::Arc::new(AtomicU64::new(0));
        let writer_done = std::sync::Arc::clone(&done);
        let writer_reads = std::sync::Arc::clone(&reads_seen);
        let writer_caught = std::sync::Arc::clone(&caught_seen);
        let writer = std::thread::spawn(move || {
            for _ in 0..MIN_FIRES {
                writer_page.enter(0);
                writer_page.exit(0);
            }
            let mut fires = MIN_FIRES;
            while fires < MAX_FIRES
                && (writer_reads.load(Ordering::Acquire) < MIN_READS
                    || writer_caught.load(Ordering::Acquire) == 0)
            {
                for _ in 0..CHUNK {
                    writer_page.enter(0);
                    writer_page.exit(0);
                }
                fires += CHUNK;
            }
            writer_done.store(true, Ordering::Release);
            fires
        });

        let mut reads = 0u64;
        let mut caught_in_tick = 0u64;
        loop {
            // Sampled BEFORE the reading, so the reading taken on the last pass
            // is one the writer had already finished before — the final state
            // goes through the oracle too.
            let quiesced = done.load(Ordering::Acquire);
            let r = peer.read_slot(0).expect("slot 0 exists");
            assert!(
                r.exited <= r.entered,
                "a reading of {r:?} is a pair NO writer can produce — the two loads \
                 must read EXIT first, so the stale half can only ever make the pair \
                 look MORE in-a-tick, never less"
            );
            if r.in_tick() {
                caught_in_tick += 1;
                caught_seen.store(caught_in_tick, Ordering::Release);
            }
            reads += 1;
            reads_seen.store(reads, Ordering::Release);
            if quiesced {
                break;
            }
        }
        let fires = writer.join().expect("writer thread");

        // Anti-vacuity: the probe really did race the writer rather than
        // observing a quiesced page. A reader that never caught a fire in flight
        // would satisfy the assertion above trivially. Both are now GUARANTEED by
        // the handshake unless the ceiling was hit, which the message names.
        assert!(
            reads >= MIN_READS,
            "the reader took only {reads} readings against a {MIN_FIRES}+ fire writer \
             that waits for {MIN_READS} — the handshake did not hold (writer fired \
             {fires}, ceiling {MAX_FIRES})"
        );
        assert!(
            caught_in_tick > 0,
            "{reads} readings and NOT ONE caught the writer inside a tick — the arm \
             asserts nothing about torn reads unless it observes some (writer fired \
             {fires}, ceiling {MAX_FIRES})"
        );
        // The writer finished, so the page is quiesced and balanced. The count is
        // the writer's OWN, because the handshake makes it dynamic.
        let final_read = peer.read_slot(0).expect("slot 0 exists");
        assert_eq!(
            final_read,
            SlotReading {
                entered: fires,
                exited: fires
            }
        );
        assert!(!final_read.in_tick());
    }

    /// The SIZE check is the one seam the `shm_map` adoption RE-SHAPED (an
    /// inline `fstat` on a raw fd became `OpenedSegment::size()`), and nothing
    /// pinned it — every other arm in this file opens a page a healthy
    /// `create_owned` sized.
    ///
    /// The orphan is built the way a real one arises: a creator that died
    /// between `shm_open` and `ftruncate` leaves a NAMED, ZERO-LENGTH object
    /// under a perfectly good name. Mapping it anyway fails
    /// PLATFORM-DEPENDENTLY — on Linux the `mmap` SUCCEEDS and the first touch
    /// is a SIGBUS process kill with no Rust-level error to attribute; on macOS
    /// it is refused with an errno that says nothing about what was wrong —
    /// which is exactly why the threshold and its diagnostic stay at this site
    /// rather than in the substrate.
    #[test]
    fn a_zero_length_orphan_is_refused_with_the_size_diagnostic_rather_than_mapped() {
        let t = tag("shortseg");
        let name_str = wedge_page_shm_name(&t, 0);
        let name = std::ffi::CString::new(name_str.clone()).expect("name");

        // SAFETY: FFI unlink of a name this test derived; ENOENT is expected.
        unsafe { libc::shm_unlink(name.as_ptr()) };
        // SAFETY: FFI create of a named SHM object, deliberately NOT ftruncated
        // — this is the half-made orphan the check exists for.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "fixture shm_open failed: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: close the descriptor this fixture just opened; the NAME (and
        // the zero-length object under it) survives, which is the point.
        unsafe { libc::close(fd) };

        let err =
            MappedWedgePage::open_unowned(&t, 0).expect_err("a zero-length page must be refused");

        // Clean up BEFORE asserting, so a failing assertion cannot leave a
        // named zero-length segment behind for the next run to trip over.
        // SAFETY: best-effort unlink of the name this fixture created.
        unsafe { libc::shm_unlink(name.as_ptr()) };

        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData, "{err}");
        let msg = err.to_string();
        assert!(
            msg.contains(&name_str),
            "the refusal must name the segment an operator would go and look at: {msg}"
        );
        assert!(
            msg.contains("is 0 bytes"),
            "the refusal must report the size it OBSERVED, so a truncated page \
             and an unsized one read differently: {msg}"
        );
        assert!(
            msg.contains(&format!("short of the {WEDGE_PAGE_BYTES}-byte page")),
            "the refusal must state the threshold it was measured against: {msg}"
        );
    }

    #[test]
    fn a_crashed_runs_orphan_page_is_replaced_rather_than_inherited() {
        let t = tag("orphan");
        let first = MappedWedgePage::create_owned(&t, 0, 2).expect("create");
        first.enter(0);
        // The previous deployment "crashes": the mapping goes, the NAME stays
        // (leak it so the unlink-first create is what removes it).
        std::mem::forget(first);
        let second = MappedWedgePage::create_owned(&t, 0, 2).expect("re-create");
        assert_eq!(
            second.read_slot(0),
            Some(SlotReading {
                entered: 0,
                exited: 0
            }),
            "a fresh create must not inherit a dead run's seq pair — it would look \
             like a node wedged before this run started"
        );
    }
}

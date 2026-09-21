// SPDX-License-Identifier: AGPL-3.0-only
//! The child BREADCRUMB — the one page a fork child is allowed to
//! write, and the only thing that keeps its watchdog a LIVENESS check rather than a
//! duration cap.
//!
//! # Why a page, and why this page
//!
//! The watchdog kills a capture child only for a stalled counter, never for
//! elapsed time, because a duration cap is a **cost-derived refusal** — the
//! rejected outcome where a node big enough to exceed the deadline never
//! produces an anchor for the life of the robot. That inverts the question the parent
//! must answer from "how long has this taken?" to "is it still doing work?", and the
//! only way to answer the second is for the child to say so.
//!
//! It cannot say so through the state ring. The canonical sorted serializer
//! buffers keys before emitting anything, so a 30 M-entry map produces **zero ring
//! records** for the whole sort — a cursor-based watchdog would SIGKILL exactly the
//! giant the fork carrier exists to serve. Hence a separate word, bumped at every bounded
//! unit of work.
//!
//! # The constraints that pick the mechanism
//!
//! A fork child may not allocate on this path, may not take a lock the parent's other
//! threads could have held at the fork instant, and may not make a syscall per unit of
//! work (the child's ban list). A `MAP_SHARED | MAP_ANONYMOUS` page satisfies all three: it
//! is mapped ONCE at arm time in the parent, `fork` leaves `MAP_SHARED` regions
//! **shared rather than copy-on-write** (the same property `shm_ring` relies on,
//! `shm_ring.rs:320`), and a progress bump is one relaxed store into memory the parent
//! already has mapped. No `shm_open`, no name, no unlink: the page has no identity
//! outside the process tree that inherits it, so there is nothing to collide with and
//! nothing to leak if either side dies.
//!
//! # This page is DELIBERATELY excluded from the MADV_DONTFORK sweep
//!
//! The fork exclusion marks the mappings a child must NOT inherit — the iceoryx2
//! pools, the trace ring, the `MappedBarrier` page, the `StateArmWord`. The breadcrumb
//! and the state ring are the exact complement: they are the child's only channels, so
//! marking them would leave a child that cannot report progress (killed at the first
//! stall timeout, every time) and cannot emit an anchor. The `dontfork` sibling module
//! states that pairing where the sweep is applied.
//!
//! # What the parent may conclude from it
//!
//! Only that the child was alive and doing work at some point since the last
//! observation. It is a LIVENESS signal, not a progress percentage: the units are
//! whatever the encoder found convenient to count, they are not comparable across
//! nodes, and nothing derives a completion estimate from them. The `node_idx` /
//! `field_idx` / [`ChildPhase`] fields exist so a report can NAME what the child was
//! doing, which is what turns `ChildStalled` from a shrug into a diagnosis.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Distinctive magic identifying a Cerulion child breadcrumb: ASCII `"CERCRUMB"`.
const MAGIC: u64 = 0x4345_5243_5255_4D42;

/// The mapped size of the breadcrumb: one 4 KiB region.
///
/// `mmap` rounds up to the platform page size, so this is a page of its own on a 4 KiB
/// target (x86-64, L4T aarch64) and on a 16 KiB one (Apple Silicon). Sizing it as a
/// whole page is not about the 32 bytes it uses — it is so the child's progress store
/// shares no cache line, and no page, with anything the parent writes.
pub const BREADCRUMB_BYTES: usize = 4096;

/// What the child was doing when it last stamped the breadcrumb.
///
/// The load-bearing member is [`ChildPhase::RingFull`], and here is
/// why: a dead or wedged `bagd` stops draining the state ring, the child's next push
/// blocks, its progress word freezes, and a watchdog reading progress alone reports
/// `ChildStalled` — sending the operator to hunt an encoder bug in a node while the
/// RECORDER is the thing that died. One relaxed store before the blocking push makes
/// the two conditions distinguishable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildPhase {
    /// Between `fork(2)` and the first node — signal reset, fd close, `prctl`.
    Starting,
    /// Inside a node's encoder.
    Encoding,
    /// Blocked pushing a record because the state ring is FULL.
    ///
    /// The child is healthy and would resume the instant the ring drains, so a stall
    /// observed in this phase is a statement about the RECORDER, not the encoder.
    RingFull,
    /// Every node encoded; pushing the completion record.
    Finishing,
    /// A phase word this build does not recognise.
    ///
    /// Reachable only from a corrupted or future-version page. It is a distinct
    /// variant rather than a silent fold into `Encoding` because a diagnosis that
    /// names a phase the child never reported is worse than one that says it does not
    /// know — and the classifier treats it as NOT `RingFull`, which is the safe
    /// direction (see [`super::watchdog`]).
    Unknown,
}

impl ChildPhase {
    /// The wire value stored in the breadcrumb.
    pub fn as_wire(self) -> u32 {
        match self {
            Self::Starting => 0,
            Self::Encoding => 1,
            Self::RingFull => 2,
            Self::Finishing => 3,
            Self::Unknown => u32::MAX,
        }
    }

    /// Decode a wire value. Anything unrecognised is [`ChildPhase::Unknown`].
    pub fn from_wire(raw: u32) -> Self {
        match raw {
            0 => Self::Starting,
            1 => Self::Encoding,
            2 => Self::RingFull,
            3 => Self::Finishing,
            _ => Self::Unknown,
        }
    }

    /// The operator-facing noun, used in every report that names a phase.
    pub fn label(self) -> &'static str {
        match self {
            Self::Starting => "starting",
            Self::Encoding => "encoding",
            Self::RingFull => "ring-full",
            Self::Finishing => "finishing",
            Self::Unknown => "unknown",
        }
    }
}

/// The shared page itself.
///
/// `#[repr(C)]`, atomics only, so parent and child map it identically. Accessed only
/// through a pointer into the mapped region — never moved, never copied.
#[repr(C)]
pub struct ChildBreadcrumb {
    /// [`MAGIC`] once initialised. `Release`-stored LAST at create and `Acquire`-loaded
    /// FIRST by any reader, mirroring [`crate::state_arm`]'s discipline: a reader
    /// either fails clean or observes every field below.
    magic: AtomicU64,
    /// Bumped at every bounded unit of work. THE liveness signal.
    progress: AtomicU64,
    /// Which node of the fork set the child is inside (index into the fork set, not a
    /// ring manifest index — the parent holds both).
    node_idx: AtomicU32,
    /// Which field, where the encoder can report one. `u32::MAX` = not reported.
    field_idx: AtomicU32,
    /// [`ChildPhase::as_wire`].
    phase: AtomicU32,
    /// How many fork-set positions the child has finished ACCOUNTING FOR — captured
    /// or refused, both of which leave a record in the ring.
    ///
    /// This is the word the reserved slot was left for, and it exists to make the
    /// parent's post-mortem exact rather than approximate. After a child dies partway
    /// the parent must report the nodes it never covered, and `node_idx` alone cannot
    /// say where the boundary is: [`enter_node`](ChildBreadcrumb::enter_node) is
    /// stamped BEFORE the encode, so a child killed just after finishing node `k`
    /// still reads `node_idx == k` — and a parent skipping "from `node_idx` onward"
    /// would emit a SKIP for a node whose complete anchor is already in the ring.
    /// A reader would then hold a part and a refusal for one node at one step, which
    /// is a contradiction it has no rule for resolving.
    ///
    /// With this counter the rule is exact FOR EVERY OUTCOME THE CHILD REACHES ON ITS
    /// OWN: positions `0..nodes_accounted` are in the ring (as a part or as the
    /// child's own refusal), positions `nodes_accounted..len` are the ones the parent
    /// must report.
    ///
    /// # The one window it does NOT close
    ///
    /// The counter is bumped AFTER the record is durable, and the two are separate
    /// statements — `fork.rs` publishes with `sink.finish()` / `push_skip` and bumps
    /// on the line below. An ASYNCHRONOUS kill (an external SIGKILL, an OOM kill)
    /// landing between them leaves the record in the ring with the counter still
    /// pointing at or before that position, so the parent emits a SKIP for a node
    /// whose complete anchor is already there — the very contradiction described
    /// above, narrowed from "every child killed after node k" to "a child killed
    /// inside that one gap".
    ///
    /// The ORDER is nevertheless the right one, and reversing it is strictly worse:
    /// bumping first would trade a visible contradiction for a SILENT ABSENCE (a node
    /// marked accounted with no record, which the parent then does not report and no
    /// reader can distinguish from a node that was never due). This repo prefers the
    /// loud failure.
    ///
    /// It is CLOSED by a READER-side precedence rule, because no parent-side ordering
    /// can make two separate memory writes atomic against an asynchronous kill, and
    /// the parent cannot read its own ring (it is the producer; the recorder is the
    /// consumer). See [`crate::state_ring::StateAnchorEvent::SkipAfterComplete`]: for
    /// one `(run, step, node_idx)` the published Complete WINS, and the skip is
    /// reported as the diagnostic it is. So a child killed in this window costs the
    /// recording nothing — the anchor is applied — and the fact that it happened is
    /// still counted.
    nodes_accounted: AtomicU32,
}

/// The field a `field_idx` carries when the encoder cannot name one.
pub const FIELD_UNREPORTED: u32 = u32::MAX;

// The layout is a cross-PROCESS contract (the child writes it, the parent reads it),
// so the offsets are pinned the way `state_arm` and `barrier` pin theirs. A field
// reordered by an innocent-looking edit would make the parent read the child's
// progress out of its phase word.
const _: () = assert!(std::mem::size_of::<ChildBreadcrumb>() == 32);
const _: () = assert!(std::mem::align_of::<ChildBreadcrumb>() == 8);
const _: () = assert!(std::mem::size_of::<ChildBreadcrumb>() <= BREADCRUMB_BYTES);

impl ChildBreadcrumb {
    /// Initialise in place. Called ONCE by the parent, before any fork.
    ///
    /// # Safety
    ///
    /// `ptr` must point to at least [`BREADCRUMB_BYTES`] of writable, correctly
    /// aligned, exclusively-owned mapped memory.
    unsafe fn init(ptr: *mut ChildBreadcrumb) {
        let b = unsafe { &*ptr };
        b.progress.store(0, Ordering::Relaxed);
        b.node_idx.store(0, Ordering::Relaxed);
        b.field_idx.store(FIELD_UNREPORTED, Ordering::Relaxed);
        b.phase
            .store(ChildPhase::Starting.as_wire(), Ordering::Relaxed);
        b.nodes_accounted.store(0, Ordering::Relaxed);
        // LAST, and Release: a reader that sees the magic sees everything above.
        b.magic.store(MAGIC, Ordering::Release);
    }

    /// `true` once the page has been published by its (private) initialiser.
    pub fn is_initialised(&self) -> bool {
        self.magic.load(Ordering::Acquire) == MAGIC
    }

    /// CHILD SIDE: bump the liveness counter.
    ///
    /// One relaxed store. No allocation, no lock, no syscall, no clock read — which is
    /// what lets it sit inside an encoder's inner loop and inside the async-signal-
    /// unsafe window a fork child lives in.
    ///
    /// `Relaxed` is correct and not a shortcut: the parent draws exactly one
    /// conclusion from this word — "it changed, so the child is alive" — and that
    /// conclusion needs no happens-before with anything else. The fields the parent
    /// reads ALONGSIDE it (`node_idx`, `phase`) are diagnostics whose worst case is
    /// naming the previous unit of work.
    pub fn bump(&self) {
        self.progress.fetch_add(1, Ordering::Relaxed);
    }

    /// CHILD SIDE: record which node is being encoded, and reset the field.
    pub fn enter_node(&self, node_idx: u32) {
        self.node_idx.store(node_idx, Ordering::Relaxed);
        self.field_idx.store(FIELD_UNREPORTED, Ordering::Relaxed);
        self.phase
            .store(ChildPhase::Encoding.as_wire(), Ordering::Relaxed);
    }

    /// CHILD SIDE: record which field is being encoded, where the encoder knows.
    pub fn enter_field(&self, field_idx: u32) {
        self.field_idx.store(field_idx, Ordering::Relaxed);
    }

    /// CHILD SIDE: record the phase. The `RingFull` store is the recorder-vs-encoder split's whole cost.
    pub fn set_phase(&self, phase: ChildPhase) {
        self.phase.store(phase.as_wire(), Ordering::Relaxed);
    }

    /// CHILD SIDE: one more fork-set position is fully ACCOUNTED FOR in the ring.
    ///
    /// Called after a node's parts are finished AND after a node's own refusal record
    /// is pushed — both leave the reader something for that node, and the parent must
    /// report neither. One relaxed `fetch_add` per NODE (not per unit of work), so it
    /// costs nothing measurable beside the encode it follows.
    pub fn note_node_accounted(&self) {
        self.nodes_accounted.fetch_add(1, Ordering::Relaxed);
    }

    /// PARENT SIDE: how many fork-set positions the child got a record into the ring
    /// for. The first position the parent must report is exactly this index.
    pub fn nodes_accounted(&self) -> u32 {
        self.nodes_accounted.load(Ordering::Relaxed)
    }

    /// PARENT SIDE: read the liveness counter.
    pub fn progress(&self) -> u64 {
        self.progress.load(Ordering::Relaxed)
    }

    /// PARENT SIDE: the node the child last entered.
    pub fn node_idx(&self) -> u32 {
        self.node_idx.load(Ordering::Relaxed)
    }

    /// PARENT SIDE: the field the child last entered, or [`FIELD_UNREPORTED`].
    pub fn field_idx(&self) -> u32 {
        self.field_idx.load(Ordering::Relaxed)
    }

    /// PARENT SIDE: the phase the child last stamped.
    pub fn phase(&self) -> ChildPhase {
        ChildPhase::from_wire(self.phase.load(Ordering::Relaxed))
    }
}

/// An owned `MAP_SHARED | MAP_ANONYMOUS` mapping holding one [`ChildBreadcrumb`].
///
/// Created by the parent at ARM time — once per run, not once per anchor — because the
/// mapping is the thing `fork` shares and a per-anchor map/unmap would be a syscall
/// pair on the boundary path for no benefit. A completed child's stamps are reset by
/// [`rearm`](Self::rearm), not by re-mapping.
pub struct MappedBreadcrumb {
    ptr: *mut ChildBreadcrumb,
}

// The mapping is process-wide shared memory reached only through atomics; sending the
// handle across threads is exactly what the reaper thread needs (the parent's watchdog
// does not run on the node thread).
unsafe impl Send for MappedBreadcrumb {}
unsafe impl Sync for MappedBreadcrumb {}

impl MappedBreadcrumb {
    /// Map and initialise a fresh breadcrumb page.
    pub fn create() -> std::io::Result<Self> {
        // SAFETY: a fresh anonymous mapping of a whole page; the kernel either returns
        // MAP_FAILED or a region we exclusively own for the life of this handle.
        let addr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                BREADCRUMB_BYTES,
                libc::PROT_READ | libc::PROT_WRITE,
                // MAP_SHARED is the whole point: a MAP_PRIVATE page would be
                // copy-on-write across `fork`, so every stamp the child made would
                // land in the CHILD's copy and the parent would watch a frozen
                // counter — a watchdog that kills every child at the first timeout.
                libc::MAP_SHARED | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        if addr == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error());
        }
        let ptr = addr as *mut ChildBreadcrumb;
        // SAFETY: `addr` is a fresh page-sized mapping, aligned by construction.
        unsafe { ChildBreadcrumb::init(ptr) };
        Ok(Self { ptr })
    }

    /// Reset the page for a NEW child, leaving the mapping in place.
    ///
    /// The progress counter restarts at zero, which is what the parent's watchdog
    /// baseline expects: carrying the previous child's high-water mark forward would
    /// make the new child's first bumps look like no advance at all.
    pub fn rearm(&self) {
        let b = self.get();
        b.progress.store(0, Ordering::Relaxed);
        b.node_idx.store(0, Ordering::Relaxed);
        b.field_idx.store(FIELD_UNREPORTED, Ordering::Relaxed);
        b.phase
            .store(ChildPhase::Starting.as_wire(), Ordering::Relaxed);
        // Carrying the PREVIOUS child's tally forward would tell the parent that this
        // child had already covered nodes it has not started — so a child that died at
        // its first node would be reported as having covered them all, silently.
        b.nodes_accounted.store(0, Ordering::Relaxed);
    }

    /// The mapping's base address and length — what the `MADV_DONTFORK` sweep would
    /// need if this page were ever excluded. It is not (see the module docs), so this
    /// exists for the symmetric assertion: a test can prove the sweep did NOT mark it.
    pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
        (self.ptr as *mut std::ffi::c_void, BREADCRUMB_BYTES)
    }

    fn get(&self) -> &ChildBreadcrumb {
        // SAFETY: `ptr` is a live mapping owned by `self` for `self`'s lifetime, and
        // `ChildBreadcrumb` is atomics-only, so shared access is sound from any thread
        // and from a forked child.
        unsafe { &*self.ptr }
    }
}

impl std::ops::Deref for MappedBreadcrumb {
    type Target = ChildBreadcrumb;

    fn deref(&self) -> &Self::Target {
        self.get()
    }
}

impl std::fmt::Debug for MappedBreadcrumb {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let b = self.get();
        f.debug_struct("MappedBreadcrumb")
            .field("progress", &b.progress())
            .field("node_idx", &b.node_idx())
            .field("field_idx", &b.field_idx())
            .field("phase", &b.phase())
            .field("nodes_accounted", &b.nodes_accounted())
            .finish()
    }
}

impl Drop for MappedBreadcrumb {
    fn drop(&mut self) {
        // SAFETY: unmapping exactly the region this handle mapped. A child holding an
        // inherited mapping is unaffected — `munmap` releases this address space's
        // reference, never the object.
        unsafe {
            libc::munmap(self.ptr as *mut std::ffi::c_void, BREADCRUMB_BYTES);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_wire_round_trips_and_unknown_is_closed() {
        // Hand oracle: the wire values are a cross-process contract, so they are
        // asserted as literals rather than derived from the enum.
        for (phase, wire) in [
            (ChildPhase::Starting, 0u32),
            (ChildPhase::Encoding, 1),
            (ChildPhase::RingFull, 2),
            (ChildPhase::Finishing, 3),
        ] {
            assert_eq!(phase.as_wire(), wire, "{phase:?} wire value");
            assert_eq!(ChildPhase::from_wire(wire), phase, "wire {wire} decode");
        }
        // Anything else decodes to Unknown rather than to a neighbouring phase — a
        // diagnosis must not name a phase the child never reported.
        for raw in [4u32, 5, 99, u32::MAX - 1, u32::MAX] {
            assert_eq!(
                ChildPhase::from_wire(raw),
                ChildPhase::Unknown,
                "wire {raw} must decode Unknown"
            );
        }
        assert_eq!(ChildPhase::Unknown.as_wire(), u32::MAX);
    }

    #[test]
    fn a_fresh_breadcrumb_is_published_and_zeroed() {
        let b = MappedBreadcrumb::create().expect("map breadcrumb");
        assert!(b.is_initialised(), "the magic must be published at create");
        assert_eq!(b.progress(), 0);
        assert_eq!(b.node_idx(), 0);
        assert_eq!(b.field_idx(), FIELD_UNREPORTED);
        assert_eq!(b.phase(), ChildPhase::Starting);
        assert_eq!(b.nodes_accounted(), 0);
    }

    #[test]
    fn the_accounting_word_is_what_stops_a_post_mortem_contradicting_the_ring() {
        // `enter_node` is stamped BEFORE the encode, so a child killed just after
        // finishing node 1 still reads `node_idx == 1`. A parent reporting "from
        // node_idx onward" would emit a SKIP for a node whose complete anchor is
        // already in the ring, and a reader would hold a part AND a refusal for one
        // node at one step. The accounting word is what makes the boundary exact.
        let b = MappedBreadcrumb::create().expect("map breadcrumb");
        b.enter_node(0);
        b.note_node_accounted();
        b.enter_node(1);
        b.note_node_accounted();
        assert_eq!(
            b.node_idx(),
            1,
            "the position word still names the node just finished"
        );
        assert_eq!(
            b.nodes_accounted(),
            2,
            "while the accounting word says both are in the ring, so the parent's \
             first un-covered position is 2"
        );

        // And it counts NODES, never units of work: the progress bumps in between
        // must not move it.
        b.enter_node(2);
        for _ in 0..1000 {
            b.bump();
        }
        assert_eq!(b.nodes_accounted(), 2, "a bump is not an accounting");
    }

    #[test]
    fn stamps_are_readable_through_the_mapping() {
        let b = MappedBreadcrumb::create().expect("map breadcrumb");
        b.enter_node(7);
        b.enter_field(3);
        b.bump();
        b.bump();
        b.bump();
        assert_eq!(b.progress(), 3);
        assert_eq!(b.node_idx(), 7);
        assert_eq!(b.field_idx(), 3);
        assert_eq!(b.phase(), ChildPhase::Encoding);

        b.set_phase(ChildPhase::RingFull);
        assert_eq!(b.phase(), ChildPhase::RingFull);

        // `enter_node` resets the field, so a stale field index from the PREVIOUS node
        // can never be attributed to this one.
        b.enter_node(8);
        assert_eq!(b.field_idx(), FIELD_UNREPORTED);
        assert_eq!(b.node_idx(), 8);
        assert_eq!(b.phase(), ChildPhase::Encoding);
    }

    #[test]
    fn rearm_resets_the_watchdog_baseline_without_remapping() {
        let b = MappedBreadcrumb::create().expect("map breadcrumb");
        b.enter_node(4);
        b.enter_field(2);
        b.set_phase(ChildPhase::RingFull);
        b.note_node_accounted();
        b.note_node_accounted();
        for _ in 0..1000 {
            b.bump();
        }
        assert_eq!(b.progress(), 1000);
        assert_eq!(b.nodes_accounted(), 2);

        let (before, _) = b.mapping();
        b.rearm();
        let (after, _) = b.mapping();

        // Carrying 1000 forward would make the NEXT child's first thousand bumps look
        // like no advance at all — the watchdog compares against a watermark.
        assert_eq!(b.progress(), 0, "rearm must zero the counter");
        assert_eq!(b.node_idx(), 0);
        assert_eq!(b.field_idx(), FIELD_UNREPORTED);
        assert_eq!(b.phase(), ChildPhase::Starting);
        assert_eq!(
            b.nodes_accounted(),
            0,
            "carrying the tally forward would tell the parent this child had already \
             covered nodes it has not started"
        );
        assert!(b.is_initialised(), "rearm must not un-publish the page");
        assert_eq!(before, after, "rearm must not re-map");
    }
}

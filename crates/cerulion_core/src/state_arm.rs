// SPDX-License-Identifier: AGPL-3.0-only
//! The checkpoint ARM WORD — the mapped-SHM word that says
//! whether a recorder is attached, how often to anchor, and which workers are
//! currently spending memory on a child.
//!
//! It exists because `cerulion bag record --run` is a MID-RUN ATTACH: there is no
//! spawn to configure, so the running graph learns it is being checkpointed by
//! reading a word a recorder created. While DETACHED the boundary check is a null
//! test on a field the loop already has in cache — nothing is spawned, no arena is
//! allocated, no ring exists, no `fork` is reachable, and every line of generated
//! capture code is cold on every path. **That is how "a detached graph pays nothing" is
//! satisfied structurally rather than by measurement**, and it is the same class the
//! codebase already documents for `pump_history_all`.
//!
//! It reuses the mapped-SHM-word primitive [`crate::barrier`] establishes: an
//! `O_EXCL`-created, `MAP_SHARED`, owner-unlinks-on-drop segment holding nothing but
//! atomics, with a magic word `Release`-stored LAST so a racing open either fails
//! clean or sees a fully initialised word.
//!
//! # What is here, and what is deliberately the carrier's
//!
//! The LAYOUT is a cross-process contract, so this module fixes it whole — including
//! the pid-tagged claim table the stale-claim sweep requires, which the `busy_workers`
//! / `rss_reserved` reservation is DERIVED from
//! ([`StateArmWord::reservation`]). Growing a `MAP_SHARED` struct later is the
//! expensive move ([`crate::barrier`] has to argue ABI-safety for taking
//! `BarrierShared` from 16 to 24 bytes), and shipping the reservation as a BARE count
//! would make the stale-claim sweep impossible to add without changing the layout again:
//! validating a claimant's liveness needs a pid ALONGSIDE the claim, so the pid is not
//! an optional extra.
//!
//! What is NOT here is the POLICY: the memory PROJECTION, the cached
//! `MemAvailable` read, and the `kill(pid, 0)` liveness syscall all live in the
//! carrier. [`StateArmWord::sweep_stale_claims`] therefore takes the
//! liveness predicate as an ARGUMENT, so the stale-claim rule — a worker dying before
//! its reap must not freeze every survivor's cadence as `StillEncoding` forever — is
//! oracle-testable here with no syscall and no second process (mutating a live-syscall
//! path inherits that syscall's blast radius).
//!
//! # The claim table is CAS-owned, because it is shared across processes
//!
//! There is exactly ONE representation of the reservation: the table. Two
//! representations of one fact in a `MAP_SHARED` page is where every race in this
//! module would live, and a cached aggregate is unfixable rather than merely delicate
//! — a process SIGKILLed mid-claim leaves a contribution no reclaim can know how to
//! undo, whichever order the two words are written in (the argument in full is on
//! [`StateArmWord::reservation`]).
//!
//! The rule that removes the class: a slot's `owner` word is its single ownership
//! token, it carries BOTH the lifecycle tag and the holder's pid, and every transition
//! is a compare-exchange on it. So exactly one party runs each claim's initialisation
//! and exactly one runs its teardown; a held slot always names its holder, which is
//! what makes a dead one reclaimable at all; and every exchange is identity-checked,
//! so a recycled slot cannot be torn down on a stale observation. Nothing ever
//! publishes a total derived from a scan — a scan is not atomic, so such a total is
//! stale the moment a peer claims or releases, and storing it would overwrite that
//! peer's update (leaving a ghost reservation that blocks every future fork, or an
//! underflowed one that permits forks against memory already spoken for). The full
//! ordering argument, including which direction the publication windows err in and
//! what residual is left, is on [`StateArmWord::claim`].
//!
//! # MADV_DONTFORK
//!
//! The child must not be able to touch this word, so the carrier's `MADV_DONTFORK` sweep
//! covers it. That imposes a LAYOUT constraint this module owns and holds:
//! [`MappedStateArm`] maps [`STATE_ARM_BYTES`] as its OWN mapping holding nothing
//! else, so one `madvise(ptr, STATE_ARM_BYTES, MADV_DONTFORK)` covers it exactly and
//! nothing else. (`MADV_DONTFORK` is Linux-only, which is a property of the
//! sweep, not of this layout.)
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on its
//! `pub mod state_arm;` declaration in `lib.rs`.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// The shared POSIX map substrate + the name fold
// (`barrier`, `shm_ring` and `credit` sit on the same one).
use crate::shm_map::{create_exclusive, fnv1a64, unlink, unmap, OpenedSegment};

/// Distinctive magic identifying a Cerulion state arm word: ASCII `"CERSTARM"`.
const MAGIC: u64 = 0x4345_5253_5441_524D;

/// On-word format version. Bumped on any layout change.
///
/// **2** — the claim slot's lifecycle tag and its holder's pid were merged into one
/// `owner` atomic (see [`StateClaimSlot`]). Version 1's two-word slot has the same
/// size, the same alignment and the same table offset, so a peer built against it
/// would open a version-2 segment, pass every other check, and read the tag half of an
/// ownership word as a `state` and the pid half as… a `state` again. Nothing but this
/// number distinguishes them, so a stale peer must fail LOUDLY when
/// [`MappedStateArm::open_unowned`] validates the header, rather than misread the
/// table. (The bump costs nothing and closes a stale-peer foot-gun.)
pub const STATE_ARM_VERSION: u32 = 2;

/// Per-process claim slots in the reservation table.
///
/// A slot is claimed by ONE worker process for the life of ONE child. 32 is well
/// above the worker count a process-per-node split produces on any shipping
/// robot, and the whole table is 512 bytes — small enough that sizing it generously
/// costs nothing against the page the word occupies anyway.
pub const STATE_CLAIM_SLOTS: usize = 32;

/// The mapped size of the arm word: one 4 KiB region.
///
/// `mmap` rounds up to the platform page size, so this is "its own mapping, holding
/// nothing else" on a 4 KiB page (x86-64, most aarch64 Linux) AND on a 16 KiB one
/// (Apple Silicon). That is what the carrier's `MADV_DONTFORK` sweep needs:
/// one call covering exactly this object.
pub const STATE_ARM_BYTES: usize = 4096;

/// The cross-process checkpoint arm word.
///
/// `#[repr(C)]`, atomics + fixed-size fields only, so it maps identically across
/// processes. Accessed only through a pointer into the mapped segment.
#[repr(C)]
pub struct StateArmWord {
    /// [`MAGIC`] once fully initialised. `Release`-stored as the LAST write at
    /// create and `Acquire`-loaded FIRST by an opener, so a racing open either fails
    /// clean or observes every field below.
    magic: AtomicU64,
    /// [`STATE_ARM_VERSION`].
    version: u32,
    /// [`STATE_CLAIM_SLOTS`], so an opener refuses a segment whose table it would
    /// index past.
    slot_count: u32,
    /// Nonzero while a recorder is attached. THE field the per-boundary check reads.
    armed: AtomicU32,
    /// Explicit padding; never read. Layout only.
    _pad0: u32,
    /// The logical anchor cadence in STEPS (derived once at arm time, never a
    /// wall timer). `0` means one-shot: anchor at
    /// [`first_anchor_step`](Self::first_anchor_step) and never again.
    cadence_steps: AtomicU64,
    /// The first step at which an anchor is due. On a mid-run attach the recorder
    /// sets this to "next boundary" so the checkpoint and the trace/frame gates land
    /// on ONE `S` by construction rather than by two writers being trusted to agree.
    first_anchor_step: AtomicU64,
    /// Explicit padding so the claim table starts at offset 64 (one cache line), so
    /// it never false-shares with the header words above. Never read; layout only.
    ///
    /// It is 24 bytes rather than 8 because no CACHED aggregate words
    /// sit here: `busy_workers` and `rss_reserved` are DERIVED from
    /// the table on read ([`reservation`](Self::reservation)), which is what makes an
    /// interrupted claim reclaimable at all — see that method for the argument.
    _pad1: [u64; 3],
    /// The pid-tagged claim table.
    claims: [StateClaimSlot; STATE_CLAIM_SLOTS],
}

/// One worker's reservation claim.
///
/// The pid is what makes the stale-claim sweep possible: a bare count cannot tell a worker that
/// is mid-anchor from one that DIED before its reap, and under `--peer-loss continue`
/// the second freezes every survivor's cadence as `StillEncoding` forever.
///
/// # Ownership and identity are ONE word, and that is the whole protocol
///
/// `owner` packs the [`SlotState`] discriminant and the holder's pid into a single
/// atomic: `(tag as u64) << 32 | pid as u32`. Every transition is a compare-exchange
/// on that word, so a slot is never held by an owner nobody can name — the instant
/// the CAS wins the slot it has ALREADY published who holds it.
///
/// The pid answers ONE question: **who must be alive for this transition to finish?**
/// While `Claiming` or `Active` that is the claimant; while `Releasing` it is whoever
/// is running the teardown, which is not always the same process (see
/// [`StateArmWord::release`]). Reading it as "the claimant" in every state is what
/// made an in-progress release indistinguishable from an abandoned one.
///
/// A two-word form (a `state` discriminant plus a separate `pid`) cannot offer that,
/// and the gap is not theoretical: a process SIGKILLed between winning the slot and
/// storing its pid leaves the slot held, non-free, and ANONYMOUS. Liveness cannot be
/// tested on an owner with no identity, so no sweep can ever reclaim it; the table is
/// finite, so repeated over a robot's life it is a RATCHET that ends with every
/// [`claim`](StateArmWord::claim) returning `None` and checkpointing dead
/// machine-wide. Packing removes the window rather than narrowing it.
///
/// It also makes every ownership CAS IDENTITY-CHECKED for free. A sweep that observes
/// `(Active, 4242)` and exchanges on that exact word cannot release a DIFFERENT
/// claim that took the same slot in between — with a bare tag the exchange would
/// succeed against `(Active, 5555)` and free a live peer's reservation (classic ABA
/// on a recycled slot).
///
/// Still exactly 16 bytes, so the table still starts at offset 64.
#[repr(C)]
pub struct StateClaimSlot {
    /// The OWNERSHIP WORD: `(tag << 32) | pid`. See the type doc — and
    /// [`StateArmWord::claim`] for the protocol built on it. The winner of a
    /// transition CAS is the only party that may touch this slot's `reserved_bytes`.
    owner: AtomicU64,
    /// Bytes this claim reserved. Written by the party that owns the slot, so a
    /// reader must take the `owner` [`Acquire`](Ordering::Acquire) edge first — see
    /// [`StateArmWord::claim_view`], the one read path.
    reserved_bytes: AtomicU64,
}

/// A claim slot's lifecycle position — the ownership token of
/// [`StateArmWord::claim`]'s protocol.
///
/// Exposed (via [`StateArmWord::claim_state`]) because Principle #3 says a slot's
/// position must be observable rather than inferred from a count that looks wrong —
/// a worker SIGKILLed inside the few instructions between taking a slot and publishing
/// it leaves it in one of the two TRANSIENT states until a sweep reclaims it.
///
/// EVERY held state names its holder (the pid rides in the same atomic — see
/// [`StateClaimSlot`]), so every one of them is reclaimable from a dead owner. The
/// tag says what its holder was DOING, never whether anyone can be found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotState {
    /// Nobody holds it; a `claim` may take it.
    Free,
    /// A claimer holds it and is initialising. Its `reserved_bytes` are NOT yet
    /// trustworthy, and a sweep must leave it alone unless its owner is dead.
    Claiming,
    /// A published claim. `reserved_bytes` is readable, and the derived reservation
    /// includes it.
    Active,
    /// A releaser holds it and is finishing. Its pid names the party running the
    /// TEARDOWN — the claimant, a supervisor releasing on its behalf, or a sweep
    /// reclaiming a dead one — which is exactly the party that must be alive for the
    /// slot to come back. A sweep must leave it alone unless THAT party is dead. See
    /// [`StateArmWord::release`].
    Releasing,
    /// A discriminant this build does not know — reported rather than guessed.
    Unrecognized(u32),
}

impl SlotState {
    /// The wire discriminant. Frozen: these values live in a `MAP_SHARED` page read
    /// by peer processes.
    pub fn as_wire(self) -> u32 {
        match self {
            Self::Free => SLOT_FREE,
            Self::Claiming => SLOT_CLAIMING,
            Self::Active => SLOT_ACTIVE,
            Self::Releasing => SLOT_RELEASING,
            Self::Unrecognized(raw) => raw,
        }
    }

    /// Decode a discriminant. Never fails — an unknown value is REPORTED, because a
    /// slot in a state this build cannot name is exactly what an operator needs to
    /// see, and silently reading it as `Free` would hand it to a second claimer.
    pub fn from_wire(raw: u32) -> Self {
        match raw {
            SLOT_FREE => Self::Free,
            SLOT_CLAIMING => Self::Claiming,
            SLOT_ACTIVE => Self::Active,
            SLOT_RELEASING => Self::Releasing,
            other => Self::Unrecognized(other),
        }
    }

    /// Whether the slot is mid-transition — held by somebody, publishable by nobody.
    pub fn is_in_flight(self) -> bool {
        matches!(self, Self::Claiming | Self::Releasing)
    }
}

/// One ordered observation of a claim slot — what
/// [`StateArmWord::claim_view`] returns.
///
/// Assembled through a single `Acquire` load of the slot's `state`, which is the
/// publication edge the other two fields ride on. See `claim_view` for the ordering
/// contract and for what a view does NOT promise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClaimView {
    /// The slot's lifecycle position.
    pub state: SlotState,
    /// The claiming process's pid, or 0 when free.
    pub pid: i32,
    /// Bytes the claim reserved.
    pub reserved_bytes: u64,
}

/// A fresh, zero-filled slot is FREE, which is what makes an `O_EXCL` create sound.
const SLOT_FREE: u32 = 0;
const SLOT_CLAIMING: u32 = 1;
const SLOT_ACTIVE: u32 = 2;
const SLOT_RELEASING: u32 = 3;

/// The pid a slot carries when NOBODY nameable holds it: free, or held by a reclaim
/// (see [`StateArmWord::sweep_stale_claims`]).
///
/// It is also why [`StateArmWord::claim`] refuses a non-positive pid. `kill(0, 0)`
/// signals the CALLER'S OWN process group and `kill(-N, 0)` signals group `N`, so a
/// liveness probe on either answers a question nobody asked — and would report a dead
/// claimant ALIVE, which is exactly how the stale-claim sweep fails silently.
const PID_NONE: i32 = 0;

/// Pack a [`SlotState`] discriminant and a pid into one ownership word.
const fn pack_owner(tag: u32, pid: i32) -> u64 {
    // `pid as u32` is the two's-complement bit pattern, so a negative pid round-trips
    // and cannot bleed into the tag half.
    ((tag as u64) << 32) | (pid as u32 as u64)
}

/// The [`SlotState`] discriminant half of an ownership word.
const fn owner_tag(word: u64) -> u32 {
    (word >> 32) as u32
}

/// The pid half of an ownership word.
const fn owner_pid(word: u64) -> i32 {
    word as u32 as i32
}

/// A slot nobody holds. Zero, so a zero-filled mapping is an empty table.
const OWNER_FREE: u64 = pack_owner(SLOT_FREE, PID_NONE);

/// The marker a sweep exchanges a dead owner's word for, giving it exclusive
/// possession of the slot's teardown. Distinct from every word a live participant can
/// write, because a claimant's pid is always positive.
const OWNER_RECLAIMING: u64 = pack_owner(SLOT_RELEASING, PID_NONE);

// Layout is a cross-process contract: pin size, alignment and every load-bearing
// offset so a field reorder or type change fails the BUILD rather than silently
// mismatching one participant's view of the word.
const _: () = assert!(std::mem::size_of::<StateClaimSlot>() == 16);
const _: () = assert!(std::mem::align_of::<StateClaimSlot>() == 8);
const _: () = assert!(std::mem::offset_of!(StateClaimSlot, owner) == 0);
const _: () = assert!(std::mem::offset_of!(StateClaimSlot, reserved_bytes) == 8);
// The packing is a cross-process contract too, and its two failure modes are silent:
// a free slot that is not the zero word would make a fresh mapping look OCCUPIED, and
// a pid whose sign bit bled into the tag would make a held slot unnameable.
const _: () = assert!(OWNER_FREE == 0);
const _: () = assert!(owner_tag(OWNER_RECLAIMING) == SLOT_RELEASING);
const _: () = assert!(owner_pid(OWNER_RECLAIMING) == PID_NONE);
const _: () = assert!(owner_tag(pack_owner(SLOT_ACTIVE, -1)) == SLOT_ACTIVE);
const _: () = assert!(owner_pid(pack_owner(SLOT_ACTIVE, i32::MIN)) == i32::MIN);
const _: () = assert!(owner_pid(pack_owner(SLOT_CLAIMING, i32::MAX)) == i32::MAX);
const _: () = assert!(std::mem::align_of::<StateArmWord>() == 8);
const _: () = assert!(std::mem::offset_of!(StateArmWord, magic) == 0);
const _: () = assert!(std::mem::offset_of!(StateArmWord, version) == 8);
const _: () = assert!(std::mem::offset_of!(StateArmWord, slot_count) == 12);
const _: () = assert!(std::mem::offset_of!(StateArmWord, armed) == 16);
const _: () = assert!(std::mem::offset_of!(StateArmWord, cadence_steps) == 24);
const _: () = assert!(std::mem::offset_of!(StateArmWord, first_anchor_step) == 32);
const _: () = assert!(std::mem::offset_of!(StateArmWord, claims) == 64);
const _: () = assert!(
    std::mem::size_of::<StateArmWord>() <= STATE_ARM_BYTES,
    "the arm word must fit the region MappedStateArm maps (and MADV_DONTFORKs)"
);

/// PURE: is an anchor due at `step`?
///
/// Split out of [`StateArmWord::due`] so the whole decision — including both sides of
/// its boundary — is oracle-testable with no SHM, no clock and no process.
///
/// - a DETACHED word is never due (the null test the per-boundary check rests on);
/// - a step BEFORE `first_anchor_step` is never due, and the first step AT it always
///   is (`>=`, not `>`: the attach's own boundary is the one the trace and
///   frame gates are pinned to, so skipping it would leave the bag with a checkpoint at `S` and
///   gates at a different `S`);
/// - `cadence_steps == 0` is ONE-SHOT — due exactly at `first_anchor_step`. That is
///   not a degenerate case: periodic anchors are OFF by default for
///   `graph run --record` (step 0 is a free and perfect anchor), and a mid-run
///   attach wants an immediate one. It also means the modulo below can never divide
///   by zero.
pub fn cadence_due(armed: bool, first_anchor_step: u64, cadence_steps: u64, step: u64) -> bool {
    if !armed || step < first_anchor_step {
        return false;
    }
    let delta = step - first_anchor_step;
    if cadence_steps == 0 {
        return delta == 0;
    }
    delta.is_multiple_of(cadence_steps)
}

/// What [`StateArmWord::sweep_stale_claims`] cleared.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StaleClaimSweep {
    /// How many claims were held by a process that is no longer alive AND were
    /// released by THIS sweep (it lost the race for any it did not clear).
    pub cleared: u32,
    /// How many reserved bytes this sweep gave back — across BOTH arms
    /// ([`cleared`](Self::cleared) and
    /// [`interrupted_reclaimed`](Self::interrupted_reclaimed)). An interrupted claim
    /// that had already written its byte count was holding those bytes against every
    /// peer's fork gate, so freeing them silently would under-report the recovery.
    pub freed_bytes: u64,
    /// How many published claims were seen alive.
    ///
    /// An OBSERVATION, not an authority: the walk is not atomic, so this is what the
    /// table looked like as the sweep passed each slot, and it is deliberately NOT
    /// written back to `busy_workers` (see
    /// [`sweep_stale_claims`](StateArmWord::sweep_stale_claims)).
    pub live: u32,
    /// How many slots were held by a LIVE peer mid-transition, by a reclaim this sweep
    /// did not win, or in a state this build cannot name — none of them the sweep's to
    /// judge.
    ///
    /// This is a TRANSIENT observation, not a leak count: since ownership carries its
    /// holder's identity, a slot whose owner has DIED is reclaimable from every held
    /// state and is reported under `cleared` / `interrupted_reclaimed` instead. A
    /// persistently nonzero `in_flight` therefore means peers really are transitioning
    /// as the sweep passes, which is ordinary.
    pub in_flight: u32,
    /// How many INTERRUPTED claims were reclaimed — slots whose owner died mid-claim
    /// or mid-release, i.e. after taking the slot and before finishing with it.
    ///
    /// Counted apart from [`cleared`](Self::cleared) because it means something
    /// different and rarer: a worker was killed inside a window a handful of
    /// instructions wide. An operator seeing this repeatedly is watching something
    /// kill workers constantly, which an ordinary stale claim does not imply.
    pub interrupted_reclaimed: u32,
}

impl StaleClaimSweep {
    /// Whether anything was reclaimed at all (the caller logs LOUDLY when so —
    /// the stale-claim rule is "validate claimant liveness, clear stale claims
    /// loudly"). Covers BOTH a stale published claim and an interrupted one.
    pub fn is_empty(&self) -> bool {
        self.cleared == 0 && self.interrupted_reclaimed == 0
    }
}

impl StateArmWord {
    /// Initialise a freshly-mapped, zero-filled word. Writes every field, then
    /// `Release`-stores the magic LAST.
    ///
    /// # Safety
    ///
    /// `self` must point at a zero-filled region of at least
    /// [`STATE_ARM_BYTES`] that no peer is reading yet.
    fn init(&self) {
        // The fields below are plain (non-atomic) and are written exactly once,
        // before the magic publishes them.
        // SAFETY: `self` is a freshly-created, exclusively-owned mapping; the
        // interior-mutability write happens before any peer can observe the word
        // (the magic is stored last, with Release ordering).
        unsafe {
            let me = self as *const Self as *mut Self;
            (*me).version = STATE_ARM_VERSION;
            (*me).slot_count = STATE_CLAIM_SLOTS as u32;
        }
        self.armed.store(0, Ordering::Relaxed);
        self.cadence_steps.store(0, Ordering::Relaxed);
        self.first_anchor_step.store(0, Ordering::Relaxed);
        for slot in &self.claims {
            slot.owner.store(OWNER_FREE, Ordering::Relaxed);
            slot.reserved_bytes.store(0, Ordering::Relaxed);
        }
        self.magic.store(MAGIC, Ordering::Release);
    }

    /// Validate a mapped word an opener did not create: magic, version, slot count.
    fn validate(&self) -> Result<(), String> {
        if self.magic.load(Ordering::Acquire) != MAGIC {
            return Err(
                "magic is not set — the segment is not a state arm word, or its \
                        creator has not finished initialising it"
                    .to_string(),
            );
        }
        if self.version != STATE_ARM_VERSION {
            return Err(format!(
                "version {} is not {STATE_ARM_VERSION}",
                self.version
            ));
        }
        if self.slot_count as usize != STATE_CLAIM_SLOTS {
            return Err(format!(
                "slot_count {} is not {STATE_CLAIM_SLOTS} — this build would index past the table",
                self.slot_count
            ));
        }
        Ok(())
    }

    /// ARM the word: publish the cadence and the first anchor step, THEN the armed
    /// flag.
    ///
    /// The order is load-bearing and mirrors the ring header's magic-last rule: a
    /// reader that observes `armed` with `Acquire` observes the two values that came
    /// before it, so it can never anchor against a cadence from a previous attach.
    pub fn arm(&self, cadence_steps: u64, first_anchor_step: u64) {
        self.cadence_steps.store(cadence_steps, Ordering::Relaxed);
        self.first_anchor_step
            .store(first_anchor_step, Ordering::Relaxed);
        self.armed.store(1, Ordering::Release);
    }

    /// DISARM: the boundary check goes back to a null test and nothing downstream
    /// runs. The cadence values are left as they are — a later
    /// [`arm`](Self::arm) republishes them before re-arming.
    pub fn disarm(&self) {
        self.armed.store(0, Ordering::Release);
    }

    /// Whether a recorder is attached. `Acquire`, so the cadence values arm published
    /// are visible to whoever reads them next.
    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::Acquire) != 0
    }

    /// The anchor cadence in steps (`0` = one-shot).
    pub fn cadence_steps(&self) -> u64 {
        self.cadence_steps.load(Ordering::Relaxed)
    }

    /// The first step at which an anchor is due.
    pub fn first_anchor_step(&self) -> u64 {
        self.first_anchor_step.load(Ordering::Relaxed)
    }

    /// THE boundary check: is an anchor due at `step`?
    ///
    /// Reads `armed` first with `Acquire` and short-circuits, so a detached word costs
    /// one load and no arithmetic.
    pub fn due(&self, step: u64) -> bool {
        if !self.is_armed() {
            return false;
        }
        cadence_due(
            true,
            self.first_anchor_step.load(Ordering::Relaxed),
            self.cadence_steps.load(Ordering::Relaxed),
            step,
        )
    }

    /// The live reservation: `(claim count, reserved bytes)`, DERIVED from the claim
    /// table by summing every slot SOMEBODY HOLDS — anything that is not
    /// [`SlotState::Free`].
    ///
    /// # Why held, not published
    ///
    /// A slot counts from the instant its ownership CAS wins it, not from the instant
    /// it is published. Counting `Active` alone leaves a window — the claim's own body
    /// — in which a peer walking the table sees nothing while a fork is a few
    /// instructions away, and both gates this feeds are asymmetric in the same
    /// direction: the busy-worker gate skips the anchor when the count is nonzero, and the
    /// memory-headroom gate skips when the bytes leave too little headroom. OVER-reporting costs one
    /// skipped anchor at a cadence measured in tens of thousands of steps;
    /// UNDER-reporting lets two workers fork against the same memory, which is the OOM
    /// the memory-headroom gate exists to prevent. The window is spent on the safe side.
    ///
    /// So the COUNT is exact from the winning CAS — there is no observable moment in
    /// which a held slot is uncounted. The BYTES lag it by one relaxed store (a claim
    /// writes `reserved_bytes` after taking the slot; it cannot write them before,
    /// having no title to the slot yet), so a claim caught inside that window
    /// contributes its count with zero bytes. Closing it needs a 128-bit exchange the
    /// portable atomics do not offer; what is achievable is that the window is one
    /// store rather than the whole claim body, and that the gate whose failure mode is
    /// an all-or-nothing anchor is exact.
    ///
    /// # Why derived rather than cached, and what that closes
    ///
    /// A cached aggregate is a SECOND representation of what the table already says,
    /// and every hazard it invites comes from the two disagreeing. The first
    /// is a scan-derived total STORED over a peer's concurrent update. The second
    /// is worse, because no ordering can fix it: a process
    /// SIGKILLed mid-claim leaves a slot whose aggregate contribution is UNKNOWABLE —
    /// the claim writes its fields and then updates the aggregate, and a crash can
    /// land on either side of that, or (with two cached words) BETWEEN them, leaving
    /// a half-applied pair. A reclaim tail would have to GUESS what to subtract.
    ///
    /// Interposing a state discriminant does NOT resolve it: the discriminant and the
    /// aggregate are separate memory operations, so a crash can land between them
    /// whichever order they are written in. Making the aggregate crash-atomic with the
    /// slot needs an undo log — or removing the second copy, which is this.
    ///
    /// Derived, an interrupted claim contributes NOTHING by construction (it is not
    /// `Active`), reclaiming it is a pure slot operation with no arithmetic to undo,
    /// and the stored-total race is unrepresentable because nothing is ever stored.
    ///
    /// # The cost
    ///
    /// The busy-worker gate is not "one relaxed load": it is a walk of
    /// [`STATE_CLAIM_SLOTS`] contiguous slots (512 B, 8 cache lines). The gate
    /// is still NEGLIGIBLE, for this reason:
    /// this runs once per ANCHOR (inside `take_anchor`, gated by
    /// [`due`](Self::due)), NOT once per step boundary. At the shipping cadence
    /// that is once per 30 000 steps. The PER-BOUNDARY check is exactly one
    /// relaxed load of `armed`.
    ///
    /// # What it does not promise
    ///
    /// The walk is not atomic across slots, so a concurrent claim or release makes it
    /// a torn sum. That is the same guarantee a cached word would give (it would be updated
    /// per slot too), and the memory-headroom gate reads it as a floor check against a
    /// slowly-moving quantity.
    pub fn reservation(&self) -> (u32, u64) {
        let mut live = 0u32;
        let mut bytes = 0u64;
        for slot in 0..STATE_CLAIM_SLOTS {
            // Through the ONE ordered read path — see `claim_view`.
            if let Some(view) = self.claim_view(slot) {
                if view.state != SlotState::Free {
                    live += 1;
                    bytes = bytes.saturating_add(view.reserved_bytes);
                }
            }
        }
        (live, bytes)
    }

    /// The busy-worker gate: how many workers currently HOLD a claim slot — published or
    /// still in flight. See [`reservation`](Self::reservation) for why held rather
    /// than published.
    ///
    /// A caller needing both halves should use [`reservation`](Self::reservation),
    /// which answers in ONE walk.
    pub fn busy_workers(&self) -> u32 {
        self.reservation().0
    }

    /// Total bytes reserved across live claims.
    pub fn rss_reserved(&self) -> u64 {
        self.reservation().1
    }

    /// The number of claim slots this word carries.
    pub fn slot_count(&self) -> usize {
        STATE_CLAIM_SLOTS
    }

    /// Read `slot` as ONE ordered observation. `None` for an out-of-range index.
    ///
    /// # This is the ONLY read path, and that is the point
    ///
    /// The state and the pid ride ONE atomic (see [`StateClaimSlot`]), so they are
    /// read together or not at all — a slot can never be observed held-but-anonymous,
    /// nor can a state be paired with a different claim's pid.
    ///
    /// `reserved_bytes` is a separate word, written `Relaxed` by whichever party owns
    /// the slot and published by the `Release` exchange of `owner`. Release/Acquire
    /// pairs synchronize only when they are on the SAME atomic, so an `Acquire` load
    /// of `reserved_bytes` would pair with a `Release` store TO RESERVED_BYTES — which
    /// never happens — and establish NOTHING. A reader that loads it without first
    /// `Acquire`-loading `owner` is unordered against the write that produced it and
    /// may, on a weakly-ordered target, observe a stale value for a live claim.
    ///
    /// The guard is STRUCTURAL rather than a convention: `owner` is loaded `Acquire`
    /// FIRST and both of its halves are RETURNED, so the load cannot be elided, and
    /// every public accessor delegates here — an unordered read is unrepresentable
    /// through this type. Pinned by `state_arm_test`'s source walk, which fails if a
    /// second read site appears.
    ///
    /// # What this does NOT promise
    ///
    /// The two WORDS are one ORDERED read, not one ATOMIC read: a slot can be released
    /// and re-claimed between them, so a view may pair an owner with a later claim's
    /// byte count. It is a DIAGNOSTIC snapshot (Principle #3), and every decision path
    /// — `claim`, `release`, the sweep — resolves through a compare-exchange on the
    /// ownership word instead of a read.
    pub fn claim_view(&self, slot: usize) -> Option<ClaimView> {
        let s = self.claims.get(slot)?;
        // THE EDGE. The byte read below is `Relaxed` precisely because this load makes
        // that write visible; reordering these two lines reintroduces the hazard the
        // doc above describes.
        let owner = s.owner.load(Ordering::Acquire);
        let reserved_bytes = s.reserved_bytes.load(Ordering::Relaxed);
        Some(ClaimView {
            state: SlotState::from_wire(owner_tag(owner)),
            pid: owner_pid(owner),
            reserved_bytes,
        })
    }

    /// The pid holding `slot`, or 0 if free. `None` for an out-of-range index.
    ///
    /// Delegates to [`claim_view`](Self::claim_view) — see there for the ordering
    /// contract. A caller that needs more than one field should read the view once
    /// rather than call two accessors.
    pub fn claim_pid(&self, slot: usize) -> Option<i32> {
        self.claim_view(slot).map(|v| v.pid)
    }

    /// The bytes reserved by `slot`. `None` for an out-of-range index.
    pub fn claim_reserved(&self, slot: usize) -> Option<u64> {
        self.claim_view(slot).map(|v| v.reserved_bytes)
    }

    /// The lifecycle state of `slot`. `None` for an out-of-range index.
    ///
    /// The Principle #3 window onto the residual: a slot reported
    /// [`SlotState::is_in_flight`] across successive observations is one whose owner
    /// died mid-transition (see [`claim`](Self::claim)'s residual note).
    pub fn claim_state(&self, slot: usize) -> Option<SlotState> {
        self.claim_view(slot).map(|v| v.state)
    }

    /// Claim a slot for `pid`, reserving `reserved_bytes`.
    ///
    /// Returns the slot index, or `None` if the table is full — which the caller must
    /// treat as "do not fork", never as a free claim: a claim nobody recorded is a
    /// reservation nobody can release.
    ///
    /// `pid` must be a real process id (`> 0`); a non-positive one is REFUSED, because
    /// it cannot be liveness-tested: `kill(0, 0)` signals the CALLER'S OWN process
    /// group and `kill(-N, 0)` signals group `N`, so either would report a dead
    /// claimant ALIVE and its slot would never be swept. `0` is additionally this
    /// table's "no owner" marker.
    ///
    /// # The protocol, and why it is a CAS rather than an ordering
    ///
    /// The slot's `owner` word is its single OWNERSHIP TOKEN, and it carries the
    /// holder's identity (see [`StateClaimSlot`]). Every transition is a
    /// compare-exchange on it, so for each claim EXACTLY ONE party ever runs the
    /// initialisation and exactly one ever runs the teardown.
    ///
    /// Claiming is: CAS `(Free, 0) → (Claiming, pid)` (this wins the slot AND names
    /// its owner, in one instruction) · write `reserved_bytes` · CAS
    /// `(Claiming, pid) → (Active, pid)` (this publishes it).
    ///
    /// # The ordering argument
    ///
    /// **A claim is counted from the instant it is WON, not from the instant it is
    /// published** — [`reservation`](Self::reservation) sums every slot that is not
    /// `Free`, so there is no observable moment in which a peer can see a held slot it
    /// is not counting. That direction is deliberate and asymmetric: over-reporting
    /// makes a peer DECLINE to fork, which is safe; under-reporting would let it fork
    /// against memory this process has already committed to, which is the OOM the
    /// memory-headroom gate exists to prevent.
    ///
    /// **A reader never sees a half-initialised claim.** `Active` is published by a
    /// `Release` exchange and every reader `Acquire`-loads `owner` first, so observing
    /// `Active` happens-after the `reserved_bytes` write.
    ///
    /// **The publish is a CAS, not a store.** A sweep that judged this pid dead may
    /// have taken the slot while this claim was in its body; a blind store would
    /// overwrite that reclaim and leave two parties believing they own one slot. The
    /// exchange fails instead, and the claim — which now holds nothing, its bytes
    /// already cleared by the reclaimer — moves on to another slot.
    ///
    /// # Residual, stated because it is real
    ///
    /// A process SIGKILLed anywhere inside this body leaves the slot held in a
    /// transient state, but never ANONYMOUS: its owner is named by the same
    /// instruction that took the slot, so the sweep can test liveness and reclaim it
    /// ([`sweep_stale_claims`](Self::sweep_stale_claims)). What remains is the limit of
    /// a pid-based liveness predicate — a claimant's pid RECYCLED to an unrelated live
    /// process reads as alive and is never reclaimed. That needs a start-time or a
    /// generation to close, it is bounded by the arm word's lifetime (a recorder
    /// re-attach `shm_unlink`s and recreates the table — see
    /// [`MappedStateArm::create_owned`]), and it is not the ratchet: a recycled pid
    /// requires the OS to wrap its whole pid space between a claim and a sweep.
    pub fn claim(&self, pid: i32, reserved_bytes: u64) -> Option<usize> {
        self.claim_inner(pid, reserved_bytes, || {}, || {})
    }

    /// TEST SEAM: [`claim`](Self::claim) with a hook that runs at the instant the
    /// slot becomes OBSERVABLE to a peer — immediately after the `Release`-store that
    /// publishes it.
    ///
    /// It exists to pin the ordering rule: the reservation must already
    /// include this claim by then, because a peer that can SEE the slot and not COUNT
    /// it will fork against memory this process has committed to. That window is
    /// nanoseconds wide and unreachable single-threaded through the public API, so the
    /// hook stands in for the racing peer — the same reason
    /// [`sweep_stale_claims_with_hook_for_test`](Self::sweep_stale_claims_with_hook_for_test)
    /// exists. The hook is defined by its POSITION (just past publication), so an
    /// implementation that publishes before committing carries it along and is caught.
    /// Zero production impact: `claim` routes through this body with a no-op closure.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn claim_with_hook_for_test(
        &self,
        pid: i32,
        reserved_bytes: u64,
        on_published: impl FnMut(),
    ) -> Option<usize> {
        self.claim_inner(pid, reserved_bytes, || {}, on_published)
    }

    /// TEST SEAM: [`claim`](Self::claim) with a hook that runs at the instant the slot
    /// is WON — immediately after the ownership CAS, before anything else is written.
    ///
    /// It stands in for a peer observing the table at the one instant that decides
    /// whether a claim slot can leak: a protocol that names the owner in the CAS is
    /// attributable here, and one that stores the pid afterwards is ANONYMOUS here and
    /// leaves an unreclaimable slot if the process dies. The hook is defined by its
    /// POSITION, so an implementation that reverts to a separate pid store carries it
    /// along and is caught. Zero production impact: `claim` routes through this body
    /// with no-op closures.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn claim_with_owned_hook_for_test(
        &self,
        pid: i32,
        reserved_bytes: u64,
        on_owned: impl FnMut(),
    ) -> Option<usize> {
        self.claim_inner(pid, reserved_bytes, on_owned, || {})
    }

    fn claim_inner(
        &self,
        pid: i32,
        reserved_bytes: u64,
        mut on_owned: impl FnMut(),
        mut on_published: impl FnMut(),
    ) -> Option<usize> {
        // A claim must be ATTRIBUTABLE or the slot it takes is unreclaimable. See
        // `PID_NONE` for why a non-positive pid is not.
        if pid <= PID_NONE {
            debug_assert!(
                pid > PID_NONE,
                "a claim's pid must be a real process id (> 0); {pid} cannot be \
                 liveness-tested and 0 is this table's no-owner marker"
            );
            return None;
        }
        let owned = pack_owner(SLOT_CLAIMING, pid);
        let published = pack_owner(SLOT_ACTIVE, pid);
        for (i, slot) in self.claims.iter().enumerate() {
            if slot
                .owner
                .compare_exchange(OWNER_FREE, owned, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }
            // WON AND NAMED in that one instruction. There is no window in which this
            // slot is held by an owner a sweep cannot identify, and — because
            // `reservation()` counts every held slot — none in which a peer can see it
            // uncounted either.
            on_owned();
            slot.reserved_bytes.store(reserved_bytes, Ordering::Relaxed);
            // Publish by EXCHANGE, not a store: a sweep that judged this pid dead may
            // have taken the slot out from under us, and a blind store would overwrite
            // its reclaim — leaving two parties believing they own one slot. Losing
            // means we hold nothing (the reclaimer cleared our bytes and freed the
            // slot), so carry on to another one.
            //
            // The byte count we wrote above is left behind on that abandoned slot. It
            // is not ours to clear — a new claimer may already own the slot, and
            // writing into it would be exactly the overwrite this exchange prevents —
            // and it is harmless: a FREE slot contributes nothing to `reservation()`,
            // and the next claimer overwrites it one store after taking the slot,
            // inside the same window the byte lag is documented for.
            if slot
                .owner
                .compare_exchange(owned, published, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }
            on_published();
            return Some(i);
        }
        None
    }

    /// Release a slot claimed by [`claim`](Self::claim), returning the bytes it was
    /// holding.
    ///
    /// `releaser_pid` is the pid of the process CALLING this — not the claimant's. See
    /// below; it must be a real process id (`> 0`) for the same reason
    /// [`claim`](Self::claim) demands one.
    ///
    /// Takes the slot by exchanging the EXACT ownership word it observed for
    /// `(Releasing, releaser_pid)`, so a release RACING a sweep that has judged the
    /// same slot dead resolves to exactly one winner: the loser returns `None` and
    /// changes nothing. Releasing an already-free, mid-transition or out-of-range slot
    /// returns `None` for the same reason — a double release must not free a
    /// reservation twice and hand a peer a budget nobody gave back.
    ///
    /// Exchanging the whole word (rather than the tag alone) also makes this
    /// IDENTITY-CHECKED: if the slot was released and re-claimed by a different
    /// process between the load and the exchange, the exchange fails rather than
    /// tearing down the newcomer's live claim.
    ///
    /// # Why the RELEASER's pid, and not the claimant's
    ///
    /// The pid in a held word answers exactly one question: **who must be alive for
    /// this transition to finish?** While a slot is `Releasing`, that is whoever is
    /// running the teardown — which is NOT always the claimant. This function takes a
    /// slot INDEX and checks no ownership, and the shipping release shape is the
    /// PARENT releasing a forked child's claim on reap, so a live recorder is
    /// routinely mid-teardown on a claim whose claimant is genuinely dead.
    ///
    /// Stamping the claimant there would make that ordinary case look exactly like an
    /// ABANDONED release: a concurrent sweep reads `(Releasing, dead_claimant)`, finds
    /// the pid dead — truthfully — reclaims the slot, and a new claimant takes it,
    /// while the live releaser is still inside its own tail. That is a live claim
    /// erased, with no misjudgement anywhere. Stamping the releaser removes the case
    /// rather than defending against it: the sweep tests the liveness of the party
    /// that can actually strand the slot, and a releaser that dies mid-tail is still
    /// named, so its slot is still reclaimable.
    ///
    /// The teardown tail's identity-checked publish is the backstop for what this rule
    /// cannot exclude — a liveness predicate that calls a LIVE holder dead. It
    /// exchanges on the exact word it owns, so a tail whose slot was reclaimed under it
    /// ABORTS (returning `None`) instead of publishing FREE over whoever holds the slot
    /// now.
    pub fn release(&self, slot: usize, releaser_pid: i32) -> Option<u64> {
        self.release_inner(slot, releaser_pid, || {})
    }

    /// TEST SEAM: [`release`](Self::release) with a hook that runs INSIDE the teardown
    /// tail — after the reservation is given back and before the slot is published
    /// FREE.
    ///
    /// That instant is the whole of the reclaim/release race: it is the only point
    /// from which a peer can reclaim a slot whose teardown is already in flight, and
    /// it is a few instructions wide, so no test can reach it through the public API.
    /// The hook is defined by its POSITION, so an implementation that publishes FREE
    /// with a blind store carries it along and is caught. Zero production impact:
    /// `release` routes through this body with a no-op closure.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn release_with_tail_hook_for_test(
        &self,
        slot: usize,
        releaser_pid: i32,
        on_tail: impl FnMut(),
    ) -> Option<u64> {
        self.release_inner(slot, releaser_pid, on_tail)
    }

    fn release_inner(&self, slot: usize, releaser_pid: i32, on_tail: impl FnMut()) -> Option<u64> {
        if releaser_pid <= PID_NONE {
            debug_assert!(
                releaser_pid > PID_NONE,
                "a releaser's pid must be a real process id (> 0); {releaser_pid} \
                 cannot be liveness-tested and 0 is this table's no-owner marker"
            );
            return None;
        }
        let s = self.claims.get(slot)?;
        let owner = s.owner.load(Ordering::Acquire);
        if owner_tag(owner) != SLOT_ACTIVE {
            return None;
        }
        let owned = pack_owner(SLOT_RELEASING, releaser_pid);
        if s.owner
            .compare_exchange(owner, owned, Ordering::AcqRel, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        self.finish_release_hooked(s, owned, on_tail)
    }

    /// Finish a teardown whose ownership exchange THIS caller won, `owned` being the
    /// exact word it exchanged INTO. `None` if the slot was taken away mid-tail.
    ///
    /// The shared tail of [`release`](Self::release) and the sweep's dead-owner arm:
    /// one body, so the two paths cannot drift in what they give back.
    ///
    /// # The publish is an EXCHANGE, and a blind store here is a live-claim ERASER
    ///
    /// The tail is not instantaneous, and while it runs a sweep can judge this slot's
    /// holder dead and reclaim it. The reclaimer then frees the slot, a NEW claimant
    /// takes it, and a blind `store(OWNER_FREE)` from the original tail lands on top —
    /// erasing a live claim whose owner still believes it holds the slot, and whose
    /// reservation vanishes from every peer's fork gate. Exchanging on `owned` makes
    /// the losing tail abort instead, leaving the slot to whoever legitimately holds
    /// it now.
    ///
    /// [`release`](Self::release) stamping the RELEASER (rather than the claimant)
    /// keeps that interleave off the shipping path entirely — see there. This is the
    /// backstop for the case that rule cannot exclude: a liveness predicate that calls
    /// a live holder dead, i.e. the recycled-pid residual. Consistent with the rest of
    /// the module, the decision resolves through a compare-exchange rather than
    /// through trust.
    ///
    /// The bytes go back BEFORE the slot does, so a peer's walk can see this slot
    /// holding nothing but never see it FREE while it still reports a reservation.
    /// A losing tail has therefore already zeroed bytes the winner will report as 0;
    /// that costs the winner's `freed_bytes` an under-report on a path that requires a
    /// lying predicate to reach at all, and it is the safe direction — the memory is
    /// genuinely free either way, and no live claim is touched.
    fn finish_release(&self, slot: &StateClaimSlot, owned: u64) -> Option<u64> {
        self.finish_release_hooked(slot, owned, || {})
    }

    fn finish_release_hooked(
        &self,
        slot: &StateClaimSlot,
        owned: u64,
        mut on_tail: impl FnMut(),
    ) -> Option<u64> {
        let bytes = slot.reserved_bytes.swap(0, Ordering::AcqRel);
        on_tail();
        slot.owner
            .compare_exchange(owned, OWNER_FREE, Ordering::AcqRel, Ordering::Relaxed)
            .ok()?;
        Some(bytes)
    }

    /// THE STALE-CLAIM SWEEP: release every slot whose holder is no longer alive — in
    /// WHICHEVER state it died.
    ///
    /// Without it, a worker dying before its reap — the ordinary shape under
    /// `--peer-loss continue` — freezes every survivor's cadence as `StillEncoding`
    /// forever, which converts the all-or-nothing anchor into
    /// nothing-forever-SILENTLY.
    ///
    /// # Every held state, not just the published one
    ///
    /// A slot is reclaimable from `Active`, `Claiming` AND `Releasing`, because the
    /// ownership word names its holder in every one of them (see [`StateClaimSlot`]).
    /// Reclaiming only published claims would leave the two transient states as a
    /// finite-table RATCHET — narrower than the published-claim one, and with the
    /// same terminal outcome.
    ///
    /// # It RELEASES; it does not PUBLISH a total
    ///
    /// A dead claim is cleared through exactly the same CAS-guarded
    /// release tail an ordinary [`release`](Self::release) uses, so the sweep
    /// composes with concurrent claims and releases the way two releases compose. It
    /// deliberately does NOT derive aggregate totals from its scan and store them:
    /// a scan is not atomic, so any total derived from one is stale the moment a peer
    /// claims or releases, and storing it would silently overwrite that peer's update
    /// — leaving a ghost reservation (blocking every future fork) or an underflowed
    /// one (permitting forks against memory that is spoken for). [`live`] is therefore
    /// an OBSERVATION reported to the caller, never an authority written back.
    ///
    /// # `is_alive` is injected
    ///
    /// The production predicate is `kill(pid, 0) != ESRCH`. Taking it as an argument
    /// keeps the syscall out of the decision this function makes — the RULE (which
    /// claims are stale, what happens to their reservations, who wins a race with a
    /// concurrent release) is what needs pinning, and reverting to a live-syscall path
    /// would inherit that syscall's blast radius.
    ///
    /// Returns what was cleared; the caller logs it LOUDLY when
    /// [`StaleClaimSweep::is_empty`] is false.
    ///
    /// [`live`]: StaleClaimSweep::live
    pub fn sweep_stale_claims(&self, is_alive: impl Fn(i32) -> bool) -> StaleClaimSweep {
        self.sweep_inner(is_alive, || {})
    }

    /// TEST SEAM: [`sweep_stale_claims`](Self::sweep_stale_claims) with a hook that
    /// runs AFTER the slot walk and BEFORE the function returns — exactly where a
    /// scan-then-publish implementation would store its derived totals.
    ///
    /// It exists SOLELY to pin the no-stale-publication rule deterministically: the
    /// hazard needs a claim or a release to land between the scan and the
    /// publication, which cannot be forced single-threaded through the public API.
    /// The `drain_with_pre_commit_hook_for_test` seam in [`crate::trace_ring`] is the
    /// same pattern for the same reason. Zero production impact: `sweep_stale_claims`
    /// routes through this body with a no-op closure that inlines away.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn sweep_stale_claims_with_hook_for_test(
        &self,
        is_alive: impl Fn(i32) -> bool,
        after_scan: impl FnOnce(),
    ) -> StaleClaimSweep {
        self.sweep_inner(is_alive, after_scan)
    }

    fn sweep_inner(
        &self,
        is_alive: impl Fn(i32) -> bool,
        after_scan: impl FnOnce(),
    ) -> StaleClaimSweep {
        let mut sweep = StaleClaimSweep::default();
        for slot in &self.claims {
            // ONE ordered read of the ownership word, and the exchange below uses THIS
            // value: a slot released and re-claimed while we were judging it fails the
            // exchange instead of having a live newcomer's claim torn down.
            let owner = slot.owner.load(Ordering::Acquire);
            let state = SlotState::from_wire(owner_tag(owner));
            let pid = owner_pid(owner);
            match state {
                SlotState::Free => continue,
                // A state this build cannot name is not ours to judge.
                SlotState::Unrecognized(_) => {
                    sweep.in_flight += 1;
                    continue;
                }
                SlotState::Claiming | SlotState::Active | SlotState::Releasing => {}
            }
            if pid == PID_NONE {
                // A reclaim already in progress (`OWNER_RECLAIMING`) — another sweeper
                // owns this slot's teardown. Unreachable for Claiming/Active, which
                // `claim` refuses to create without a real pid.
                sweep.in_flight += 1;
                continue;
            }
            if is_alive(pid) {
                if state == SlotState::Active {
                    sweep.live += 1;
                } else {
                    // A live peer mid-transition; it will finish in nanoseconds.
                    sweep.in_flight += 1;
                }
                continue;
            }
            // A DEAD owner, in whichever state it died. Take the slot with one
            // identity-checked exchange onto a marker no live participant can write
            // (a claimant's pid is always positive), so exactly one sweeper proceeds.
            // Losing means the owner, or another sweeper, got there first — nothing to
            // clear.
            //
            // Every held state is reclaimable BECAUSE identity rides in the ownership
            // word: there is no "held by nobody nameable" state to strand a slot in.
            // The table is FINITE, so anything unreclaimable here is a permanent leak
            // that ends in machine-wide checkpoint death.
            if slot
                .owner
                .compare_exchange(owner, OWNER_RECLAIMING, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }
            // The tail publishes FREE by the same identity-checked exchange every
            // teardown uses, so it reports what it did rather than assuming. It cannot
            // currently lose — `OWNER_RECLAIMING` is written only here, and the
            // `pid == PID_NONE` guard above stops a second sweeper judging a slot
            // already in reclaim — so this arm is unreachable today; it is handled
            // rather than asserted so the two teardown paths share ONE body and
            // neither has to carry its own reasoning about who may finish it.
            let Some(freed) = self.finish_release(slot, OWNER_RECLAIMING) else {
                continue;
            };
            sweep.freed_bytes = sweep.freed_bytes.saturating_add(freed);
            if state == SlotState::Active {
                sweep.cleared += 1;
            } else {
                sweep.interrupted_reclaimed += 1;
            }
        }
        after_scan();
        sweep
    }
}

/// Derive the POSIX SHM object name for `tag`: `/cer_sta_<fnv1a64(tag):016x>`.
///
/// Fixed-length hex keeps the name PREFIX-FREE against `shm_ring`'s `/cer_rg_`,
/// `barrier`'s `/cer_bar_` and `doorbell`'s `/cer_db_` (there is a known iceoryx2
/// hazard with string-prefix collisions) and ≤ 31 chars for macOS's `PSHMNAMLEN`
/// (`/cer_sta_` = 9 + 16 = 25). Pure — hermetically testable on any OS.
pub fn state_arm_shm_name(tag: &str) -> String {
    format!("/cer_sta_{:016x}", fnv1a64(tag.as_bytes()))
}

/// The checkpoint arm TAG a run is known by.
///
/// # Why a run's tag is DERIVED rather than agreed
///
/// The tag is the token the whole checkpoint plane meets on: the arm word is
/// named by it, and so is every rank's state ring
/// ([`crate::state_ring::state_ring_tag`]). Were both halves handed
/// the same string BEFORE the graph started, the mid-run case would be
/// unservable — a recorder attaching to a live run (`cerulion bag record --run`)
/// has no way to learn a name nobody wrote down.
///
/// A run's `run_id` is already the thing both sides can independently know: the
/// graph mints it at launch and publishes it in `run.json`, and a recorder that
/// resolves a run has read exactly that file. Deriving the tag from it means the
/// two halves agree with NO coordination at all.
///
/// # It is a `run_id`, not a name
///
/// The graph NAME is not usable here: two runs of one graph would collide, and a
/// run is precisely what a checkpoint plane must not confuse. `run_id` is minted
/// per run (`transport::run_registry::mint_run_id`), so the derived tag inherits
/// that uniqueness.
///
/// # What has NO run_id, and what that costs
///
/// A state-bearing process that is not a `graph run` — an `rmw_cerulion`-hosted
/// ROS 2 process is the shape that exists today — mints no run and writes no
/// `run.json`, so nothing here can be derived for it. That is why the explicit
/// `CERULION_STATE_ARM_TAG` override REMAINS: it is not a legacy escape hatch,
/// it is the ONLY way to name such a process's checkpoint plane, and a shape
/// with neither a run nor an explicit tag is one this system refuses to arm
/// rather than arming under a name it invented.
pub fn state_arm_tag_for_run(run_id: u128) -> String {
    format!("cer_run_{run_id:032x}")
}

/// A process-shared SHM mapping of a [`StateArmWord`].
///
/// Construct via [`create_owned`](MappedStateArm::create_owned) (the recorder —
/// `O_EXCL`-creates, sizes, initialises, owns the name and `shm_unlink`s it on drop)
/// or [`open_unowned`](MappedStateArm::open_unowned) (a graph process — STRICT
/// open-existing, maps only). [`Deref`](std::ops::Deref)s to the word, so
/// `due`/`claim`/… reach through into the shared page.
///
/// As with [`crate::barrier`]: `shm_unlink` removes only the NAME, so an owner
/// dropping mid-run can never invalidate a peer's live mapping — the object persists
/// until the last `munmap`.
#[must_use = "the mapping is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local"]
pub struct MappedStateArm {
    ptr: *mut StateArmWord,
    name: std::ffi::CString,
    name_str: String,
    owns_name: bool,
}

// SAFETY: the mapped object is a `StateArmWord` (atomics plus two fields written
// once before the magic publishes them) in a shared page. All post-init access goes
// through atomic ops, so it is sound to send/share the handle across threads; the OS
// keeps the MAP_SHARED page coherent across mappings.
unsafe impl Send for MappedStateArm {}
unsafe impl Sync for MappedStateArm {}

impl MappedStateArm {
    /// Create + map + initialise the arm word for `tag` as its OWNER.
    ///
    /// A pre-existing orphan of the same name (a crashed prior recorder) is
    /// `shm_unlink`ed first so the `O_EXCL` create yields a fresh, zero-filled
    /// object — including a zeroed claim table, so a crashed run's claims can never
    /// be inherited as live ones.
    pub fn create_owned(tag: &str) -> std::io::Result<Self> {
        let name_str = state_arm_shm_name(tag);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;

        // Unlink-first (so a crashed prior recorder's orphan is cleared and the
        // `O_EXCL` create yields a fresh, ZERO-FILLED object — including a
        // zeroed claim table, so a crashed run's claims can never be inherited
        // as live ones) + create + `ftruncate` EXACTLY ONCE + `mmap`, cleanup
        // arms included: `shm_map::create_exclusive`.
        let addr = create_exclusive(&name, STATE_ARM_BYTES)?;
        let ptr = addr as *mut StateArmWord;
        // SAFETY: `ptr` is a freshly-mapped, exclusively-owned (O_EXCL), page-aligned
        // (≥ the 8-byte alignment the word needs) zero-filled region ≥
        // size_of::<StateArmWord>() (const-asserted above).
        unsafe { (*ptr).init() };
        // Fork exclusion, AT BIRTH: a capture child must not inherit the
        // live claim table — its peers are reading and writing it right now.
        // SAFETY: the region just mapped and exclusively owned here.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                addr,
                STATE_ARM_BYTES,
                crate::state_carrier::ForkExcludedMapping::StateArmWord,
            );
        }
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: true,
        })
    }

    /// Open + map an EXISTING arm word for `tag` as a peer.
    ///
    /// STRICT open-existing (`O_RDWR`, no `O_CREAT`), so a missing object is an
    /// error rather than a silently-created word nobody armed. The mapped header is
    /// validated (magic / version / slot count) before the handle is returned, so a
    /// racing open either fails clean or observes a fully initialised word.
    pub fn open_unowned(tag: &str) -> std::io::Result<Self> {
        let name_str = state_arm_shm_name(tag);
        let name = std::ffi::CString::new(name_str.clone())
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        // STRICT open-existing (`O_RDWR`, no `O_CREAT`); `seg`'s Drop closes the
        // descriptor on every early return below.
        let seg = OpenedSegment::open(&name)?;
        let size = seg.size()?;
        if (size as usize) < STATE_ARM_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "state arm '{name_str}' is {size} bytes, short of the \
                     {STATE_ARM_BYTES}-byte word"
                ),
            ));
        }
        let addr = seg.map_shared(STATE_ARM_BYTES)?;
        let ptr = addr as *mut StateArmWord;
        // SAFETY: `ptr` maps ≥ STATE_ARM_BYTES of a live object; `validate` reads
        // only initialised-or-zero words and never indexes the claim table.
        if let Err(reason) = unsafe { (*ptr).validate() } {
            // SAFETY: unmap exactly what we just mapped; nothing references it.
            unsafe { unmap(addr, STATE_ARM_BYTES) };
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("state arm '{name_str}' failed validation: {reason}"),
            ));
        }
        // The fork exclusion applies to a PEER mapping too: it is per-mapping, and
        // this process forks its own capture children regardless of who created the
        // segment.
        // SAFETY: the region just mapped and exclusively owned by this handle.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                addr,
                STATE_ARM_BYTES,
                crate::state_carrier::ForkExcludedMapping::StateArmWord,
            );
        }
        Ok(Self {
            ptr,
            name,
            name_str,
            owns_name: false,
        })
    }

    /// The POSIX SHM object name — hand this to the graph processes.
    pub fn name(&self) -> &str {
        &self.name_str
    }

    /// The mapping's base address and length — exactly the region the carrier's
    /// `MADV_DONTFORK` sweep must cover, and nothing else.
    pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
        (self.ptr as *mut std::ffi::c_void, STATE_ARM_BYTES)
    }
}

impl std::ops::Deref for MappedStateArm {
    type Target = StateArmWord;
    fn deref(&self) -> &StateArmWord {
        // SAFETY: `ptr` is a valid mapping of an initialised `StateArmWord` that
        // stays mapped for as long as `self` lives (unmapped only in `Drop`).
        unsafe { &*self.ptr }
    }
}

/// By design, the mapped word is the ONE production source of the
/// scheduler's `Period` catch-up clamp.
///
/// The read order is the load-bearing part and it is the module's own rule read
/// back: [`StateArmWord::arm`] stores `first_anchor_step` (and the cadence)
/// BEFORE `Release`-storing `armed`, so an observer that `Acquire`-loads
/// `armed` FIRST is guaranteed the step number that was published with it.
/// Taken the other way round, a reader could pair a fresh `armed` with a
/// PREVIOUS attach's onset step — and on a multi-process run that is a rank
/// clamping from the wrong step, i.e. exactly the divergence
/// [`crate::scheduler::catchup_clamp`] keys on `first_anchor_step` to prevent.
impl crate::scheduler::catchup_clamp::CatchupArm for MappedStateArm {
    fn onset(&self) -> crate::scheduler::catchup_clamp::ArmOnset {
        // `is_armed()` is the `Acquire` edge; it MUST be read first.
        let armed = self.is_armed();
        crate::scheduler::catchup_clamp::ArmOnset {
            armed,
            first_anchor_step: self.first_anchor_step(),
        }
    }
}

impl std::fmt::Debug for MappedStateArm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MappedStateArm")
            .field("name", &self.name_str)
            .field("owns_name", &self.owns_name)
            .field("armed", &self.is_armed())
            .field("busy_workers", &self.busy_workers())
            .finish()
    }
}

impl Drop for MappedStateArm {
    fn drop(&mut self) {
        // SAFETY: unmap exactly the region create_owned/open_unowned mapped;
        // nothing references it after this.
        unsafe {
            unmap(self.ptr as *mut std::ffi::c_void, STATE_ARM_BYTES);
        }
        if self.owns_name {
            // Best-effort unlink of the name this owner created. Removes the
            // NAME only — a peer's live mapping survives until its own munmap.
            unlink(&self.name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- the pure cadence decision (a prime mutation target) ----

    #[test]
    fn a_detached_word_is_never_due() {
        for step in 0..100 {
            assert!(
                !cadence_due(false, 0, 1, step),
                "a detached word is a null test — it must not anchor at ANY step"
            );
        }
    }

    #[test]
    fn the_first_anchor_step_boundary_is_pinned_on_both_sides() {
        const FIRST: u64 = 1_000;
        const CADENCE: u64 = 10;
        assert!(
            !cadence_due(true, FIRST, CADENCE, FIRST - 1),
            "the step BEFORE the first anchor is not due"
        );
        assert!(
            cadence_due(true, FIRST, CADENCE, FIRST),
            "the first anchor step IS due — the trace and frame gates are pinned to it"
        );
        assert!(!cadence_due(true, FIRST, CADENCE, FIRST + 1));
        assert!(!cadence_due(true, FIRST, CADENCE, FIRST + CADENCE - 1));
        assert!(cadence_due(true, FIRST, CADENCE, FIRST + CADENCE));
        assert!(cadence_due(true, FIRST, CADENCE, FIRST + 2 * CADENCE));
    }

    #[test]
    fn a_zero_cadence_is_one_shot_not_a_divide_by_zero() {
        assert!(cadence_due(true, 7, 0, 7));
        for step in [0u64, 6, 8, 9, 1_000_000] {
            assert!(
                !cadence_due(true, 7, 0, step),
                "a one-shot arm is due at its first step and never again (step {step})"
            );
        }
    }

    #[test]
    fn a_cadence_of_one_is_due_at_every_step_from_the_first() {
        for step in 0..10u64 {
            assert_eq!(cadence_due(true, 3, 1, step), step >= 3, "step {step}");
        }
    }

    #[test]
    fn the_shipping_cadence_example_matches_the_design() {
        // A 1 kHz graph with a 60 s ring anchors every 30 000 steps.
        const CADENCE: u64 = 30_000;
        assert!(cadence_due(true, 0, CADENCE, 0));
        assert!(!cadence_due(true, 0, CADENCE, 29_999));
        assert!(cadence_due(true, 0, CADENCE, 30_000));
        assert!(cadence_due(true, 0, CADENCE, 60_000));
    }

    // ---- the claim table ----

    /// A zero-filled word, on the heap, for the pure table tests — no SHM needed.
    fn word() -> Box<StateArmWord> {
        // SAFETY: `StateArmWord` is atomics + integers + arrays of the same, so an
        // all-zero bit pattern is a valid (and, for this type, correctly
        // initialised-looking) value; `init` then writes every field properly.
        let w: Box<StateArmWord> = unsafe { Box::new(std::mem::zeroed()) };
        w.init();
        w
    }

    #[test]
    fn a_fresh_word_is_detached_with_an_empty_table() {
        let w = word();
        assert!(!w.is_armed());
        assert_eq!(w.busy_workers(), 0);
        assert_eq!(w.rss_reserved(), 0);
        assert_eq!(w.slot_count(), STATE_CLAIM_SLOTS);
        for i in 0..STATE_CLAIM_SLOTS {
            assert_eq!(w.claim_pid(i), Some(0));
            assert_eq!(w.claim_reserved(i), Some(0));
        }
        assert_eq!(w.claim_pid(STATE_CLAIM_SLOTS), None, "out of range");
    }

    #[test]
    fn arm_publishes_the_cadence_and_disarm_takes_it_back() {
        let w = word();
        w.arm(500, 42);
        assert!(w.is_armed());
        assert_eq!(w.cadence_steps(), 500);
        assert_eq!(w.first_anchor_step(), 42);
        assert!(w.due(42));
        assert!(w.due(542));
        assert!(!w.due(43));
        w.disarm();
        assert!(!w.is_armed());
        assert!(!w.due(42), "a disarmed word anchors nothing");
        // Re-arming republishes the values first.
        w.arm(0, 900);
        assert!(w.due(900));
        assert!(!w.due(1400));
    }

    #[test]
    fn claims_and_releases_move_both_counters_exactly() {
        let w = word();
        let a = w.claim(111, 1_000).expect("slot");
        let b = w.claim(222, 2_500).expect("slot");
        assert_ne!(a, b, "two claims take two slots");
        assert_eq!(w.busy_workers(), 2);
        assert_eq!(w.rss_reserved(), 3_500);
        assert_eq!(w.claim_pid(a), Some(111));
        assert_eq!(w.claim_reserved(b), Some(2_500));

        assert_eq!(w.release(a, 111), Some(1_000));
        assert_eq!(w.busy_workers(), 1);
        assert_eq!(w.rss_reserved(), 2_500);
        assert_eq!(w.claim_pid(a), Some(0), "the slot is free again");

        // A double release must not decrement twice and hand a peer a budget nobody
        // freed.
        assert_eq!(w.release(a, 111), None);
        assert_eq!(w.busy_workers(), 1);
        assert_eq!(w.rss_reserved(), 2_500);
        assert_eq!(w.release(STATE_CLAIM_SLOTS, 111), None, "out of range");

        assert_eq!(w.release(b, 222), Some(2_500));
        assert_eq!(w.busy_workers(), 0);
        assert_eq!(w.rss_reserved(), 0);
    }

    #[test]
    fn a_full_table_refuses_rather_than_handing_out_an_untracked_claim() {
        let w = word();
        for i in 0..STATE_CLAIM_SLOTS {
            assert!(w.claim(1000 + i as i32, 1).is_some(), "slot {i}");
        }
        assert_eq!(w.busy_workers(), STATE_CLAIM_SLOTS as u32);
        assert_eq!(
            w.claim(9999, 1),
            None,
            "a full table must refuse — a claim nobody recorded is a reservation \
             nobody can release"
        );
        assert_eq!(w.busy_workers(), STATE_CLAIM_SLOTS as u32);
        assert_eq!(w.rss_reserved(), STATE_CLAIM_SLOTS as u64);
    }

    /// The invariant the reservation exists to hold: it equals what the HELD slots
    /// say. Read through the public accessors, so it is an independent oracle rather
    /// than a restatement of the code under test. Valid only at quiescence (no slot
    /// mid-transition), which every caller below asserts — so at its call sites "held"
    /// and "published" coincide, and the oracle is insensitive to which rule the
    /// implementation uses.
    fn counters_match_table(w: &StateArmWord) -> (bool, u32, u64) {
        let mut live = 0u32;
        let mut bytes = 0u64;
        for i in 0..STATE_CLAIM_SLOTS {
            assert!(
                !w.claim_state(i).expect("in range").is_in_flight(),
                "slot {i} is mid-transition — the table oracle is only valid at quiescence"
            );
            if w.claim_state(i) == Some(SlotState::Active) {
                live += 1;
                bytes += w.claim_reserved(i).expect("in range");
            }
        }
        (
            w.busy_workers() == live && w.rss_reserved() == bytes,
            live,
            bytes,
        )
    }

    // THE STALE-CLAIM RULE: a worker that died between fetch_add and reap must not freeze
    // every survivor's cadence forever.
    #[test]
    fn a_dead_claimants_slot_is_released_with_exactly_its_own_reservation() {
        let w = word();
        let dead = w.claim(4242, 700_000).expect("slot");
        let live = w.claim(4243, 300_000).expect("slot");
        assert_eq!(w.busy_workers(), 2);
        assert_eq!(w.rss_reserved(), 1_000_000);

        let sweep = w.sweep_stale_claims(|pid| pid != 4242);
        assert_eq!(
            sweep,
            StaleClaimSweep {
                cleared: 1,
                freed_bytes: 700_000,
                live: 1,
                in_flight: 0,
                interrupted_reclaimed: 0,
            }
        );
        assert!(!sweep.is_empty(), "the caller logs this LOUDLY");
        assert_eq!(w.claim_pid(dead), Some(0), "the dead claim is gone");
        assert_eq!(w.claim_state(dead), Some(SlotState::Free));
        assert_eq!(w.claim_pid(live), Some(4243), "the live claim is untouched");
        assert_eq!(w.busy_workers(), 1);
        assert_eq!(w.rss_reserved(), 300_000);
        assert!(counters_match_table(&w).0);
        // The freed slot is claimable again — the whole point.
        assert_eq!(w.claim(5555, 10), Some(dead));
    }

    #[test]
    fn a_sweep_with_every_claimant_alive_changes_nothing() {
        let w = word();
        w.claim(1, 10).expect("slot");
        w.claim(2, 20).expect("slot");
        let sweep = w.sweep_stale_claims(|_| true);
        assert_eq!(
            sweep,
            StaleClaimSweep {
                cleared: 0,
                freed_bytes: 0,
                live: 2,
                in_flight: 0,
                interrupted_reclaimed: 0,
            }
        );
        assert!(sweep.is_empty(), "nothing to log");
        assert_eq!(w.busy_workers(), 2);
        assert_eq!(w.rss_reserved(), 30);
    }

    // ---- the sweep must not publish a total derived from its own scan ----
    //
    // THE race, driven deterministically through the test hook: the intervening
    // mutation lands exactly where a scan-then-publish implementation would store its
    // derived totals. Both directions, because they fail differently — an intervening
    // CLAIM is UNDER-counted (a peer forks against memory already spoken for), an
    // intervening RELEASE is OVER-counted or UNDERFLOWED (a ghost reservation that
    // blocks every future fork). Measured on a scan-then-publish implementation:
    // counters (1, 131) against a live table of (2, 908), and (1, 0) against
    // an empty one.

    #[test]
    fn a_claim_landing_after_the_sweeps_scan_is_not_overwritten_by_it() {
        let w = word();
        let held = w.claim(11, 500).expect("slot");
        let dead = w.claim(22, 700).expect("slot");

        let sweep = w.sweep_stale_claims_with_hook_for_test(
            |pid| pid != 22,
            || {
                // A peer commits a reservation while the sweep is between its scan and
                // its return. Under scan-then-publish this claim is erased.
                w.claim(33, 900).expect("slot");
            },
        );
        assert_eq!(sweep.cleared, 1);
        assert_eq!(sweep.freed_bytes, 700);

        let (matches, live, bytes) = counters_match_table(&w);
        assert!(
            matches,
            "counters ({}, {}) must equal the live table ({live}, {bytes}) — a total \
             derived from a non-atomic scan is stale the moment a peer claims",
            w.busy_workers(),
            w.rss_reserved(),
        );
        assert_eq!((live, bytes), (2, 1_400), "the held claim plus the new one");
        assert_eq!(w.claim_pid(held), Some(11));
        // The dead claim is gone from the table. Its SLOT is deliberately not
        // asserted free: the sweep released it during the scan, so the hook's claim is
        // free to take it — which is the recycling the stale-claim sweep exists for.
        assert!(
            (0..STATE_CLAIM_SLOTS).all(|i| w.claim_pid(i) != Some(22)),
            "no slot may still be held by the dead claimant"
        );
        let _ = dead;
    }

    #[test]
    fn a_release_landing_after_the_sweeps_scan_is_not_resurrected_by_it() {
        let w = word();
        let a = w.claim(11, 500).expect("slot");
        let b = w.claim(22, 700).expect("slot");

        let sweep = w.sweep_stale_claims_with_hook_for_test(
            |_| true,
            || {
                // Both peers finish while the sweep is between its scan and its return.
                assert_eq!(w.release(a, 11), Some(500));
                assert_eq!(w.release(b, 22), Some(700));
            },
        );
        assert_eq!(sweep.live, 2, "the scan really did see two live claims");
        assert_eq!(sweep.cleared, 0);

        let (matches, live, bytes) = counters_match_table(&w);
        assert!(
            matches,
            "counters ({}, {}) must equal the now-empty table ({live}, {bytes}) — \
             storing the scan's totals here resurrects two released reservations",
            w.busy_workers(),
            w.rss_reserved(),
        );
        assert_eq!((live, bytes), (0, 0));
        assert_eq!(w.busy_workers(), 0);
        assert_eq!(w.rss_reserved(), 0, "and never an underflowed one");
    }

    // The other half of the same protocol: a sweep and a release CONTEND for ONE slot,
    // and exactly one may subtract its reservation. Driven both orders.
    //
    // ORDER 1 injects the release inside the `is_alive` predicate, which the sweep
    // calls between loading the slot's state and attempting its ownership CAS — the
    // precise window where the two collide, and one no hook placed around the scan
    // can reach.
    #[test]
    fn a_sweep_and_a_release_contending_for_one_slot_yield_exactly_one_winner() {
        let w = word();
        let doomed = w.claim(77, 400).expect("slot");
        let sweep = w.sweep_stale_claims(|_pid| {
            // The owner finishes WHILE the sweep is judging this very slot …
            assert_eq!(w.release(doomed, 77), Some(400), "the owner wins the CAS");
            // … and the sweep then decides the (now departed) claimant was dead.
            false
        });
        assert_eq!(
            sweep.cleared, 0,
            "the sweep LOST the ownership CAS, so it cleared nothing"
        );
        assert_eq!(sweep.freed_bytes, 0);
        assert_eq!(
            w.busy_workers(),
            0,
            "a second subtract here UNDERFLOWS the count — the state a blind clear \
             leaves behind, and a permanent SkippedLowMemory for every peer"
        );
        assert_eq!(w.rss_reserved(), 0);
        assert_eq!(w.claim_state(doomed), Some(SlotState::Free));

        // ORDER 2: the sweep clears a dead claim first, and the owner's LATE release
        // must find nothing to do rather than double-subtracting.
        let w = word();
        let doomed = w.claim(88, 400).expect("slot");
        let sweep = w.sweep_stale_claims(|_| false);
        assert_eq!(sweep.cleared, 1);
        assert_eq!(sweep.freed_bytes, 400);
        assert_eq!(
            w.release(doomed, 88),
            None,
            "the slot was already released by the sweep — a second subtract would \
             hand a peer a budget nobody freed"
        );
        assert_eq!(w.busy_workers(), 0);
        assert_eq!(w.rss_reserved(), 0);
    }

    // A slot ANOTHER party is mid-transition on is not ours to release. This is the
    // half `release`'s ownership CAS carries and the two contention orders above
    // cannot reach: there the peer's teardown completes atomically from this thread's
    // point of view, whereas a real peer is descheduled part-way through it, leaving
    // the slot RELEASING with its pid still set — which a pid-presence check reads as
    // "a live claim, mine to free", freeing a reservation its owner will free again.
    #[test]
    fn a_slot_another_party_is_mid_transition_on_is_not_releasable() {
        for frozen in [SLOT_RELEASING, SLOT_CLAIMING] {
            let w = word();
            let slot = w.claim(99, 250).expect("slot");
            assert_eq!(w.busy_workers(), 1);
            // A peer holds it and has not finished; its pid is still set.
            w.claims[slot]
                .owner
                .store(pack_owner(frozen, 99), Ordering::Release);
            assert_eq!(
                w.claim_pid(slot),
                Some(99),
                "a held slot always names its holder"
            );

            assert_eq!(
                w.release(slot, 99),
                None,
                "state {frozen}: a slot held by another party must not be released here"
            );
            assert_eq!(
                w.claim_state(slot).map(|s| s.as_wire()),
                Some(frozen),
                "state {frozen}: the holder still holds it — a release that took it \
                 would race the holder's own teardown"
            );
            // The reservation still counts it: the memory is spoken for until whoever
            // holds the slot gives it back, and a peer that stopped counting a held
            // claim would fork against it.
            assert_eq!(
                w.reservation(),
                (1, 250),
                "state {frozen}: a HELD slot is a live reservation"
            );
        }
    }

    // The ordering rule: a peer that can see a
    // published slot must already COUNT it. The hook fires at the instant the slot
    // becomes observable, which is where a racing peer would read.
    //
    // The direction is what matters and it is not symmetric: over-reporting makes a
    // peer DECLINE to fork (safe), under-reporting lets it fork against memory this
    // process has already committed to (the OOM the memory-headroom gate exists to prevent).
    #[test]
    fn a_slot_is_counted_before_it_becomes_observable_never_after() {
        let w = word();
        let existing = w.claim(11, 500).expect("slot");
        let mut observed: Option<(u32, u64)> = None;
        {
            let seen = &mut observed;
            let w_peer = &*w;
            let slot = w
                .claim_with_hook_for_test(22, 700, || {
                    // A peer reading at the moment of publication.
                    *seen = Some((w_peer.busy_workers(), w_peer.rss_reserved()));
                })
                .expect("slot");
            assert_ne!(slot, existing);
        }
        assert_eq!(
            observed,
            Some((2, 1_200)),
            "at the instant the slot became readable, the aggregates must ALREADY \
             include it — publishing first and committing after leaves a window in \
             which a peer sees a claim it is not counting"
        );
    }

    /// Freeze `slot` as if its owner were SIGKILLed mid-transition, in `tag`
    /// (`SLOT_CLAIMING` or `SLOT_RELEASING`), keeping the holder it already has.
    ///
    /// There is deliberately no "anonymous" variant: the ownership word carries the
    /// pid, so a held-but-unnameable slot is not a state this protocol can reach.
    /// Reaching it needs a raw store of a hand-built word, which is what
    /// `a_slot_held_by_nobody_nameable_is_not_a_state_this_protocol_can_produce`
    /// constructs — precisely to show it is unreachable through the API.
    fn freeze_mid_transition(w: &StateArmWord, slot: usize, tag: u32) {
        let pid = w.claim_pid(slot).expect("in range");
        assert_ne!(pid, PID_NONE, "the slot must be held to be frozen");
        w.claims[slot]
            .owner
            .store(pack_owner(tag, pid), Ordering::Release);
    }

    // AN INTERRUPTED CLAIM IS RECLAIMED. A worker killed after winning the ownership
    // CAS and before publishing leaves a `Claiming` slot; the table is FINITE, so
    // leaving it is a permanent leak that ends in machine-wide checkpoint death with
    // both counters reading zero. All three dispositions in one body, because the
    // discrimination between them is the whole rule.
    #[test]
    fn an_interrupted_claim_with_a_dead_owner_is_reclaimed_and_the_others_are_not() {
        let w = word();
        let dead_claiming = w.claim(11, 500).expect("slot");
        let live_claiming = w.claim(22, 300).expect("slot");
        let dead_releasing = w.claim(33, 700).expect("slot");
        let published = w.claim(44, 900).expect("slot");
        freeze_mid_transition(&w, dead_claiming, SLOT_CLAIMING);
        freeze_mid_transition(&w, live_claiming, SLOT_CLAIMING);
        // Died the other side of its own lifecycle — mid-RELEASE. Reclaimable for the
        // same reason, which is the half a Claiming-only sweep silently leaks.
        freeze_mid_transition(&w, dead_releasing, SLOT_RELEASING);

        // Only pid 22 is alive; 44's published claim is therefore stale too.
        let sweep = w.sweep_stale_claims(|pid| pid == 22);
        assert_eq!(
            sweep,
            StaleClaimSweep {
                cleared: 1,         // the published-but-stale claim
                freed_bytes: 2_100, // 900 published + 500 claiming + 700 releasing
                live: 0,            // nothing published survived
                in_flight: 1,       // only the LIVE claimer's slot
                interrupted_reclaimed: 2,
            },
            "a slot whose owner died is reclaimed from EVERY held state and counted \
             apart from an ordinary stale claim; a LIVE claimer's slot is left alone"
        );
        assert_eq!(w.claim_state(dead_claiming), Some(SlotState::Free));
        assert_eq!(w.claim_state(dead_releasing), Some(SlotState::Free));
        assert_eq!(w.claim_state(published), Some(SlotState::Free));
        assert_eq!(w.claim_state(live_claiming), Some(SlotState::Claiming));
        assert!(!sweep.is_empty(), "the caller logs this LOUDLY");

        // THE BALANCE ASSERTION: reclaiming touches no arithmetic, so the derived
        // reservation is exactly what the surviving HELD slots say — here, the live
        // claimer's, which is still holding its 300.
        assert_eq!(w.reservation(), (1, 300));

        // A reclaimed slot is genuinely re-issuable; a held one is never handed out.
        let reissued = w.claim(55, 42).expect("the reclaimed slot is available");
        assert!(reissued == dead_claiming || reissued == dead_releasing || reissued == published);
        assert_eq!(
            w.reservation(),
            (2, 342),
            "and it contributes exactly its own, on top of the live claimer's"
        );
        for _ in 0..STATE_CLAIM_SLOTS {
            if let Some(i) = w.claim(99, 1) {
                assert_ne!(i, live_claiming, "a held slot is never handed out twice");
            }
        }
    }

    // THE CLAIM-SLOT LEAK: a
    // worker killed between winning the slot and naming itself would leave an ANONYMOUS
    // held slot — unreclaimable, because liveness cannot be tested on an owner with
    // no identity — and 32 of those kill checkpointing machine-wide, permanently,
    // with the reservation reading zero.
    //
    // The protocol gives that instant no width: the CAS that wins the slot IS the one
    // that names its owner. The hook fires at exactly that instant, so a protocol that
    // stores the pid afterwards is caught here rather than argued about.
    #[test]
    fn a_slot_is_attributable_from_the_instant_it_is_won() {
        let w = word();
        let mut at_the_instant: Option<(SlotState, i32, u32, u64)> = None;
        {
            let seen = &mut at_the_instant;
            let peer = &*w;
            // The slot index is not yet returned, so scan for the one that is held —
            // which is itself the assertion that a peer can FIND the holder.
            w.claim_with_owned_hook_for_test(4242, 700, || {
                let held = (0..STATE_CLAIM_SLOTS)
                    .filter_map(|i| peer.claim_view(i))
                    .find(|v| v.state != SlotState::Free);
                let (count, bytes) = peer.reservation();
                *seen = held.map(|v| (v.state, v.pid, count, bytes));
            })
            .expect("slot");
        }
        let (state, pid, count, bytes) = at_the_instant.expect("a peer must SEE the held slot");
        assert_eq!(
            (state, pid),
            (SlotState::Claiming, 4242),
            "at the instant the slot is WON it must already name its owner — an \
             anonymous held slot can never be liveness-tested, so it can never be \
             reclaimed, and the finite table becomes a ratchet"
        );
        // RACE_ACCOUNTING, same instant: a
        // peer walking the table here must already COUNT this claim. The COUNT is
        // exact from the winning CAS; the BYTES lag it by the one relaxed store a
        // claim cannot make before it has title to the slot (see `reservation`).
        assert_eq!(
            count, 1,
            "a peer that can SEE a held slot must COUNT it — under-counting lets it \
             fork against memory this process has already committed to"
        );
        assert_eq!(bytes, 0, "the byte store has not happened yet — documented");
    }

    // The other side of the same instant: once the claim's own body has run, the peer
    // sees the full reservation. Without this the assertion above is satisfied by an
    // implementation that never records the bytes at all.
    #[test]
    fn a_claim_in_flight_is_counted_and_its_bytes_land_one_store_later() {
        let w = word();
        let slot = w.claim(4242, 700).expect("slot");
        assert_eq!(w.reservation(), (1, 700));
        // …and it is STILL counted once frozen mid-transition, which is the state a
        // crashed worker leaves behind.
        freeze_mid_transition(&w, slot, SLOT_CLAIMING);
        assert_eq!(
            w.reservation(),
            (1, 700),
            "a held slot's memory is spoken for whatever its holder was doing"
        );
    }

    // The publish is an EXCHANGE, and this is the interleaving that makes it matter: a
    // sweep judges this claimant dead — a `kill(pid, 0)` race, or a recycled pid —
    // while it is standing inside its own claim body, and takes the slot. A blind
    // `store` would then re-publish a claim the table has already given away, leaving
    // two parties owning one slot and one reservation counted once.
    //
    // The `on_owned` hook is the seam: it is the only point at which a peer can act on
    // a slot this claim has won but not yet published.
    #[test]
    fn a_claim_whose_slot_was_reclaimed_under_it_does_not_publish_over_the_reclaim() {
        let w = word();
        let mut probe: Option<(usize, u32)> = None;
        let mut fired = false;
        let landed = {
            let seen = &mut probe;
            let peer = &*w;
            w.claim_with_owned_hook_for_test(4242, 700, || {
                // Once: the retry must be allowed to succeed, or the claim walks the
                // whole table losing every slot and the test measures nothing.
                if fired {
                    return;
                }
                fired = true;
                let held = (0..STATE_CLAIM_SLOTS)
                    .find(|&i| peer.claim_state(i) != Some(SlotState::Free))
                    .expect("the claim has won a slot by now");
                let n = peer.sweep_stale_claims(|_| false).interrupted_reclaimed;
                *seen = Some((held, n));
            })
            .expect("the claim must land on ANOTHER slot")
        };
        let (stolen, reclaimed) = probe.expect("the hook must have run");
        assert_eq!(
            reclaimed, 1,
            "PRECONDITION: the sweep really did take the slot out from under the claim"
        );
        assert_ne!(
            landed, stolen,
            "the claim must NOT still be holding the slot the sweep reclaimed — a \
             blind store re-publishes a claim the table has given away"
        );
        assert_eq!(
            w.claim_state(stolen),
            Some(SlotState::Free),
            "the reclaimed slot stays free and re-issuable"
        );
        assert_eq!(
            w.claim_view(landed)
                .map(|v| (v.state, v.pid, v.reserved_bytes)),
            Some((SlotState::Active, 4242, 700)),
            "and the claim really did land, whole, somewhere else"
        );
        assert_eq!(
            w.reservation(),
            (1, 700),
            "exactly ONE live claim — not the claim plus the ghost it overwrote"
        );
    }

    // THE RECLAIM/RELEASE RACE. Reclaiming a
    // `Releasing` slot — which is what closes the transient-state ratchet — lets a sweep run a
    // teardown tail CONCURRENTLY with an ordinary release that has already won the
    // slot. A releaser's blind `store(OWNER_FREE)` would then land AFTER the reclaimer
    // has freed it and a NEW claimant has taken it, erasing a live claim.
    //
    // It needs NO misjudgement: `release` takes a slot INDEX and checks no ownership,
    // and the shipping release shape is the PARENT releasing a forked child's claim
    // on reap — so a live recorder is routinely mid-tail on a claim whose stamped pid
    // is GENUINELY dead, and a truthful `kill(pid, 0)` reclaims it.
    //
    // The hook fires at the one instant that interleaving exists.
    #[test]
    fn a_release_tail_cannot_free_a_slot_a_new_claimant_has_taken() {
        const RECORDER: i32 = 99;
        const DEAD_WORKER: i32 = 11;
        const NEWCOMER: i32 = 22;

        let w = word();
        let slot = w.claim(DEAD_WORKER, 500).expect("slot");
        let mut reclaimed = 0u32;
        let mut fired = false;

        let released = {
            let seen = &mut reclaimed;
            let peer = &*w;
            // The live RECORDER releases the dead worker's claim, as its reap path
            // does. Mid-tail, a sweep with a TRUTHFUL predicate — pid 11 really is
            // dead — reclaims the slot, and an unrelated live worker takes it.
            w.release_with_tail_hook_for_test(slot, RECORDER, || {
                if fired {
                    return;
                }
                fired = true;
                *seen = peer
                    .sweep_stale_claims(|pid| pid != DEAD_WORKER)
                    .interrupted_reclaimed;
                if *seen > 0 {
                    assert_eq!(peer.claim(NEWCOMER, 900), Some(slot), "recycled");
                }
            })
        };

        // With the RELEASER stamped, the sweep sees a LIVE holder and leaves the slot
        // alone, so the interleaving never even arises — that is the first half of the guard.
        assert_eq!(
            reclaimed, 0,
            "a sweep must judge the party that is RUNNING the teardown, not the claim \
             it is tearing down — stamping the dead claimant makes an in-progress \
             release indistinguishable from an abandoned one"
        );
        assert_eq!(released, Some(500), "the recorder's release completed");
        assert_eq!(
            w.claim_state(slot),
            Some(SlotState::Free),
            "and left the slot free"
        );
        assert_eq!(w.reservation(), (0, 0));
    }

    // The SECOND half, isolated: even if a predicate calls a LIVE holder dead — the
    // recycled-pid residual, which the stamping rule cannot exclude — the losing tail
    // must ABORT rather than publish FREE over whoever holds the slot now.
    #[test]
    fn a_tail_that_lost_its_slot_mid_teardown_does_not_publish_over_the_newcomer() {
        const RECORDER: i32 = 99;
        const NEWCOMER: i32 = 22;

        let w = word();
        let slot = w.claim(11, 500).expect("slot");
        let mut reclaimed = 0u32;
        let mut fired = false;

        let released = {
            let seen = &mut reclaimed;
            let peer = &*w;
            w.release_with_tail_hook_for_test(slot, RECORDER, || {
                if fired {
                    return;
                }
                fired = true;
                // A LYING predicate: the recorder is alive and standing right here.
                *seen = peer.sweep_stale_claims(|_| false).interrupted_reclaimed;
                assert_eq!(*seen, 1, "PRECONDITION: the sweep took the slot");
                assert_eq!(peer.claim(NEWCOMER, 900), Some(slot), "recycled");
            })
        };

        assert_eq!(
            released, None,
            "the tail lost the slot, so it freed nothing and must say so"
        );
        assert_eq!(
            w.claim_view(slot)
                .map(|v| (v.state, v.pid, v.reserved_bytes)),
            Some((SlotState::Active, NEWCOMER, 900)),
            "the newcomer's LIVE claim must survive — a blind store here erases it, \
             and the slot then belongs to two parties at once"
        );
        assert_eq!(w.reservation(), (1, 900));
    }

    // A releaser is a holder like any other: if IT dies mid-tail, its slot must still
    // be reclaimable — otherwise stamping the releaser would merely move the
    // ratchet onto the release path.
    #[test]
    fn a_releaser_that_dies_mid_tail_leaves_a_slot_the_sweep_can_still_reclaim() {
        const RECORDER: i32 = 99;

        let w = word();
        let slot = w.claim(11, 500).expect("slot");
        // Freeze the slot exactly as a releaser killed inside its tail leaves it.
        w.claims[slot]
            .owner
            .store(pack_owner(SLOT_RELEASING, RECORDER), Ordering::Release);
        assert_eq!(w.claim_pid(slot), Some(RECORDER), "named by its RELEASER");

        // While the recorder lives, nobody may touch it.
        let alive = w.sweep_stale_claims(|pid| pid == RECORDER);
        assert_eq!(
            (alive.in_flight, alive.interrupted_reclaimed),
            (1, 0),
            "a live releaser's slot is reported, never taken"
        );

        // Once it is gone, the slot comes back.
        let dead = w.sweep_stale_claims(|_| false);
        assert_eq!(dead.interrupted_reclaimed, 1);
        assert_eq!(w.claim_state(slot), Some(SlotState::Free));
        assert_eq!(w.claim(55, 42), Some(slot), "and is re-issuable");
    }

    // A release must be ATTRIBUTABLE for the same reason a claim must — the sweep
    // tests the RELEASER's liveness, so a pid it cannot test strands the slot.
    #[test]
    fn a_release_refuses_a_pid_that_cannot_be_liveness_tested() {
        for bad in [0, -1, i32::MIN] {
            let w = word();
            let slot = w.claim(11, 500).expect("slot");
            assert_eq!(
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| w.release(slot, bad)))
                    .unwrap_or(None),
                None,
                "pid {bad} must be refused"
            );
            assert_eq!(
                w.claim_view(slot).map(|v| (v.state, v.pid)),
                Some((SlotState::Active, 11)),
                "pid {bad}: a refused release changes nothing"
            );
            assert_eq!(w.reservation(), (1, 500), "pid {bad}");
        }
    }

    // A claim must be ATTRIBUTABLE, and a non-positive pid is not: `kill(0, 0)`
    // signals the caller's OWN process group and `kill(-N, 0)` a group, so either
    // reports a dead claimant ALIVE and its slot is never swept — the stale-claim sweep defeated
    // by an argument value rather than by a missing rule. 0 is additionally this
    // table's no-owner marker.
    #[test]
    fn a_claim_refuses_a_pid_that_cannot_be_liveness_tested() {
        for bad in [0, -1, -4242, i32::MIN] {
            let w = word();
            assert_eq!(
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| w.claim(bad, 500)))
                    .unwrap_or(None),
                None,
                "pid {bad} must be refused"
            );
            assert_eq!(
                w.reservation(),
                (0, 0),
                "pid {bad}: a refused claim reserves nothing"
            );
            for i in 0..STATE_CLAIM_SLOTS {
                assert_eq!(
                    w.claim_state(i),
                    Some(SlotState::Free),
                    "pid {bad}: a refused claim takes no slot"
                );
            }
        }
    }

    // Held-but-anonymous is not reachable through the API (the two tests above), so
    // it can only arrive as CORRUPTION or from a future participant this build does
    // not know. It must then be REPORTED, never silently handed to a second claimer
    // and never mistaken for a reclaimable slot — a sweep that "reclaimed" it would be
    // guessing that nobody is inside it.
    #[test]
    fn a_slot_held_by_nobody_nameable_is_not_a_state_this_protocol_can_produce() {
        for tag in [SLOT_CLAIMING, SLOT_ACTIVE, SLOT_RELEASING] {
            let w = word();
            w.claims[0]
                .owner
                .store(pack_owner(tag, PID_NONE), Ordering::Release);
            let sweep = w.sweep_stale_claims(|_| false);
            assert_eq!(
                (sweep.in_flight, sweep.cleared, sweep.interrupted_reclaimed),
                (1, 0, 0),
                "tag {tag}: an unnameable holder is reported, not judged"
            );
            assert_eq!(
                w.claim_state(0).map(|s| s.as_wire()),
                Some(tag),
                "tag {tag}: and it is left exactly as found"
            );
        }
    }

    // A sweep judges a slot, then the world moves under it. The `is_alive` predicate
    // is the seam: the sweep calls it BETWEEN loading the ownership word and
    // exchanging on it, which is the only place this interleaving exists.
    //
    // With a tag-only exchange the sweep succeeds against the NEWCOMER's word and
    // tears down a live claim (classic ABA on a recycled slot). Exchanging the whole
    // word makes the identity part of the compare, so it fails instead.
    #[test]
    fn a_sweep_cannot_tear_down_a_claim_that_took_the_slot_while_it_was_judging() {
        let w = word();
        let slot = w.claim(11, 500).expect("slot");
        let sweep = w.sweep_stale_claims(|pid| {
            if pid == 11 {
                // The owner finishes and an UNRELATED live worker takes the same slot,
                // all while the sweep is deciding that pid 11 is dead.
                assert_eq!(w.release(slot, 11), Some(500));
                assert_eq!(w.claim(22, 900), Some(slot), "the newcomer recycles it");
            }
            false
        });
        assert_eq!(
            sweep.cleared, 0,
            "the sweep judged pid 11 and must not act on that judgement against pid \
             22's claim — the slot it observed is gone"
        );
        assert_eq!(sweep.freed_bytes, 0);
        assert_eq!(
            w.claim_view(slot)
                .map(|v| (v.state, v.pid, v.reserved_bytes)),
            Some((SlotState::Active, 22, 900)),
            "the newcomer's live claim must survive intact"
        );
        assert_eq!(w.reservation(), (1, 900));
    }

    // THE EXHAUSTION ARM — a permanent regression pin.
    // Burn EVERY slot with an interrupted claim whose owner is dead, and require the
    // table to come all the way back. Without the reclaim this ends with `claim`
    // returning `None` for the lifetime of the mapping. Driven from BOTH transient
    // states, because a sweep that handles only one leaks the other just as finally.
    #[test]
    fn a_table_burned_out_by_interrupted_claims_is_fully_reclaimed_by_the_sweep() {
        for frozen in [SLOT_CLAIMING, SLOT_RELEASING] {
            let w = word();
            for i in 0..STATE_CLAIM_SLOTS {
                let slot = w.claim(1000 + i as i32, 64).expect("slot");
                freeze_mid_transition(&w, slot, frozen);
            }
            assert_eq!(
                w.claim(9999, 1),
                None,
                "state {frozen}: PRECONDITION — the table really is exhausted"
            );
            assert_eq!(
                w.reservation(),
                (STATE_CLAIM_SLOTS as u32, 64 * STATE_CLAIM_SLOTS as u64),
                "state {frozen}: and the exhaustion is VISIBLE — every one of those \
                 slots is still holding memory, which is what a peer's fork gate must \
                 see (counting published claims only reports an empty table here, so \
                 the leak is silent as well as terminal)"
            );

            let sweep = w.sweep_stale_claims(|_| false);
            assert_eq!(
                sweep.interrupted_reclaimed, STATE_CLAIM_SLOTS as u32,
                "state {frozen}"
            );
            assert_eq!(
                sweep.cleared, 0,
                "state {frozen}: none of them were ever published"
            );
            assert_eq!(sweep.in_flight, 0, "state {frozen}");
            assert_eq!(
                sweep.freed_bytes,
                64 * STATE_CLAIM_SLOTS as u64,
                "state {frozen}: every byte they were holding comes back"
            );

            // Every slot is usable again, and the reservation still balances.
            for i in 0..STATE_CLAIM_SLOTS {
                assert!(
                    w.claim(2000 + i as i32, 8).is_some(),
                    "state {frozen}: slot {i} must be claimable after the reclaim"
                );
            }
            assert_eq!(w.reservation(), (STATE_CLAIM_SLOTS as u32, 8 * 32));
        }
    }

    #[test]
    fn slot_state_wire_round_trips_and_preserves_an_unknown_discriminant() {
        for s in [
            SlotState::Free,
            SlotState::Claiming,
            SlotState::Active,
            SlotState::Releasing,
        ] {
            assert_eq!(SlotState::from_wire(s.as_wire()), s);
        }
        assert_eq!(SlotState::from_wire(77), SlotState::Unrecognized(77));
        assert_eq!(SlotState::Unrecognized(77).as_wire(), 77);
        assert!(!SlotState::Free.is_in_flight());
        assert!(!SlotState::Active.is_in_flight());
        assert!(SlotState::Claiming.is_in_flight());
        assert!(SlotState::Releasing.is_in_flight());
        // A fresh (zero-filled) slot must decode as FREE — an O_EXCL create relies on
        // it, and an `Unrecognized(0)` would make every new word unusable.
        assert_eq!(SlotState::from_wire(0), SlotState::Free);
    }

    // Belt and braces over the deterministic hook arms: REAL concurrency, asserted
    // only at quiescence so it can never flake. It cannot place an interleaving (the
    // hook arms do that), but it exercises orderings no single-threaded test reaches,
    // and a protocol that double-subtracted or lost an update leaves a nonzero — or
    // catastrophically underflowed — counter behind.
    #[test]
    fn concurrent_claim_release_and_sweep_leave_the_counters_exactly_empty() {
        const THREADS: usize = 6;
        const ROUNDS: usize = 400;
        let w = word();
        let claimed = std::sync::atomic::AtomicU64::new(0);

        std::thread::scope(|s| {
            for t in 0..THREADS {
                let w = &*w;
                let claimed = &claimed;
                s.spawn(move || {
                    for r in 0..ROUNDS {
                        if let Some(slot) = w.claim(1000 + t as i32, (r as u64 % 7) + 1) {
                            claimed.fetch_add(1, Ordering::Relaxed);
                            std::hint::spin_loop();
                            w.release(slot, 1000 + t as i32);
                        }
                    }
                });
            }
            // A sweeper that believes EVERY claimant is dead — maximally hostile, so
            // it contends with every release above.
            let w_sweep = &*w;
            s.spawn(move || {
                for _ in 0..ROUNDS {
                    w_sweep.sweep_stale_claims(|_| false);
                    std::hint::spin_loop();
                }
            });
        });

        assert!(
            claimed.load(Ordering::Relaxed) > 0,
            "ANTI-VACUITY: the run must really have claimed something"
        );
        let (matches, live, bytes) = counters_match_table(&w);
        assert!(
            matches,
            "after quiescence the counters ({}, {}) must equal the table ({live}, \
             {bytes})",
            w.busy_workers(),
            w.rss_reserved(),
        );
        assert_eq!(w.busy_workers(), 0, "every claim was released or swept");
        assert_eq!(w.rss_reserved(), 0);
    }

    // ---- names ----

    #[test]
    fn the_object_name_is_deterministic_prefix_free_and_fits_macos() {
        let a = state_arm_shm_name("run-a");
        assert_eq!(a, state_arm_shm_name("run-a"), "deterministic");
        assert_ne!(a, state_arm_shm_name("run-b"));
        assert_eq!(a.len(), 25, "/cer_sta_ + 16 hex ≤ PSHMNAMLEN (31)");
        assert!(a.starts_with("/cer_sta_"));
        // Prefix-free against the other SHM families in this crate.
        assert!(!a.starts_with("/cer_rg_"));
        assert!(!a.starts_with("/cer_bar_"));
        assert!(!a.starts_with("/cer_db_"));
    }
}

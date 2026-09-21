// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process **block-credit word** — one SHM word per
//! cross-process `block` edge, so a `#[input(backpressure = block)]` edge whose
//! producer and consumer land in DIFFERENT process groups is REPRESENTABLE at
//! all.
//!
//! # What it is
//!
//! Key `(topic, consumer_node, consumer_input)` — the same identity as the
//! in-process mirror maps (`sub_block_by_input` in `graph/runtime.rs`).
//! Same-process edges keep their existing `Arc<AtomicU64>` (zero change, zero
//! syscalls); only a SPLIT edge needs a word both address spaces can see.
//!
//! Semantics are unchanged from `BlockDeferEdge`: the producer defers ENTRY to a
//! tick while `outstanding >= depth`, and the consumer's drain frees room.
//!
//! # A capability enabler, not a fix for a shipping silent hole
//!
//! Without this word a single-producer cross-process block edge cannot RUN: `subgraph_for`
//! drops the foreign producer from the consumer's subgraph, the consumer's
//! worker dies LOUDLY at `GraphTopology::validate`'s producer-less-block arm on
//! every build path, and the supervisor fail-fasts before GO. Nothing is
//! silently lossy — the edge simply does not exist. What this word buys is the
//! flagless multi-process default (which derives process-per-node), under
//! which NO block-bearing graph could otherwise run multi-process at all.
//!
//! The one genuinely silent shape — a `multi_publisher_topics` block topic split
//! with a producer CO-LOCATED in the consumer's group, where validate passes and
//! the remote producer never defers — is the multi-producer residual this design
//! deliberately fences OUT (see [the scope fence](#the-scope-fence)); it
//! stays closed by the narrowed `validate_block_colocation` refusal, not by
//! credit.
//!
//! # Determinism firewall (NON-NEGOTIABLE)
//!
//! The credit word is a **WHEN gate on tick ENTRY** — it changes only WHEN a
//! producer is permitted to fire, NEVER WHAT fires or with what payload. It is
//! never consumed as data and never feeds a node body. Replay stays identical to
//! live (Principle #7) for exactly the reason [`crate::barrier`] gives: the gate
//! synchronizes a boundary and leaves the fire set + data flow untouched. Same
//! firewall as [`crate::doorbell`], [`crate::monitor_wait`] and
//! [`crate::barrier`].
//!
//! # Why this is NOT a barrier
//!
//! [`crate::barrier`]'s `BarrierShared` is a RENDEZVOUS: N participants meet, the
//! last one through opens a generation, and the whole design (sense reversal, the
//! count-down re-arm, the one-generation max-skew invariant) exists to make that
//! meeting correct. A credit word has no meeting — a producer asks "is there
//! room?" and a consumer says "there is more room now". So there is no
//! generation, no sense, no re-arm and no skew invariant to uphold: strictly
//! simpler, and deliberately so. What IS transplanted from the barrier is the
//! part that was hard to get right — the `MAP_SHARED` page, the FNV name
//! derivation, the `parked`-gated kernel wake with its store-buffer fences, and
//! the RAII bit guard.
//!
//! # SHM-layout compatibility
//!
//! [`CreditShared`] is `#[repr(C)]` and contains ONLY atomics, so it can be
//! placed directly in an mmap'd `MAP_SHARED` page and operated on by two
//! processes at the same physical address. Every operation is lock-free (atomic
//! load / `fetch_add` / `fetch_update` / store), so a crashed peer can never
//! leave the word holding a lock.
//!
//! Layout is evolvable later on the barrier's own three-clause argument
//! (`barrier.rs`'s layout-bump note): a deployment's supervisor + workers are the
//! SAME binary, segment names are per-run scoped, and bags carry no credit
//! structs, so there is no window in which an old-layout process maps a
//! new-layout segment.
//!
//! # The scope fence
//!
//! A **multi-producer** cross-process credit edge is unserializable at decide
//! time (the fused-decide argument cannot cross processes); the sound fix
//! is CAS reservation (claim-iff-below-depth, publish-or-release). This module does
//! NOT build it — multi-producer block flows keep the co-location seed as a hard
//! derivation constraint, and single-producer cross-process edges (the
//! overwhelmingly common shape) get this word. [`CreditShared::try_claim`] is
//! sound as a CLAIM only under that fence; see its docs.
//!
//! # Scope of this module
//!
//! The primitive ONLY. The runtime, the scheduler and the plan
//! reference it from outside (the `credit_edges` plan field,
//! the mapped `BlockDeferEdge` variant, the producer-side wake predicate and the
//! `validate_block_colocation` narrowing). `pub` because the multi-process
//! supervisor in `cerulion_cli_engine` creates and opens the words — the same
//! standing this crate gives [`crate::barrier`], [`crate::doorbell`] and
//! [`crate::monitor_wait`].
//!
//! # Residuals (named, not implied)
//!
//! **The LOST-DECREMENT wedge.** Nothing in this module returns credit on behalf of
//! a consumer that DIED. `outstanding` is decremented only by
//! [`CreditShared::record_drained`], which the consumer itself calls, so a
//! consumer that is SIGKILLed while `outstanding >= depth` pins the word FULL and
//! its producer defers FOREVER — bounded-loud on the producer's side only in the
//! sense that its own liveness is unaffected; the edge is simply dead. This is a
//! deliberate consequence of the [wedge-and-warn decision](self): there
//! is no auto-release path.
//!
//! The two REMEDIES that decision names live outside this module: (a) the supervisor's
//! per-edge DEATH WARN — the supervisor already sweeps a dead
//! rank's stale `parked` bit via [`CreditShared::clear_parked_producer`], and the
//! same sweep is where a dead CONSUMER's edge is named; and (b) a bounded
//! producer-side loud warn after N × `cap` of observing the word FULL with
//! `wake_seq` UNMOVED (full-and-nobody-is-draining is exactly the wedge
//! signature, and `wake_seq` not moving is what separates it from a merely busy
//! consumer). Note also that [`CreditShared::record_drained`]`(u64::MAX)` already
//! functions as a FORCE-RELEASE through any live mapping (the decrement
//! saturates at zero), so a supervisor holding a mapping has the primitive it
//! needs without new API.
//!
//! **The macOS park does NOT ride the BARRIER's kill switch.** The wait side's
//! `os_sync` tier reads the shared backend FACT from
//! `crate::os_sync::os_sync_backend_usable` — resolved, and not latched off by
//! an unrecoverable errno (one primitive family, one unusable-latch, shared
//! with the barrier and with the monitor-wait park nap) — and then
//! applies its OWN switch, [`CREDIT_OS_SYNC_ENV`].
//!
//! It deliberately does NOT ask `crate::barrier::os_sync_active`, which ANDs in
//! `CERULION_BARRIER_OS_SYNC`: if either of this module's macOS call sites —
//! the availability predicate or `park_wait_credit` itself — did, setting the
//! BARRIER's switch to `0` would silently disable the credit tier, while
//! `docs/user-api.md` promises the two are independent. That is why the
//! FACT/DECISION split is structural: the fact lives in `crate::os_sync`, and
//! each consumer owns its decision. Pinned by
//! `credit_os_sync_independence_test::prc_the_credit_os_sync_tier_ignores_the_barrier_kill_switch`,
//! which drives both directions in subprocesses and FAILS on coupled code.
//!
//! The `=0`/`=1`/garbage GRAMMAR is still the one shared
//! `crate::kill_switch::parse_kill_switch`, so the switches cannot drift in what
//! they accept.
//!
//! **The create/open/map/unlink sequence is SHARED.** It lives ONCE in
//! `crate::shm_map`, and this module, `barrier.rs`, `shm_ring.rs`,
//! `state_arm.rs` and `wedge_page` all read through it. One behaviour worth
//! knowing here: the `mmap` errno is captured BEFORE
//! the `close` — see `shm_map`'s own module docs.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::Duration;

// The Apple `os_sync_*` FFI scalar used by the macOS wake tier. Imported only
// where that tier compiles so no unused-import fires elsewhere.
#[cfg(target_os = "macos")]
use core::ffi::c_void;

/// The mapped region size — 64 bytes, ONE cache line.
///
/// Only the leading [`core::mem::size_of::<CreditShared>()`](CreditShared) bytes
/// hold atomics; the rest pads to a full line so a credit word never false-shares
/// with an adjacent object. Same rationale (and same number) as the barrier page
/// and the doorbell.
pub const CREDIT_BYTES: usize = 64;

/// The lock-free, `#[repr(C)]`, SHM-mappable block-credit word.
///
/// See the [module docs](self) for the purpose, the determinism firewall and the
/// scope fence. All fields are private; the public methods are the entire
/// contract.
#[derive(Debug)]
#[repr(C)]
pub struct CreditShared {
    /// Frames PUBLISHED minus frames DRAINED — the shared mirror of the
    /// consumer's iceoryx2 queue depth. Bumped (`Release`) by the producer on
    /// every successful send, decremented (saturating) by the consumer on every
    /// drain.
    outstanding: AtomicU64,
    /// The declared `#[input(depth = N)]` ceiling, stamped by the OWNER at
    /// create time.
    ///
    /// It lives IN the word (rather than being handed to each side separately)
    /// so the producer's defer-read and the consumer's ceiling cannot disagree
    /// under plan skew — the one-recipe discipline. A stamped depth ABOVE the
    /// consumer's real declaration would make the producer defer LATE and the
    /// consumer's behaviourally-real queue evict on a declared-LOSSLESS edge
    /// (Principle #6), which is why the plan stamps it from the LOADED planning
    /// topology and never from source-parsed metadata or a default.
    depth: AtomicU32,
    /// Credit-activity EPOCH — the producer's park compare word.
    ///
    /// Bumped (`Release`) whenever credit is freed and kernel-woken so a
    /// producer parked in the live loop observes the consumer's drain in µs
    /// instead of at the ~100µs sleep-recheck cadence. NEVER read as data, never
    /// gates the claim/publish/drain logic — compared only by a parker's kernel
    /// block ([`Self::park_wait_credit`]), with the caller's own
    /// [`Self::try_claim`] re-derive staying the sole correctness (Principle #7
    /// record-only firewall). `u32` wrap-around: a false-equal needs EXACTLY
    /// 2^32 bumps inside one snapshot→block window (ns–µs), and the block is
    /// slice-bounded regardless — the same argument the barrier's own wake word
    /// makes.
    wake_seq: AtomicU32,
    /// PER-EDGE parker bitmask: bit `i` = the producer at index `i` of THIS
    /// EDGE's `producer_ranks` holds a [`ParkedEdgeGuard`]. The scope
    /// fence means exactly one producer, so bit 0; the width pre-fits the CAS
    /// extension to 32 producers PER EDGE.
    ///
    /// Deliberately NOT the barrier's GLOBAL-RANK indexing. The donor's 32-rank
    /// ceiling was acceptable under lockstep (a bit-less rank lost only wake
    /// latency over a working barrier), but under free-run the wake word IS the
    /// latency plane and the flagless default is process-per-node — the
    /// `humanoid_mp` shape is 36 nodes, so ranks 32-35 would be bit-less exactly
    /// where the win matters. Per-edge indexing has no ceiling at any rank
    /// count, because an EDGE has few producers however many ranks the
    /// deployment has.
    ///
    /// A bitmask (not a count) so a dead peer's stale bit is EXACTLY sweepable
    /// by the supervisor ([`Self::clear_parked_producer`]) without perturbing
    /// live parkers.
    parked: AtomicU32,
    /// RESERVED: the owner RE-ARM epoch, bumped by `reinit` and by nothing else.
    ///
    /// # What it does NOT do today, stated because the obvious reading is wrong
    ///
    /// `reinit` is private and its ONLY caller is
    /// [`MappedCredit::create_owned`], which UNLINKS the name and `O_EXCL`-creates
    /// a FRESH object — so a "re-arm" is always a NEW PAGE, never an in-place
    /// re-init of a page somebody else holds. Two consequences, both pinned by
    /// tests rather than left to be rediscovered:
    ///
    /// - a peer's HELD mapping NEVER observes a bump (it is mapped to the OLD,
    ///   now-unnamed object, which nothing touches again), and
    /// - a peer that RE-OPENS by name always reads `1` (the fresh page was
    ///   zero-filled by POSIX, and `reinit` bumped it once).
    ///
    /// So a STALE mapping is NOT detectable from this field, on either path.
    /// **The dead-consumer story is the supervisor's per-edge death warn,
    /// NOT an epoch poll** — see the [module residuals](self).
    ///
    /// # Why it is kept
    ///
    /// It is a RESERVED seam for a future IN-PLACE re-arm (a `reinit` called on a
    /// segment peers already hold), which is the only shape that would make the
    /// value observable to anyone but its own creator. It costs 4 bytes inside a
    /// 64-byte cache line that is padding anyway, and it is the ONE observable
    /// that distinguishes a word `reinit` has armed from a bare
    /// [`CreditShared::new`] (`0`) — which is what the create-path tests read.
    ///
    /// **That last reading now has a PRODUCTION consumer, and it does not spend
    /// the reservation:** `reject_unarmed` refuses an open whose epoch is `0`,
    /// because a creator that died between `ftruncate` and `reinit` leaves a
    /// full-size zero-filled page the size check accepts and whose `depth == 0`
    /// wedges the peer's producer forever. Armed-vs-bare is a statement about
    /// whether ANY owner ever ran `reinit`; the reservation is about what a
    /// SUBSEQUENT bump would mean to a peer already mapped. The two are
    /// independent, so opening the in-place seam does not invalidate the check
    /// (an in-place re-arm only ever raises the epoch further above `0`).
    ///
    /// If that seam is ever opened, `reinit`'s no-wake and parked-mask-zeroing
    /// decisions MUST be revisited: both rest on the unlink-first invariant that
    /// an in-place re-arm would break. See the note in `reinit`.
    epoch: AtomicU32,
}

// SHM mapping across processes requires a stable, atomics-only `repr(C)` layout.
// Lock the size so a future non-atomic field (padding / bool / pointer) fails the
// build rather than silently breaking the cross-process mapping.
const _: () = assert!(
    core::mem::size_of::<CreditShared>() == 24,
    "CreditShared layout changed — SHM cross-process mapping requires the 24-byte atomics-only repr(C)"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, outstanding) == 0,
    "outstanding must sit at offset 0"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, depth) == 8,
    "depth must sit at offset 8"
);
// Pin the wake-word + parked-mask offsets so the park's kernel wait/wake address
// recipe is compile-locked (the same discipline as the barrier's).
const _: () = assert!(
    core::mem::offset_of!(CreditShared, wake_seq) == 12,
    "wake_seq must sit at offset 12 — the park wake word's kernel wait/wake address recipe"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, parked) == 16,
    "parked must sit at offset 16 — the per-edge producer park bitmask"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, epoch) == 20,
    "epoch must sit at offset 20 — the owner re-arm epoch"
);
const _: () = assert!(
    core::mem::align_of::<CreditShared>() >= 8,
    "CreditShared must be >=8-byte aligned so its `outstanding` field (offset 0) is a valid 8-byte-aligned atomic target"
);
const _: () = assert!(
    core::mem::size_of::<CreditShared>() <= CREDIT_BYTES,
    "CreditShared must fit the region MappedCredit maps"
);

impl CreditShared {
    /// Create a credit word for a consumer input whose declared depth is
    /// `depth`, with zero frames outstanding.
    ///
    /// `const` so a [`CreditShared`] can be placed in a `static` /
    /// const-initialized region.
    pub const fn new(depth: u32) -> Self {
        Self {
            outstanding: AtomicU64::new(0),
            depth: AtomicU32::new(depth),
            wake_seq: AtomicU32::new(0),
            parked: AtomicU32::new(0),
            epoch: AtomicU32::new(0),
        }
    }

    /// Re-initialize a freshly-mapped segment: zero the outstanding count, the
    /// wake epoch and the parked mask, stamp `depth`, and BUMP the re-arm epoch.
    ///
    /// Used by [`MappedCredit::create_owned`] after an unlink-then-`O_EXCL`
    /// create. POSIX zero-initializes a new SHM object, so every store but the
    /// depth stamp and the epoch bump is redundant on a fresh object — they are
    /// kept as a defensive, intent-explicit reset so this is also a correct FULL
    /// re-init if ever called on a non-fresh segment (the barrier's
    /// orphan-survivor case).
    ///
    /// The parked mask matters most of the three: a stale bit from a crashed
    /// prior run would tax every drain with a no-op wake syscall forever. An
    /// orphaned prior run's parkers sit on a DIFFERENT physical page
    /// (unlink-first + fresh `O_EXCL` create), unreachable by this segment's
    /// wake word by construction — so, exactly like the barrier, NO wake is
    /// issued here.
    ///
    /// **That invariant holds ONLY while `reinit` stays CREATE-ONLY.** Both
    /// decisions above — issuing no wake, and zeroing `parked` outright — rest
    /// on "no live peer is mapped to this page". If the reserved in-place re-arm
    /// seam is ever opened (see the `epoch` field docs), a peer PARKED on this
    /// very page would be zeroed out of the mask (so the next drain would not
    /// wake it) and never signalled here — a lost wakeup, Principle #6. Both
    /// must be revisited at that point, not merely re-read.
    ///
    /// The epoch is the one field that is BUMPED rather than zeroed. Under the
    /// create-only rule that bump is observable only to this segment's own
    /// creator (`0` = a bare `new`, `1` = armed); it is RESERVED for the in-place
    /// seam above, which is the only shape that would make it mean "somebody
    /// re-armed the edge under me".
    fn reinit(&self, depth: u32) {
        debug_assert!(
            depth >= 1,
            "a credit word stamped depth 0 is permanently full — its producer defers forever. \
             Graph validation enforces depth >= 1 (GraphTopology::validate); a 0 here is a \
             plan-stamping bug, not an operator input."
        );
        self.outstanding.store(0, Ordering::Release);
        self.depth.store(depth, Ordering::Release);
        self.wake_seq.store(0, Ordering::Release);
        self.parked.store(0, Ordering::Release);
        self.epoch.fetch_add(1, Ordering::Release);
    }

    /// `true` once the consumer's queue is at its declared depth — the producer
    /// must DEFER entry to its tick (`Acquire` load, so the producer observes
    /// every drain that happens-before).
    ///
    /// This is the mapped form of `BlockDeferEdge`'s
    /// `outstanding.load(Acquire) >= threshold`, unchanged in shape.
    ///
    /// A word stamped `depth == 0` reads full unconditionally; see the
    /// `reinit` note on why that is a plan-stamping bug rather than a
    /// reachable configuration.
    pub fn is_full(&self) -> bool {
        self.outstanding.load(Ordering::Acquire) >= u64::from(self.depth.load(Ordering::Acquire))
    }

    /// Try to CLAIM room for one frame — `true` if the producer may enter its
    /// tick, `false` if it must defer.
    ///
    /// # Why a plain read IS a claim here, and when it stops being one
    ///
    /// This is `!`[`is_full`](Self::is_full) and nothing more: it reserves
    /// nothing. That is sound as a CLAIM **only** under the scope fence
    /// (see the [module docs](self)) — with exactly ONE producer on the edge,
    /// no other claimer can consume the room between this read and the
    /// [`record_published`](Self::record_published) that follows it, so a
    /// successful read really does mean the slot is this producer's.
    ///
    /// The multi-producer extension replaces this BODY with the CAS reservation
    /// the fence defers (claim-iff-below-depth, publish-or-release) and leaves
    /// every call site untouched — which is why the name is `try_claim` rather
    /// than a read-flavoured one: the eventual semantics are the shipped
    /// semantics, narrowed by a fence rather than by a different operation.
    ///
    /// It carries the same **one-publish-per-tick assumption** `BlockDeferEdge`
    /// documents: the gate governs tick ENTRY, not individual publishes within
    /// a tick, so a node publishing more than once per tick can push the word
    /// past `depth` inside a single un-deferred tick.
    pub fn try_claim(&self) -> bool {
        !self.is_full()
    }

    /// Record that one frame was published to this edge's topic — the consumer's
    /// queue just gained a sample. Returns the `outstanding` count THIS publish
    /// created.
    ///
    /// `Release` so the producer's own subsequent `Acquire` defer-read, and the
    /// consumer's, see a consistent count. Deliberately issues NO wake: nothing
    /// ever parks waiting for the count to RISE.
    ///
    /// # Why the count is RETURNED rather than left to a later read
    ///
    /// The RMW's own result is the only race-free reading of what this publish
    /// did. A separate [`outstanding`](Self::outstanding) call afterwards is a
    /// DIFFERENT instant, and the consumer is free to drain in between — so an
    /// over-depth count created here can be gone before anybody looks, which
    /// makes a later read useless as a ceiling check (the whole point of the
    /// [`try_claim`](Self::try_claim) gate). `fetch_add` returns the PREVIOUS
    /// value and the transition is atomic, so `prev + 1` is exactly the count
    /// that existed at the instant of the publish, whatever happens next.
    ///
    /// `wrapping_add` mirrors the store: `fetch_add` wraps, so at the (utterly
    /// unreachable) `u64::MAX` boundary the reported value still equals what the
    /// word now holds rather than panicking in a debug build.
    pub fn record_published(&self) -> u64 {
        self.record_published_n(1)
    }

    /// Record that `n` frames were published to (or RETURNED to) this edge's
    /// queue — the `n`-at-a-time form of [`record_published`](Self::record_published),
    /// which delegates here so there is ONE increment implementation.
    ///
    /// Returns the `outstanding` count this call created, for the same
    /// race-free reason [`record_published`](Self::record_published) gives: the
    /// RMW's own result is the only reading that describes the instant of the
    /// increment, and a later load is a different instant.
    ///
    /// # Why an `n` form exists at all
    ///
    /// The publish path really does move one frame at a time, so `n > 1` is
    /// never a batched publish. It is the held-head SLOT DEBT
    /// re-derivation (`transport::subscriber`'s `reconcile_block_slot_debt`),
    /// which restates occupancy from the slots rather than applying a `+1`/`-1`
    /// per site: a frame that moved from the queue into a matcher slot was
    /// already decremented at its pop, so the mirror is CREDITED BACK.
    ///
    /// `held - debt` is 0, 1 or 2 by ARITHMETIC — `held` is two booleans summed
    /// — but 2 is not reachable through any public seam: every transition that
    /// fills a slot re-derives the debt before another can fill, so the delta
    /// observed in practice is always 1. Recorded because it is the difference
    /// between this being an `n` form and being a second spelling of
    /// [`record_published`](Self::record_published), and because hard-coding
    /// this argument to `1` (the change that would prove otherwise) makes no
    /// observable difference: nothing can distinguish the two.
    /// Without this form that site would reach past the word's own API into the
    /// atomic, which is exactly the second spelling of the arithmetic this type
    /// exists to prevent.
    ///
    /// `n == 0` is still an RMW returning the current value — the callers
    /// short-circuit before reaching here, and a zero increment is a correct
    /// no-op rather than a special case.
    ///
    /// Deliberately issues NO wake: nothing ever parks waiting for the count to
    /// RISE (the release direction is [`record_drained`](Self::record_drained),
    /// which does ring).
    pub fn record_published_n(&self, n: u64) -> u64 {
        self.outstanding
            .fetch_add(n, Ordering::Release)
            .wrapping_add(n)
    }

    /// Record that `removed` samples were drained from the consumer's queue —
    /// decrement the mirror and FREE the credit (bumping the wake epoch and
    /// waking any parked producer).
    ///
    /// Saturating, and that matters MORE cross-process than it did in-process:
    /// an interleaving non-drain removal (a late joiner's history replay) could
    /// otherwise drive the mirror below zero, and the far side has no way to
    /// notice. Clamping keeps `outstanding == 0` the floor.
    ///
    /// `removed == 0` is a NO-OP — no decrement, no epoch bump, no wake — so an
    /// idle drain pass never taxes a parked producer with a spurious wake.
    /// Under k-fires-per-step this is called PER POPPED FRAME by the post-FIFO
    /// drain, which is why the epoch bump has to be cheap when nobody parks.
    pub fn record_drained(&self, removed: u64) {
        if removed == 0 {
            return;
        }
        // The result is INFALLIBLE by construction: `fetch_update` returns `Err`
        // only when the closure returns `None`, and this closure always returns
        // `Some`. So the discard drops a `Result` that cannot be `Err`, never a
        // failure signal.
        let _ = self
            .outstanding
            .fetch_update(Ordering::Release, Ordering::Acquire, |v| {
                Some(v.saturating_sub(removed))
            });
        self.note_credit_freed();
    }

    // ==================== the producer park's wake word ====================
    // A kernel wake for a credit-blocked producer: parkers block on the
    // `wake_seq` epoch; freeing credit bumps it (the consumer gates only the
    // SYSCALL on the `parked` mask, so a drain stays atomics-only when nobody
    // parks). Record-only WAIT machinery (Principle #7): changes WHEN the park
    // returns, never what fires — the caller's `try_claim` re-derive after every
    // wake stays the sole correctness.

    /// The CONSUMER side of the wake word — unconditional epoch bump, then a
    /// `parked`-GATED kernel wake.
    ///
    /// Called by [`record_drained`](Self::record_drained) whenever it really
    /// freed credit; `pub` so a caller outside this module (a dead-consumer
    /// release path, say) can ring the word without pretending to drain. SAFE to
    /// over-signal: a wake with no state change is a bounded no-op re-park, by
    /// [`park_wait_credit`](Self::park_wait_credit)'s contract.
    ///
    /// # Store-buffer litmus (why the `SeqCst` fence)
    ///
    /// This pairs with [`park_enter`](Self::park_enter): parker = {set `parked`
    /// bit; load predicate}, waker = {`outstanding` RMW + `wake_seq` bump; load
    /// `parked`} — the classic SB shape, and Acquire/Release alone permits the
    /// both-see-stale outcome (waker skips the syscall AND the parker saw a
    /// stale predicate → blocked). The `SeqCst` fence here (between the bump and
    /// the `parked` load) and its twin in `park_enter` (between the bit-set and
    /// the caller's predicate loads) forbid it. Even absent the fences the
    /// UNCONDITIONAL bump keeps the failure bounded — the parker's kernel
    /// compare-value would already differ, so the worst case is one bounded
    /// slice, never a lost wake — but the fences make it model-correct rather
    /// than merely hardware-likely. The cost is one store-buffer drain per
    /// freeing drain: `dmb ish` on aarch64, `mfence` (or a locked stack op) on
    /// x86-64 — tens of cycles when the buffer is dirty, on neither target
    /// free. [`CreditWord`]'s own docs price the same fence the same way.
    pub fn note_credit_freed(&self) {
        self.wake_seq.fetch_add(1, Ordering::Release);
        core::sync::atomic::fence(Ordering::SeqCst);
        if self.parked.load(Ordering::Relaxed) != 0 {
            self.wake_word_wake();
        }
    }

    /// The kernel wake SYSCALL on the wake word — wake ALL parked producers
    /// (each re-derives `try_claim`; a spurious wake is a bounded no-op
    /// re-park).
    ///
    /// Linux: shared `FUTEX_WAKE` on `wake_seq`. macOS ≥ 14.4:
    /// `os_sync_wake_by_address_all`, gated on backend PRESENCE
    /// (`crate::os_sync::os_sync_backend().is_some()`). Other targets: no-op (no parker can block
    /// there — `park_wait_credit` reports not-performed). Return values / errnos
    /// deliberately ignored (the wake is only latency).
    ///
    /// # Why the wake gates on PRESENCE while the wait gates on `os_sync_active`
    ///
    /// The barrier's `os_sync_active` folds in the `OS_SYNC_DISABLED` LATCH and
    /// the `CERULION_BARRIER_OS_SYNC` kill switch, and both of those are
    /// **PROCESS-LOCAL** state: they describe THIS process's ability to WAIT.
    /// The parker on the other end of a credit word is BY DEFINITION in ANOTHER
    /// process (that is the whole point of the word), and its ability to be
    /// woken is decided by ITS latch, not ours. Gating the wake on our own
    /// therefore inverts the question: a consumer whose `os_sync` latched off —
    /// or one merely started with the kill switch set — would silently stop
    /// waking a peer producer that is kernel-blocked and perfectly wakeable.
    ///
    /// The asymmetry is priced, not assumed. A wake issued while we are locally
    /// latched costs ONE syscall whose rc is already ignored (and which the
    /// kernel answers cheaply — worst case `EINVAL`, no parkers, nothing woken);
    /// a wake SKIPPED costs the peer its full `cap`, once per drain, forever.
    /// The `parked` mask still gates the syscall, so a drain with nobody parked
    /// stays atomics-only either way.
    ///
    /// A host with NO backend at all is a different matter and still gates the wake:
    /// there the symbols do not exist, so there is nothing to call and no parker
    /// on any process could have blocked on the word.
    ///
    /// The Apple backend is RESOLVED BY `crate::os_sync` and shared, not
    /// re-derived: one `dlsym` cache and ONE `EINVAL`/`ENOTSUP` disable latch
    /// for the whole `os_sync` family, so a host on which the
    /// primitive is unusable degrades consistently across the barrier, the
    /// credit word, and the live-loop park's degraded-nap tier.
    #[inline]
    fn wake_word_wake(&self) {
        #[cfg(target_os = "linux")]
        {
            let word = &self.wake_seq as *const AtomicU32 as *const u32;
            // SAFETY: `word` is the live 4-byte `wake_seq` (offset-12
            // const-assert-pinned, 4-byte-aligned by layout), mapped for this
            // `&self` call; FUTEX_WAKE reads no user memory beyond keying on the
            // address. Other args are the documented "unused" FUTEX_WAKE values.
            unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    word,
                    libc::FUTEX_WAKE,
                    i32::MAX,
                    std::ptr::null::<libc::timespec>(),
                    std::ptr::null::<u32>(),
                    0u32,
                );
            }
        }
        #[cfg(target_os = "macos")]
        {
            // PRESENCE, not `os_sync_active`: the latch and the kill switch
            // describe THIS process's ability to WAIT; a parker in the peer
            // process may be kernel-blocked regardless of our local latch state.
            // A wake issued while locally latched is one harmless syscall (rc
            // deliberately ignored); a skipped wake is a peer riding its full
            // cap. See this method's docs for the full argument.
            if let Some(backend) = crate::os_sync::os_sync_backend() {
                // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq`
                // (offset-12 pinned), mapped for this `&self` call;
                // `os_sync_wake_by_address_all` reads no user memory beyond
                // keying on address + size. `backend.wake` is the
                // dlsym-resolved, signature-checked fn pointer.
                unsafe {
                    (backend.wake)(
                        self.wake_word_addr(),
                        crate::barrier::OS_SYNC_WORD_SIZE,
                        libc::OS_SYNC_WAKE_BY_ADDRESS_SHARED,
                    );
                }
            }
        }
    }

    /// The wake word's kernel wait/wake ADDRESS — the whole 4-byte `wake_seq`
    /// (no low-32 aliasing needed: it IS a `u32`). ONE recipe shared by the wake
    /// side (`wake_word_wake`) and the wait side
    /// ([`park_wait_credit`](Self::park_wait_credit)); size is the barrier's
    /// `OS_SYNC_WORD_SIZE` (4). Offset 12 from the 8-aligned struct base ⇒
    /// 4-byte alignment holds by construction (const-assert-pinned).
    #[cfg(target_os = "macos")]
    #[inline]
    fn wake_word_addr(&self) -> *mut c_void {
        &self.wake_seq as *const AtomicU32 as *mut c_void
    }

    /// Snapshot the wake-word epoch (`Acquire`).
    ///
    /// The parker takes this BEFORE its final [`try_claim`](Self::try_claim)
    /// re-derive, then hands it to
    /// [`park_wait_credit`](Self::park_wait_credit) as the kernel compare value
    /// — any bump landing after the snapshot fails the compare (no lost wake);
    /// any credit freed before it is caught by the re-derive.
    pub fn wake_seq_snapshot(&self) -> u32 {
        self.wake_seq.load(Ordering::Acquire)
    }

    /// Mark this edge's producer at index `producer_idx` as PARKED (set its
    /// `parked` bit).
    ///
    /// Returns `false` — no bit set — for a slot past [`PARKED_MASK_BITS`]
    /// (beyond the mask: that producer keeps the slice-timeout cadence; ONE loud
    /// process-wide warn names the degradation). The scope fence means the
    /// only reachable slot today is 0.
    ///
    /// The argument is a [`ProducerSlot`] — a position in THIS EDGE's
    /// `producer_ranks`, never a global rank. See that type's docs and the
    /// `parked` field docs for why the distinction is load-bearing under
    /// free-run.
    ///
    /// The `SeqCst` fence after the bit-set is the parker's half of the
    /// SB-litmus pair — see [`note_credit_freed`](Self::note_credit_freed).
    /// Callers use [`ParkedEdgeGuard`] so the bit is cleared on EVERY exit path.
    pub fn park_enter(&self, producer_idx: ProducerSlot) -> bool {
        if !producer_idx.fits_mask() {
            static WARNED: std::sync::atomic::AtomicBool =
                std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, Ordering::Relaxed) {
                tracing::warn!(
                    producer_idx = producer_idx.index(),
                    "credit park wake: producer index >= 32 exceeds the per-edge parked bitmask — \
                     this producer's park falls back to slice-timeout cadence (bounded, correct; \
                     only the wake-word latency win is lost). An edge with more than 32 producers \
                     is outside the wake-word fast path"
                );
            }
            return false;
        }
        self.parked
            .fetch_or(1u32 << producer_idx.index(), Ordering::AcqRel);
        // SB-litmus fence: order the bit-set BEFORE the caller's subsequent
        // predicate loads (`wake_seq_snapshot` + `try_claim`), pairing with the
        // fence in `note_credit_freed`. See that method's litmus writeup.
        core::sync::atomic::fence(Ordering::SeqCst);
        true
    }

    /// Clear this edge's producer `producer_idx`'s `parked` bit (park exit).
    ///
    /// Only ever called by [`ParkedEdgeGuard`]'s `Drop` (the slot fits the mask
    /// — `enter` returned `None` otherwise), so the guard below is
    /// unreachable through that path. It is not decoration: the
    /// `debug_assert!` is COMPILED OUT of a release build, where `1u32 << 32` is
    /// a MASKED shift (Rust's shift operators mask the count by the type's bit
    /// width, so it evaluates as `1u32 << 0`) — an out-of-range exit would
    /// therefore clear producer **0**'s LIVE bit, silently costing an unrelated,
    /// genuinely-parked producer its wake syscall. `pub`, so the caller set is
    /// not closed by this module; a debug-only check on a `pub` fn is a check
    /// that does not exist where it matters. Mirrors
    /// [`clear_parked_producer`](Self::clear_parked_producer)'s own early
    /// return.
    pub fn park_exit(&self, producer_idx: ProducerSlot) {
        debug_assert!(
            producer_idx.fits_mask(),
            "park_exit: producer slot {} was never bit-mapped",
            producer_idx.index()
        );
        if !producer_idx.fits_mask() {
            return;
        }
        self.parked
            .fetch_and(!(1u32 << producer_idx.index()), Ordering::AcqRel);
    }

    /// SUPERVISOR sweep — clear a DEAD producer's stale `parked` bit.
    ///
    /// A worker SIGKILLed inside its park never runs its [`ParkedEdgeGuard`]
    /// `Drop`, leaving its bit set forever: correctness is untouched (the bit
    /// only gates the consumer's wake syscall) but every freeing drain would pay
    /// a no-op kernel wake. The supervisor sweeps EDGE-KEYED — for every
    /// `credit_edges` entry whose `producer_ranks` contains the dead rank — which
    /// is why this takes a per-edge INDEX and not the barrier's global rank.
    ///
    /// A slot past [`PARKED_MASK_BITS`] is a no-op (such producers never set a
    /// bit).
    /// Idempotent; safe against the dead producer having NOT been parked
    /// (clearing an unset bit is a no-op).
    pub fn clear_parked_producer(&self, producer_idx: ProducerSlot) {
        if !producer_idx.fits_mask() {
            return;
        }
        self.parked
            .fetch_and(!(1u32 << producer_idx.index()), Ordering::AcqRel);
    }

    /// The current `parked` bitmask (`Acquire`) — a test/diagnostic accessor,
    /// never a control input.
    pub fn parked_mask(&self) -> u32 {
        self.parked.load(Ordering::Acquire)
    }

    /// Frames currently outstanding on this edge (`Acquire`) — a
    /// test/diagnostic accessor (Principle #3), never a control input; the gate
    /// is [`try_claim`](Self::try_claim).
    pub fn outstanding(&self) -> u64 {
        self.outstanding.load(Ordering::Acquire)
    }

    /// The declared depth stamped into this word (`Acquire`).
    pub fn depth(&self) -> u32 {
        self.depth.load(Ordering::Acquire)
    }

    /// The owner re-arm epoch (`Acquire`) — see the `epoch` field docs.
    pub fn epoch(&self) -> u32 {
        self.epoch.load(Ordering::Acquire)
    }

    /// The credit-blocked producer's bounded KERNEL BLOCK on the wake word —
    /// block until `wake_seq` moves off `snapshot` (credit was freed), `cap`
    /// elapses, or a spurious wake.
    ///
    /// Returns `true` if a real bounded block (or its internal bounded
    /// degrade-sleep) was performed — the caller skips its own pacing sleep;
    /// `false` if NOT performed (no primitive on this target, the `os_sync` tier
    /// inactive/latched, or a zero `cap`) — the caller keeps sleep-recheck
    /// pacing, byte-identical behavior.
    ///
    /// SPURIOUS-TOLERANT BY CONTRACT: the return value is only a pacing hint.
    /// The caller re-derives [`try_claim`](Self::try_claim) (and its other wake
    /// sources) after EVERY return — a wake with no state change is a bounded
    /// no-op re-park, never a correctness event (Principle #7 record-only).
    ///
    /// Tiers (the barrier's shape, verbatim):
    /// - **Linux**: shared `FUTEX_WAIT` on `wake_seq` with compare `snapshot`,
    ///   `cap`-bounded. Benign errnos (`EAGAIN` = word already moved, the
    ///   lost-wakeup guard; `EINTR`; `ETIMEDOUT`) return `true` silently; any
    ///   other errno warns once per distinct errno + sleeps a bounded 100µs
    ///   (capped at `cap`) so a persistently-failing syscall can never busy-loop
    ///   the caller.
    /// - **macOS ≥ 14.4**: `os_sync_wait_on_address_with_timeout` on `wake_seq`
    ///   (size 4, SHARED, `OS_CLOCK_MACH_ABSOLUTE_TIME`, relative `cap` ns). A
    ///   value mismatch returns a NON-NEGATIVE rc immediately. rc < 0:
    ///   `EINVAL`/`ENOTSUP` latch the SHARED process-wide os_sync disable (one
    ///   latch, one primitive family — the barrier's boundary wait and this park
    ///   degrade together) + warn once → `false`; benign (`ETIMEDOUT`/`EINTR`) →
    ///   `true`; other → warn-once-per-errno + bounded sleep → `true`.
    /// - **other targets**: `false` (no primitive — caller sleeps as today).
    pub fn park_wait_credit(&self, snapshot: u32, cap: Duration) -> bool {
        if cap.is_zero() {
            // Nothing to block for (the caller's remaining window is
            // exhausted); a zero-timeout kernel call risks EINVAL for no
            // benefit.
            return false;
        }
        #[cfg(target_os = "linux")]
        {
            let word = &self.wake_seq as *const AtomicU32 as *const u32;
            let ts = libc::timespec {
                tv_sec: cap.as_secs() as libc::time_t,
                tv_nsec: libc::c_long::from(cap.subsec_nanos()),
            };
            // SAFETY: `word` is the live 4-byte `wake_seq`, mapped for this
            // `&self` call; `ts` outlives the syscall. Shared (non-PRIVATE)
            // futex — the word crosses processes. The kernel's atomic
            // compare-and-block against `snapshot` IS the lost-wakeup guard.
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    word,
                    libc::FUTEX_WAIT,
                    snapshot,
                    &ts as *const libc::timespec,
                    std::ptr::null::<u32>(),
                    0u32,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if !crate::barrier::futex_wait_errno_is_benign(errno) {
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "credit wake word FUTEX_WAIT failed with an unexpected errno; the \
                             producer's credit park degrades to bounded sleep pacing for this \
                             errno"
                        );
                    }
                    std::thread::sleep(Duration::from_micros(100).min(cap));
                }
            }
            true
        }
        #[cfg(target_os = "macos")]
        {
            if !credit_os_sync_tier_active() {
                return false;
            }
            let Some(backend) = crate::os_sync::os_sync_backend() else {
                return false;
            };
            // Relative-ns timeout; `cap` is a bounded park slice (≤ the caller's
            // recheck chunk), far inside u64.
            let timeout_ns = cap.as_nanos() as u64;
            // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq`, mapped
            // for this `&self` call; `backend.wait` is the dlsym-resolved,
            // signature-checked `os_sync_wait_on_address_with_timeout`. SHARED
            // keys on the physical page (cross-process); a value mismatch
            // returns rc >= 0 immediately, reading no memory beyond addr+size.
            let rc = unsafe {
                (backend.wait)(
                    self.wake_word_addr(),
                    u64::from(snapshot),
                    crate::barrier::OS_SYNC_WORD_SIZE,
                    libc::OS_SYNC_WAIT_ON_ADDRESS_SHARED,
                    libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                    timeout_ns,
                )
            };
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if crate::os_sync::os_sync_errno_is_unrecoverable(errno) {
                    // One latch for the whole os_sync family (this park AND the
                    // barrier's boundary wait) — an EINVAL/ENOTSUP means the
                    // primitive is unusable, not one call shape.
                    if crate::os_sync::latch_os_sync_disabled() {
                        tracing::warn!(
                            errno,
                            "credit wake word os_sync_wait_on_address returned an unrecoverable \
                             errno (EINVAL/ENOTSUP); disabling the os_sync tier process-wide — \
                             the producer's credit park falls back to sleep-recheck pacing"
                        );
                    }
                    return false;
                }
                if !crate::os_sync::os_sync_errno_is_benign(errno) {
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "credit wake word os_sync_wait_on_address returned an unexpected \
                             errno; the producer's credit park degrades to bounded sleep pacing \
                             for this errno"
                        );
                    }
                    std::thread::sleep(Duration::from_micros(100).min(cap));
                }
            }
            true
        }
        #[cfg(not(any(target_os = "linux", target_os = "macos")))]
        {
            false
        }
    }
}

/// An edge-local PRODUCER SLOT — the position of a rank inside ONE
/// credit edge's `producer_ranks` list, and therefore which bit of that edge's
/// `parked` mask belongs to it.
///
/// # Why this is a type and not a `u32`
///
/// The `parked` mask is PER EDGE, so the bit a producer owns is at its position
/// in that edge's producer list — NOT at its global rank. The two are different
/// numbers that are both small non-negative integers, and under the scope
/// fence every edge has exactly ONE producer, so the slot is always 0 while a
/// rank can be anything: a site that passed a rank where a slot was wanted would
/// be WRONG on every multi-rank deployment and RIGHT on every single-edge test,
/// which is the signature of a bug that ships.
///
/// The distinction was already a type on the supervisor's sweep path
/// (`cerulion_cli_engine::multiprocess`), but the three methods it feeds took a
/// bare `u32`, so the guarantee stopped at the crate boundary and any caller
/// could still hand a rank straight to the word. This type IS that guarantee,
/// now held where the bit is actually set: [`CreditShared::park_enter`],
/// [`CreditShared::park_exit`], [`CreditShared::clear_parked_producer`] and
/// [`ParkedEdgeGuard::enter`] take nothing else.
///
/// # Scope
///
/// [`ProducerSlot::new`] is public and takes a `u32`, so this makes the
/// confusion NAMED rather than impossible — a caller can still write
/// `ProducerSlot::new(rank)`. What it buys is that the mistake now has to be
/// SPELLED: an accidental positional swap (the reachable shape — two `u32`s in
/// scope, one call) stops compiling, and a deliberate `new(rank)` is a line a
/// reader can see is wrong. The mint on the supervisor path
/// (`multiprocess::producer_slot_of`) derives the slot from the edge's own
/// producer list, so production never constructs one from a rank at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProducerSlot(u32);

impl ProducerSlot {
    /// The slot at edge-local index `index`.
    ///
    /// The argument is an INDEX INTO AN EDGE'S PRODUCER LIST. It is never a
    /// rank — see the type docs for why the two are worth telling apart.
    pub const fn new(index: u32) -> Self {
        Self(index)
    }

    /// The raw edge-local index — for the bit arithmetic and for diagnostics.
    pub const fn index(self) -> u32 {
        self.0
    }

    /// Does this slot address a bit in the 32-wide `parked` mask?
    ///
    /// `false` means the producer keeps the slice-timeout cadence: correct,
    /// bounded, and short only the wake-word latency win.
    pub const fn fits_mask(self) -> bool {
        self.0 < PARKED_MASK_BITS
    }
}

/// Width of [`CreditShared`]'s `parked` bitmask, and therefore the number of
/// producers ONE edge can put on the wake-word fast path.
///
/// Named rather than written `32` at each of the four sites that compare
/// against it: they must agree, and a literal repeated four times is four
/// chances to disagree.
pub const PARKED_MASK_BITS: u32 = 32;

/// RAII holder of a producer's `parked` bit — the ONLY caller of
/// [`CreditShared::park_exit`], so the bit is cleared on EVERY exit path (wake,
/// timeout, early loop exit, panic unwind) of the wake-word block.
///
/// `enter` returns `None` for a slot past [`PARKED_MASK_BITS`] (beyond the
/// bitmask — that producer keeps the slice-timeout cadence; `park_enter` warns
/// once) so a `None` guard doubles as the "no wake-word fast path for this
/// producer" signal.
///
/// SCOPE: the bit's semantic is "a producer is blocked on this credit word RIGHT
/// NOW" — the guard is scoped to the kernel-block arm of the caller's park (set
/// immediately before the snapshot→re-derive→block sequence, dropped right
/// after), NOT the whole park. The brief pre-block window where the bit is set
/// but the producer is not yet in the kernel is REQUIRED by the SB-litmus
/// protocol (bit-set BEFORE the predicate re-derive) and costs at most one early
/// syscall, absorbed by the block's fresh snapshot.
#[must_use = "the parked bit is cleared on drop — bind the guard for the block's lifetime"]
pub struct ParkedEdgeGuard<'a> {
    credit: &'a CreditShared,
    producer_idx: ProducerSlot,
}

impl<'a> ParkedEdgeGuard<'a> {
    /// Set producer slot `producer_idx`'s parked bit and return the clearing
    /// guard, or `None` for a beyond-mask slot (no bit set — see
    /// [`CreditShared::park_enter`]).
    pub fn enter(credit: &'a CreditShared, producer_idx: ProducerSlot) -> Option<Self> {
        if credit.park_enter(producer_idx) {
            Some(Self {
                credit,
                producer_idx,
            })
        } else {
            None
        }
    }
}

impl Drop for ParkedEdgeGuard<'_> {
    fn drop(&mut self) {
        self.credit.park_exit(self.producer_idx);
    }
}

/// The credit plane's OWN kill switch.
///
/// `CERULION_CREDIT_WAKE=0` turns the producer's credit park off: it keeps its
/// slice-timeout cadence exactly as it did before this plane existed. Correct
/// and bounded — the pre-fire gate is unchanged and re-derives `try_claim` on
/// every pass — so the only thing lost is latency. Anything other than `0`
/// leaves it ON, and a value that is neither `0` nor `1` warns once and stays
/// ON (fail-safe: a typo must not silently disable a latency plane).
pub const CREDIT_WAKE_ENV: &str = "CERULION_CREDIT_WAKE";

/// The credit plane's own os_sync-tier switch on macOS.
///
/// Distinct from the barrier's `CERULION_BARRIER_OS_SYNC` on purpose. Both
/// words ride ONE backend and ONE unrecoverable-errno latch, but a switch is a
/// statement about a CONSUMER, not about the backend: an operator disabling the
/// barrier's macOS wake tier has said nothing about the credit plane, and
/// before this the credit park silently obeyed that decision anyway because it
/// delegated its whole gate to the barrier's.
///
/// `crate::os_sync`'s module docs state the discipline this follows: one
/// grammar (`0` disables, unset/`1` enable, garbage warns and stays on) shared
/// by every consumer, and a SEPARATE resolve-and-warn wrapper per consumer
/// whose warn names ITS OWN variable.
pub const CREDIT_OS_SYNC_ENV: &str = "CERULION_CREDIT_OS_SYNC";

/// Is the producer's credit park enabled at all? Resolved ONCE per process.
///
/// Consulted by the runtime before it watches any credit word, so `=0` costs a
/// single cached atomic load and nothing else.
pub fn credit_wake_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| {
        !resolve_credit_wake_disabled(std::env::var(CREDIT_WAKE_ENV).ok().as_deref())
    })
}

/// Resolve the raw [`CREDIT_WAKE_ENV`] value to "disabled?", emitting the loud
/// garbage warn. Split out of the `OnceLock` closure so the warn is
/// `#[traced_test]`-pinnable without env / `OnceLock` games — the shape
/// `barrier::resolve_os_sync_disabled` uses, for the same reason.
fn resolve_credit_wake_disabled(raw: Option<&str>) -> bool {
    let (disabled, was_garbage) = crate::kill_switch::parse_kill_switch(raw);
    if was_garbage {
        tracing::warn!(
            env = CREDIT_WAKE_ENV,
            got = %raw.unwrap_or(""),
            "CERULION_CREDIT_WAKE is set but not `0` (disable) or `1`/unset (enable); \
             keeping the producer credit park ON (`0` is the explicit kill switch)"
        );
    }
    disabled
}

/// Is the CREDIT word's macOS os_sync tier active?
///
/// The shared BACKEND fact (`crate::os_sync::os_sync_backend_usable` — resolved
/// and not latched) AND this plane's OWN [`CREDIT_OS_SYNC_ENV`] switch. It does
/// NOT consult `crate::barrier::os_sync_active`, and that is the whole point:
/// the barrier's gate ANDs in `CERULION_BARRIER_OS_SYNC`, so routing through it
/// made `CERULION_BARRIER_OS_SYNC=0` silently disable the credit tier while
/// `docs/user-api.md` promised the two switches were independent. Both words still
/// ride ONE backend and ONE unrecoverable-errno latch — the FACT is read from
/// one place — but the DECISION is this consumer's.
#[cfg(target_os = "macos")]
fn credit_os_sync_tier_active() -> bool {
    crate::os_sync::os_sync_backend_usable() && !credit_os_sync_kill_switch()
}

/// The credit plane's own macOS os_sync switch, resolved ONCE per process.
#[cfg(target_os = "macos")]
fn credit_os_sync_kill_switch() -> bool {
    static KILL_SWITCH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *KILL_SWITCH.get_or_init(|| {
        resolve_credit_os_sync_disabled(std::env::var(CREDIT_OS_SYNC_ENV).ok().as_deref())
    })
}

/// Resolve the raw [`CREDIT_OS_SYNC_ENV`] value to "disabled?", with a warn
/// that names THIS variable — the whole point of the per-consumer wrapper rule.
#[cfg(target_os = "macos")]
fn resolve_credit_os_sync_disabled(raw: Option<&str>) -> bool {
    let (disabled, was_garbage) = crate::kill_switch::parse_kill_switch(raw);
    if was_garbage {
        tracing::warn!(
            env = CREDIT_OS_SYNC_ENV,
            got = %raw.unwrap_or(""),
            "CERULION_CREDIT_OS_SYNC is set but not `0` (disable) or `1`/unset (enable); \
             keeping the macOS os_sync credit-wake tier ON (`0` is the explicit kill switch)"
        );
    }
    disabled
}

/// Whether THIS host has a kernel-block PRIMITIVE for the credit wake word.
///
/// Linux always (shared futex); macOS iff the shared `os_sync` BACKEND is usable
/// (resolved and not latched off) and this plane's own [`CREDIT_OS_SYNC_ENV`] is
/// not set to `0`; other targets never.
///
/// It reads the BACKEND fact from `crate::os_sync::os_sync_backend_usable` —
/// both words ride the same backend and the same unrecoverable-errno latch, so
/// two answers to "does this host have the primitive" could only ever disagree
/// by being wrong — and applies its OWN switch on top. That split is the point:
/// a shared FACT is read from one place, while a per-consumer DECISION is made
/// per consumer.
///
/// It deliberately does NOT go through
/// `crate::barrier::wake_word_block_primitive_available`. That function ANDs in
/// `CERULION_BARRIER_OS_SYNC`, so delegating to it made the BARRIER's switch
/// silently disable the CREDIT tier — the coupling this doc used to describe as
/// a clean split while the code did not deliver it.
///
/// **PRIMITIVE EXISTENCE ONLY** — this does not imply any particular park path
/// is taken on a given box.
pub fn credit_wake_word_primitive_available() -> bool {
    #[cfg(target_os = "linux")]
    {
        // Shared futex on the credit word; no per-host resolution, no switch
        // beyond CREDIT_WAKE_ENV (which gates the WATCH LIST, not the tier).
        true
    }
    #[cfg(target_os = "macos")]
    {
        credit_os_sync_tier_active()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        false
    }
}

// The FNV-1a-64 fold both `credit_shm_name` variants derive their object name
// with — `crate::shm_map::fnv1a64`, IMPORTED rather than written out here.
//
// This is the fourth copy of that arithmetic retiring. When this module landed
// it argued the copy was cheaper than the coupling ("eight lines of frozen
// arithmetic with no state … a shared helper would put a cross-module
// dependency between four otherwise-independent naming schemes for no gain"),
// and DEFERRED the whole `shm_map` extraction to a later consolidation while
// ACKNOWLEDGING the fourth copy rather than excusing it. The dependency turned
// out to be the wrong way round: the four naming schemes still do not know
// about each other — they know about the POSIX map substrate they were all
// already sitting on, which is where the `shm_open` / `ftruncate`-once / `mmap`
// / `shm_unlink` sequence AROUND them now lives too.
//
// `shm_map` is a PORTABLE module (only its POSIX half is `#[cfg(unix)]`), so
// this portable module can name it in code and in a link without breaking a
// non-unix `cargo doc` under CI's `RUSTDOCFLAGS=-D warnings` (see
// `cfg_audit_test`) — which is exactly why it is gated per item rather than at
// the `pub(crate) mod` declaration.
use crate::shm_map::fnv1a64;

/// Derive the per-edge `id` a credit word is named by:
/// `credit\x1f{topic}\x1f{consumer_node}\x1f{consumer_input}`.
///
/// This is the edge IDENTITY — the same `(topic, consumer_node, consumer_input)`
/// key the in-process mirror maps use — rendered into the string
/// `credit_shm_name` hashes. The 0x1F unit separators keep the three components
/// unambiguous (`("a/b", "c", "d")` can never alias `("a", "b/c", "d")`), and
/// hashing makes the result filename-safe for ANY topic name by construction.
///
/// PER-EDGE, not per-topic: a multi-consumer block topic has one word per
/// consumer input, because that is the granularity `depth` and `outstanding`
/// describe (the doorbell's per-topic naming does not fit).
///
/// # SCOPE of the separator
///
/// The separator makes the ordinary boundary shift unambiguous — a topic name
/// ending where a node id begins, e.g. `("/a/b", "c", "d")` against
/// `("/a", "b/c", "d")`, cannot alias. It does NOT make a component containing a
/// LITERAL 0x1F byte unambiguous: `("a\x1fb", "c", "d")` and
/// `("a", "b", "c\x1fd")` render the same string, and this function does not
/// escape or length-prefix. That is stated rather than fixed because this
/// recipe is frozen and because no component can carry a control byte in
/// practice — a topic name reaches an iceoryx2 service name and a node id / port
/// name is a Rust identifier — so the reachable hazard is the boundary shift,
/// which IS closed. If a component ever becomes free-form, this needs
/// length-prefixing, not a wider separator.
pub fn credit_edge_id(topic: &str, consumer_node: &str, consumer_input: &str) -> String {
    format!("credit\u{1f}{topic}\u{1f}{consumer_node}\u{1f}{consumer_input}")
}

/// Derive the POSIX SHM object name for `(ns, id)`. Pure (no I/O), so it is
/// hermetically testable on every OS. Stable across processes, so the owner
/// ([`MappedCredit::create_owned`]) and a peer ([`MappedCredit::open_unowned`])
/// of the same `(ns, id)` derive the same name → map the same page.
///
/// Per-OS shape (the barrier's split, for the same reasons):
/// - **Linux** — `credit_shm_name_verbose`: `/cer_crd_<ns>_<fnv1a64(id):016x>`.
///   The raw `<ns>` in the `/dev/shm` filename is a deliberate debuggability aid
///   (an `ls /dev/shm` names the deployment).
/// - **non-Linux (macOS)** — `credit_shm_name_compact`:
///   `/cer_crd_<fnv1a64(ns 0x1f id):016x>` (25 chars). macOS caps POSIX SHM names
///   at 31 chars (`PSHMNAMLEN`, incl. the leading slash) and real deployment
///   namespaces (`cerdep_<graph>_<nonce>`) blow far past it, so BOTH components
///   are hashed into a fixed-width token.
///
/// Either shape preserves the tenant partition (different `ns` → different name;
/// the compact form separates `ns`/`id` with a 0x1F unit separator so `("ab","c")`
/// and `("a","bc")` can never alias). The `/cer_crd_` family prefix is
/// PREFIX-FREE against `/cer_bar_`, `/cer_rg_`, `/cer_db_` and `/cer_sta_` (there
/// is a known iceoryx2 hazard with string-prefix collisions).
///
/// `pub` on the `crate::shm_ring::ring_shm_name` precedent (a prose mention, not
/// a link — `shm_ring` is `#[cfg(unix)]`-only, see `fnv1a64`): an out-of-crate
/// test cannot CRAFT the hostile segment shapes `open_unowned` refuses (a
/// zero-length orphan, an UNARMED page) without deriving the same name the
/// production open will, and a second copy of the recipe in a test would be a
/// recipe that can drift from the one under test.
pub fn credit_shm_name(ns: &str, id: &str) -> String {
    #[cfg(target_os = "linux")]
    {
        credit_shm_name_verbose(ns, id)
    }
    #[cfg(not(target_os = "linux"))]
    {
        credit_shm_name_compact(ns, id)
    }
}

/// The Linux name shape: `/cer_crd_<ns>_<fnv1a64(id):016x>` — `ns` verbatim
/// (deployment-visible in `/dev/shm`), `id` hashed. See `credit_shm_name`.
#[cfg(any(target_os = "linux", test))]
fn credit_shm_name_verbose(ns: &str, id: &str) -> String {
    let h = fnv1a64(id.as_bytes());
    format!("/cer_crd_{ns}_{h:016x}")
}

/// The macOS-safe name shape: `/cer_crd_<fnv1a64(ns 0x1f id):016x>` — 25 chars,
/// under the macOS 31-char `PSHMNAMLEN` cap for ANY `(ns, id)`. The 0x1F unit
/// separator keeps the `(ns, id)` split unambiguous. See `credit_shm_name`.
#[cfg(any(not(target_os = "linux"), test))]
fn credit_shm_name_compact(ns: &str, id: &str) -> String {
    // hot-path-alloc-ok: name derivation runs only at create/open (cold path).
    let mut key = Vec::with_capacity(ns.len() + 1 + id.len());
    key.extend_from_slice(ns.as_bytes());
    key.push(0x1f);
    key.extend_from_slice(id.as_bytes());
    let h = fnv1a64(&key);
    format!("/cer_crd_{h:016x}")
}

/// Refuse a `depth == 0` stamp at the CREATE seam — ONE rule, called by BOTH
/// [`MappedCredit::create_owned`] arms (POSIX and the non-Unix registry), so
/// they cannot disagree about what a valid word is.
///
/// A word stamped `depth == 0` reads FULL unconditionally
/// ([`CreditShared::is_full`] is `outstanding >= 0`), so its producer defers on
/// EVERY tick entry, forever: a silent WEDGE, not a slow edge, and one with no
/// recovery path (nothing lowers `outstanding` below zero). Refusing at the
/// create seam is the only place it can be caught cheaply — after this the value
/// is in a shared page that a peer will open and believe.
///
/// This is a PLAN-STAMPING bug, never operator input: `GraphTopology::validate`
/// enforces `depth >= 1` on every build path, so a 0 arriving here means the
/// plan stamped the word from something other than the loaded planning topology.
/// The message says so, because "your graph is invalid" would send the reader to
/// a file that is fine.
///
/// `reinit`'s `debug_assert!` is deliberately KEPT alongside this: it guards the
/// (private, create-only) re-init path itself, which is one layer below the
/// public constructor this check fronts.
fn reject_zero_depth(depth: u32) -> std::io::Result<()> {
    if depth == 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "credit word stamped depth 0: `outstanding >= 0` holds unconditionally, so this \
             edge's producer would defer entry to EVERY tick forever (a silent wedge, with no \
             drain that can recover it). GraphTopology::validate enforces depth >= 1 on every \
             build path; a 0 here is a plan-stamping bug (the depth was not taken from the \
             loaded planning topology), not an operator input.",
        ));
    }
    Ok(())
}

/// Refuse an UNARMED segment at the OPEN seam — ONE rule, called by BOTH
/// [`MappedCredit::open_unowned`] arms, so they cannot disagree about what a
/// mappable word is.
///
/// The size check one layer up catches an owner that died between `shm_open`
/// and `ftruncate`; this catches the NEXT instant — an owner that died between
/// `ftruncate` and `reinit`. That leaves a FULL-SIZE, correctly-named,
/// ZERO-FILLED segment, which the size check accepts and which then reads
/// `depth == 0`: [`CreditShared::is_full`] is `outstanding >= depth`, which at
/// depth 0 holds unconditionally, so the peer's producer defers entry to EVERY
/// tick forever, SILENTLY, with no drain that can recover it. Exactly the wedge
/// [`reject_zero_depth`] refuses on the create side, arriving through the open
/// side instead.
///
/// The marker is the `epoch`, and reading it this way is what its own field docs
/// already say it is good for: `reinit` bumps it and nothing else does, so `0`
/// means "no owner ever armed this page" while `>= 1` means one did. Ordering is
/// what makes it sufficient rather than merely indicative — `reinit`'s bump is a
/// `Release` RMW and [`CreditShared::epoch`] loads `Acquire`, so an epoch this
/// side observes as non-zero carries `reinit`'s depth store with it. A separate
/// `depth != 0` check would therefore be a second spelling of this one, not a
/// second guard.
///
/// This is NOT a use of the RESERVED in-place re-arm seam: the reservation is
/// about a FUTURE `reinit` on a page peers already hold (which would make the
/// VALUE meaningful to somebody other than its creator), and armed-vs-bare is
/// the one reading that stays true whether or not that seam is ever opened.
fn reject_unarmed(name: &str, epoch: u32) -> std::io::Result<()> {
    if epoch == 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "credit segment {name:?} exists at the right size but was never ARMED (epoch 0, \
                 so no owner ever stamped a depth into it). The usual cause is an owner that \
                 died between ftruncate and reinit, leaving a zero-filled orphan under the \
                 name. Mapping it anyway is a SILENT WEDGE, not a slow edge: the word reads \
                 depth 0, `outstanding >= 0` holds unconditionally, and this edge's producer \
                 defers entry to every tick forever with no drain that can recover it. Restart \
                 the owner (the supervisor's next create_owned unlinks and replaces the \
                 orphan); a stale segment can also be cleared by hand from /dev/shm."
            ),
        ));
    }
    Ok(())
}

#[cfg(unix)]
mod imp {
    use super::{credit_shm_name, reject_unarmed, reject_zero_depth, CreditShared, CREDIT_BYTES};
    use crate::shm_map::{create_exclusive, unlink, unmap, OpenedSegment};
    use std::ffi::CString;
    use std::io;
    use std::os::raw::c_void;

    /// A process-shared SHM mapping of a [`CreditShared`] — one cross-process
    /// `block` edge's credit word living in a `MAP_SHARED` page.
    ///
    /// Construct via [`MappedCredit::create_owned`] (the SUPERVISOR — creates,
    /// sizes and arms the segment, owns the name and `shm_unlink`s it on drop)
    /// or [`MappedCredit::open_unowned`] (a worker — STRICT open-existing, maps
    /// only). [`Deref`](std::ops::Deref)s to the [`CreditShared`], so
    /// `try_claim`/`record_drained`/… reach through into the shared page.
    #[must_use = "the mapped credit word is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local"]
    pub struct MappedCredit {
        /// Pointer to the mapped [`CreditShared`] (offset 0 of the `MAP_SHARED`
        /// page).
        ptr: *mut CreditShared,
        /// The POSIX SHM object name — retained so an OWNED mapping can
        /// `shm_unlink` it on drop.
        name: CString,
        /// `true` for an owner-created mapping that owns the name and must
        /// `shm_unlink` it on drop; `false` for a peer mapping (munmap only).
        owns_name: bool,
    }

    // SAFETY: the credit word is a `CreditShared` (atomics only) in a shared
    // page. All access goes through atomic load/store/fetch ops, so it is sound
    // to send/share the handle across threads (the OS guarantees the MAP_SHARED
    // page is coherent across mappings; the atomics give the intra-process
    // ordering). `name`/`owns_name` are plain Send+Sync data.
    unsafe impl Send for MappedCredit {}
    unsafe impl Sync for MappedCredit {}

    impl MappedCredit {
        /// Create + map + arm the credit word for `(ns, id)` as the OWNER (the
        /// supervisor, before any worker spawns). `O_EXCL`-creates a FRESH
        /// segment, sizes it to one cache line, stamps `depth`, bumps the re-arm
        /// epoch, and takes ownership of the name (drop `shm_unlink`s it).
        ///
        /// The unlink-first design is deliberate (the barrier's orphan
        /// clear): a crashed prior run's segment is cleared so the `O_EXCL`
        /// create yields a FRESH zero-filled object. That makes the `O_EXCL` a
        /// best-effort double-supervisor DETECTOR rather than a lock — a
        /// NON-racing second create succeeds after clearing the first owner's
        /// name, leaving the first owner's mapping alive on the old (now
        /// unnamed) object.
        pub fn create_owned(ns: &str, id: &str, depth: u32) -> io::Result<Self> {
            // BEFORE any syscall: a refused create must leave no name behind.
            reject_zero_depth(depth)?;
            let name = CString::new(credit_shm_name(ns, id))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;

            // Unlink-first (clear a crashed-prior-run orphan — the barrier's
            // rule, which is what makes the `O_EXCL` below a best-effort
            // double-supervisor DETECTOR rather than a lock) + `O_EXCL` create +
            // `ftruncate` EXACTLY ONCE (macOS EINVALs on a re-truncate) +
            // `mmap`, cleanup arms included: `shm_map::create_exclusive`.
            let addr = create_exclusive(&name, CREDIT_BYTES)?;

            let ptr = addr as *mut CreditShared;
            // SAFETY: `ptr` is a freshly-mapped, exclusively-owned (O_EXCL),
            // page-aligned (mmap returns page alignment, ≥ the 8-byte
            // CreditShared alignment) valid CreditShared. The fresh O_EXCL
            // object is zero-filled, so `reinit` stamps the depth and bumps the
            // epoch (and re-zeros the rest for the orphan-survivor case).
            unsafe {
                (*ptr).reinit(depth);
            }
            // AT BIRTH: a capture `fork` child must not
            // inherit a live cross-process backpressure word. Never fatal.
            // SAFETY: the region just mapped and exclusively owned by this handle.
            unsafe {
                crate::state_carrier::exclude_at_birth(
                    ptr as *mut c_void,
                    CREDIT_BYTES,
                    crate::state_carrier::ForkExcludedMapping::CreditPage,
                );
            }
            Ok(Self {
                ptr,
                name,
                owns_name: true,
            })
        }

        /// Open + map an EXISTING credit word for `(ns, id)` as a peer (a
        /// worker). STRICT open-existing — `O_RDWR` with NO `O_CREAT`, so a
        /// missing object (ENOENT) is an error: the supervisor must
        /// [`create_owned`](Self::create_owned) first.
        ///
        /// A silent create would be the worst available failure: the worker
        /// would get its OWN zeroed word, read `depth == 0`, and defer forever
        /// while the real edge sat untouched.
        ///
        /// A segment that EXISTS but is SHORTER than [`CREDIT_BYTES`] is refused
        /// with `InvalidData` rather than mapped — see the size check in the
        /// body for why that would otherwise be a SIGBUS rather than an error.
        /// A segment that is full-size but was never ARMED (a creator that died
        /// one instant later, between `ftruncate` and `reinit`) is refused with
        /// `InvalidData` too — see `reject_unarmed` (private, so a prose mention:
        /// rustdoc refuses a public->private link), and note that the size
        /// check structurally cannot reach that state.
        ///
        /// The returned mapping's lifetime is INDEPENDENT of the name's: after
        /// the owner drops (its `Drop` `shm_unlink`s the name), a FRESH
        /// `open_unowned` fails with ENOENT — but THIS handle's mapping, and any
        /// park blocked through it, stay valid until this handle's own `Drop`
        /// `munmap`s (POSIX: the object persists until the last unmap).
        pub fn open_unowned(ns: &str, id: &str) -> io::Result<Self> {
            let name = CString::new(credit_shm_name(ns, id))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            // STRICT open-existing (no `O_CREAT`) — a missing segment is ENOENT
            // → `Err`, never a silent create. `seg`'s Drop closes the descriptor
            // on every early return below.
            let seg = OpenedSegment::open(&name)?;
            // SIZE CHECK before the map. `create_owned` does `shm_open` THEN
            // `ftruncate`, so an owner that died between the two leaves a
            // named, ZERO-LENGTH object under a perfectly good name — and what
            // happens next is PLATFORM-DEPENDENT, which is the real argument
            // for checking here rather than letting `mmap` decide:
            //
            //   - Linux: `mmap` of CREDIT_BYTES SUCCEEDS (length is not
            //     validated against the object at map time), and the failure
            //     lands on the FIRST TOUCH as SIGBUS — a process kill, in
            //     whatever worker happened to open it, with no Rust-level
            //     error anywhere to report or attribute.
            //   - macOS: `mmap` REFUSES (measured — see the `O_CREAT`
            //     note on `strict_open_of_a_missing_segment_errs_with_not_found`
            //     in `tests/credit_test.rs`), so the caller gets an errno whose
            //     text says nothing about what was actually wrong.
            //
            // One explicit check makes it ONE loud, attributable `Err` on both.
            // The THRESHOLD and its diagnostic stay here, at the site: the
            // substrate reports the size, this module decides what is valid.
            let size = seg.size()?;
            // `i64` on both sides rather than `libc::off_t`: the substrate
            // reports a widened size, and a 32-bit-`off_t` target would not
            // otherwise type-check here.
            if size < CREDIT_BYTES as i64 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!(
                        "credit segment {name:?} is {size} bytes, short of the {CREDIT_BYTES} \
                         this word needs. The usual cause is an owner that died between \
                         shm_open and ftruncate, leaving a zero-length orphan under the name. \
                         Mapping it anyway fails differently per platform — on Linux the mmap \
                         SUCCEEDS and the first touch raises SIGBUS (a process kill with no \
                         error to report); on macOS the mmap is refused with an unrelated \
                         errno. The supervisor's next create_owned unlinks and replaces it."
                    ),
                ));
            }
            // No `ftruncate` — the owner already sized the object. `MAP_SHARED`
            // is what shares the owner's atomics.
            let addr = seg.map_shared(CREDIT_BYTES)?;

            // ARM CHECK — the size check above cannot reach it. A creator that
            // died between `ftruncate` and `reinit` leaves a full-size,
            // zero-filled page the size check ACCEPTS; see `reject_unarmed`.
            // Refuse BEFORE the fork-exclusion registration below, so a refused
            // open leaves no trace of itself anywhere.
            //
            // SAFETY: `addr` is a freshly-mapped, page-aligned (>= the 8-byte
            // CreditShared alignment) region of at least CREDIT_BYTES, and
            // `CreditShared` is atomics-only, so a shared reference to it is
            // valid however the page is concurrently written.
            let epoch = unsafe { (*(addr as *const CreditShared)).epoch() };
            if let Err(e) = reject_unarmed(&name.to_string_lossy(), epoch) {
                // SAFETY: unmap exactly the region just mapped above; nothing
                // references it.
                unsafe { unmap(addr, CREDIT_BYTES) };
                return Err(e);
            }

            // Amendment 10 covers a PEER mapping too — this process forks its
            // own capture children whoever created the segment.
            // SAFETY: the region just mapped and exclusively owned by this handle.
            unsafe {
                crate::state_carrier::exclude_at_birth(
                    addr,
                    CREDIT_BYTES,
                    crate::state_carrier::ForkExcludedMapping::CreditPage,
                );
            }
            Ok(Self {
                ptr: addr as *mut CreditShared,
                name,
                owns_name: false,
            })
        }

        /// The mapped [`CreditShared`]'s virtual address — a test/inspection
        /// accessor (so a test can prove two mappings of the same physical page
        /// have distinct virtual addresses).
        pub fn addr(&self) -> *const CreditShared {
            self.ptr
        }
    }

    impl std::ops::Deref for MappedCredit {
        type Target = CreditShared;
        fn deref(&self) -> &CreditShared {
            // SAFETY: `ptr` is a valid, page-aligned mapping of a `CreditShared`
            // that stays mapped for as long as `self` lives (unmapped only in
            // `Drop`).
            unsafe { &*self.ptr }
        }
    }

    /// Hand-written (the `MappedStateArm` precedent) so the raw pointer is
    /// rendered as the segment NAME an operator can `ls /dev/shm` for, rather
    /// than an address that means nothing outside this process.
    impl std::fmt::Debug for MappedCredit {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("MappedCredit")
                .field("name", &self.name)
                .field("owns_name", &self.owns_name)
                .field("outstanding", &self.outstanding())
                .field("depth", &self.depth())
                .field("epoch", &self.epoch())
                .finish()
        }
    }

    /// Teardown = this process's OWN `munmap` + (owner only) `shm_unlink`.
    ///
    /// Unlink-while-parked contract: `shm_unlink` removes only the NAME; the
    /// underlying object persists until the LAST mapping is unmapped. So an
    /// owner dropping mid-run can never invalidate a peer's live mapping — nor a
    /// producer PARKED on it (the futex/os_sync target stays memory-backed; the
    /// waiter returns on its cap, never SIGSEGV/SIGBUS). A SIGBUS would require
    /// an `ftruncate` SHRINK of the live object, which nothing in this codebase
    /// performs.
    impl Drop for MappedCredit {
        fn drop(&mut self) {
            // SAFETY: unmap exactly the page create_owned/open_unowned mapped;
            // nothing references it after this.
            unsafe {
                unmap(self.ptr as *mut c_void, CREDIT_BYTES);
            }
            if self.owns_name {
                // Best-effort unlink of the name THIS mapping created and owns.
                // Existing peer mappings keep the object alive until they unmap.
                unlink(&self.name);
            }
        }
    }
}

#[cfg(not(unix))]
mod imp {
    use super::{credit_shm_name, reject_unarmed, reject_zero_depth, CreditShared};
    use std::collections::HashMap;
    use std::io;
    use std::sync::{Arc, Mutex, OnceLock};

    /// Process-global registry mapping SHM name → the shared [`CreditShared`].
    ///
    /// The non-Unix (future-Windows) stub shares a `CreditShared` BY NAME through
    /// this registry — NOT a fresh `Arc` per handle — so the behavioral tests are
    /// a REAL shared-state test on such a host too (a per-handle `Arc` would make
    /// them a tautology / false-green). Poison-tolerant.
    fn registry() -> &'static Mutex<HashMap<String, Arc<CreditShared>>> {
        static REG: OnceLock<Mutex<HashMap<String, Arc<CreditShared>>>> = OnceLock::new();
        REG.get_or_init(|| Mutex::new(HashMap::new()))
    }

    /// Non-Unix stub of [`MappedCredit`] (future Windows target only — macOS uses
    /// the REAL POSIX `shm_open` + `MAP_SHARED` arm above).
    ///
    /// It exists so the code COMPILES off Unix AND the BEHAVIORAL tests
    /// (rendezvous, the different-ns negative control, owner-drop-then-open-fails)
    /// run there. It CANNOT verify the real cross-address-space `MAP_SHARED`
    /// page. State is shared by name via the process-global registry;
    /// `Arc<CreditShared>` is auto `Send + Sync` (`CreditShared` is atomics-only),
    /// so no `unsafe impl` is needed.
    #[must_use = "the credit handle keeps its registry entry alive (and, if owned, removes it on drop) — bind it to a named local"]
    pub struct MappedCredit {
        shared: Arc<CreditShared>,
        name: String,
        owns_name: bool,
    }

    impl MappedCredit {
        /// Owner constructor — inserts a FRESH [`CreditShared`] into the registry
        /// under `(ns, id)`'s name, REPLACING any orphan (parity with the POSIX
        /// arm's unlink-then-`O_EXCL`). `reinit` runs so the epoch starts at 1 on
        /// EVERY platform.
        pub fn create_owned(ns: &str, id: &str, depth: u32) -> io::Result<Self> {
            // BEFORE the registry insert: a refused create must leave no entry.
            reject_zero_depth(depth)?;
            let name = credit_shm_name(ns, id);
            let shared = Arc::new(CreditShared::new(0));
            shared.reinit(depth);
            registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(name.clone(), Arc::clone(&shared));
            Ok(Self {
                shared,
                name,
                owns_name: true,
            })
        }

        /// Peer constructor — STRICT lookup of an existing entry (mirrors the
        /// POSIX ENOENT): a missing name is `NotFound`, never a silent create.
        pub fn open_unowned(ns: &str, id: &str) -> io::Result<Self> {
            let name = credit_shm_name(ns, id);
            let shared = registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&name)
                .map(Arc::clone);
            match shared {
                Some(shared) => {
                    // PARITY with the POSIX arm (the `reject_zero_depth`
                    // precedent — ONE rule, both arms). Structurally
                    // unreachable HERE: `create_owned` runs `reinit` before it
                    // inserts, so a registry entry is armed by the time any
                    // peer can see it, and there is no torn create to leave a
                    // bare one. Kept so a future stub change cannot diverge
                    // from the platform this ships on.
                    reject_unarmed(&name, shared.epoch())?;
                    Ok(Self {
                        shared,
                        name,
                        owns_name: false,
                    })
                }
                None => Err(io::Error::new(
                    io::ErrorKind::NotFound,
                    format!(
                        "credit segment {name:?} does not exist (owner must create_owned first)"
                    ),
                )),
            }
        }

        /// The shared [`CreditShared`]'s address — a test/inspection accessor.
        pub fn addr(&self) -> *const CreditShared {
            Arc::as_ptr(&self.shared)
        }
    }

    impl std::ops::Deref for MappedCredit {
        type Target = CreditShared;
        fn deref(&self) -> &CreditShared {
            &self.shared
        }
    }

    /// Mirrors the POSIX arm's rendering, so a diagnostic reads the same
    /// whichever platform produced it.
    impl std::fmt::Debug for MappedCredit {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("MappedCredit")
                .field("name", &self.name)
                .field("owns_name", &self.owns_name)
                .field("outstanding", &self.shared.outstanding())
                .field("depth", &self.shared.depth())
                .field("epoch", &self.shared.epoch())
                .finish()
        }
    }

    impl Drop for MappedCredit {
        fn drop(&mut self) {
            if self.owns_name {
                // The "unlink": remove the name ONLY if the stored Arc is still
                // THIS handle's segment (`Arc::ptr_eq`), so a re-created owner's
                // fresh entry isn't clobbered by a stale owner's drop. Open peer
                // handles keep their own Arc clone, so their `Deref` stays valid
                // after the name is removed (mirrors POSIX unlink ≠ unmap).
                let mut reg = registry().lock().unwrap_or_else(|e| e.into_inner());
                let is_mine = reg
                    .get(&self.name)
                    .is_some_and(|stored| Arc::ptr_eq(stored, &self.shared));
                if is_mine {
                    reg.remove(&self.name);
                }
            }
        }
    }
}

pub use imp::MappedCredit;

/// The ONE handle every `block` mirror site holds — a credit word
/// that is either process-LOCAL (a heap [`CreditShared`] both ends of a
/// co-located edge share) or MAPPED (a [`MappedCredit`] SHM page two PROCESSES
/// share).
///
/// # Why one handle and not a parallel mapped vec
///
/// Before this type the mirror was a bare `Arc<AtomicU64>` at five sites — the
/// publisher's `block_outstanding`, the subscriber's `BlockProbe`, the
/// runtime's `BlockWiring` / `BlockDeferEdge` / `BlockCreditEdge`. Adding a
/// sibling `Option<Arc<MappedCredit>>` beside each would write the increment,
/// the saturating decrement, the defer read AND the slot-debt
/// arithmetic TWICE — and the two copies would then be free to disagree about
/// the one number the lossless-backpressure guarantee rests on. With one handle
/// there is one implementation and the local/mapped split is a `match` inside
/// [`Deref`](std::ops::Deref), invisible to every call site.
///
/// # The cost on a LOCAL edge
///
/// A local edge used to hold a raw `AtomicU64`; it now holds a
/// [`CreditShared`], whose `record_drained` additionally bumps the `wake_seq`
/// epoch (`Release` RMW), issues one `SeqCst` fence, and reads the `parked`
/// mask (`Relaxed`) — the syscall itself is gated on that mask, and a local
/// word can never have a parker, so no syscall is ever made. So a local drain
/// pays one extra `Release` RMW, one `SeqCst` FENCE and one relaxed load. The
/// fence is not free on either target: on x86-64 it lowers to `mfence` (or a
/// locked stack op), on aarch64 to `dmb ish` — a store-buffer drain, tens of
/// cycles when the buffer is dirty. What it is NOT is a syscall, an
/// allocation, or a lock: the handle is cloned only at graph-build time, and
/// the publish path pays nothing new (a `fetch_add` either way). That price
/// buys ONE arithmetic across the local and mapped arms, which is the trade
/// this type is designed to make; a `block` drain is per-frame on a lossless
/// edge, not per-sample on every topic.
///
/// # Determinism
///
/// Unchanged from the raw atomic: the word is a WHEN gate on tick ENTRY, never
/// read as data and never fed to a node body — see the [module docs](self).
#[derive(Debug, Clone)]
pub enum CreditWord {
    /// Both ends of the edge live in THIS process: a heap [`CreditShared`]
    /// shared by the publisher, the subscriber and the producer's pre-fire.
    Local(std::sync::Arc<CreditShared>),
    /// The ends live in DIFFERENT processes: the supervisor-created SHM page
    /// each side mapped. Held as an `Arc` because one worker's mapping is
    /// shared by every site on ITS side of the edge (a producer's pre-fire and
    /// its publisher; a consumer's probe and its credit record).
    Mapped(std::sync::Arc<MappedCredit>),
}

impl CreditWord {
    /// Mint a process-LOCAL word for an edge whose declared depth is `depth`.
    ///
    /// The depth is stamped for diagnostics and for parity with the mapped arm
    /// — the defer GATE reads the wiring's own `threshold`, unchanged from
    /// before this type, so a local edge's behaviour is byte-identical.
    pub fn local(depth: u32) -> Self {
        Self::Local(std::sync::Arc::new(CreditShared::new(depth)))
    }

    /// Wrap an OPENED cross-process word.
    ///
    /// The named constructor exists so a caller that has a `MappedCredit`
    /// never has to name the variant — the two arms are an implementation
    /// detail of this module, and a call site that matched on them would be
    /// the second implementation of the split this type exists to have only
    /// once.
    pub fn mapped(word: std::sync::Arc<MappedCredit>) -> Self {
        Self::Mapped(word)
    }

    /// Is this word backed by a cross-process SHM page? Diagnostic only
    /// (Principle #3) — never a control input; every operation is identical on
    /// both arms.
    pub fn is_mapped(&self) -> bool {
        matches!(self, Self::Mapped(_))
    }
}

impl std::ops::Deref for CreditWord {
    type Target = CreditShared;
    fn deref(&self) -> &CreditShared {
        match self {
            Self::Local(shared) => shared,
            Self::Mapped(mapped) => mapped,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // The beyond-mask degrade pin captures a production `warn!`.
    use tracing_test::traced_test;

    #[test]
    fn layout_offsets_and_size_are_pinned() {
        // The const-asserts above already fail the BUILD on drift; this makes
        // the same facts readable in a test run and pins the mapped region.
        assert_eq!(core::mem::size_of::<CreditShared>(), 24);
        assert!(core::mem::align_of::<CreditShared>() >= 8);
        assert_eq!(core::mem::offset_of!(CreditShared, outstanding), 0);
        assert_eq!(core::mem::offset_of!(CreditShared, depth), 8);
        assert_eq!(core::mem::offset_of!(CreditShared, wake_seq), 12);
        assert_eq!(core::mem::offset_of!(CreditShared, parked), 16);
        assert_eq!(core::mem::offset_of!(CreditShared, epoch), 20);
        assert_eq!(CREDIT_BYTES, 64);
    }

    /// Both per-OS name shapes share the `/cer_crd_` family prefix, which is
    /// PREFIX-FREE against every other SHM family this crate mints.
    #[test]
    fn name_is_deterministic_and_prefix_free() {
        let n1 = credit_shm_name("ns", "id");
        let n2 = credit_shm_name("ns", "id");
        assert_eq!(n1, n2, "same (ns,id) → identical name");
        assert!(
            n1.starts_with("/cer_crd_"),
            "the name must carry the cer_crd family prefix: {n1}"
        );
        for other in ["/cer_bar_", "/cer_rg_", "/cer_db_", "/cer_sta_"] {
            assert!(
                !n1.starts_with(other),
                "the credit family prefix must not alias {other}: {n1}"
            );
        }
    }

    /// The Linux (verbose) shape oracle — `ns` verbatim, `id` FNV-hashed. Driven
    /// through the helper directly so it pins on EVERY OS.
    #[test]
    fn name_oracle_fixed_fnv_verbose() {
        // FNV-1a-64 by hand over b"id": start 0xcbf29ce484222325.
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in b"id" {
            h ^= *b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        assert_eq!(
            credit_shm_name_verbose("dep", "id"),
            format!("/cer_crd_dep_{h:016x}")
        );
    }

    /// The compact shape fits macOS's `PSHMNAMLEN` (31 incl. the slash) for ANY
    /// input, and hashes BOTH components.
    #[test]
    fn name_compact_fits_macos_pshmnamlen_for_any_input() {
        let huge_ns = "n".repeat(4096);
        let huge_id = "i".repeat(4096);
        for (ns, id) in [
            ("", ""),
            (
                "cerdep_perception_9f2c1ab0",
                "credit\u{1f}/very/long/topic\u{1f}sink\u{1f}inp",
            ),
            (huge_ns.as_str(), huge_id.as_str()),
        ] {
            let name = credit_shm_name_compact(ns, id);
            assert_eq!(name.len(), 25, "compact name is fixed-width: {name}");
            assert!(name.len() <= 31, "must fit PSHMNAMLEN: {name}");
        }
    }

    /// The 0x1F unit separator makes the `(ns, id)` split unambiguous, so
    /// `("ab","c")` can never alias `("a","bc")`.
    #[test]
    fn name_compact_separator_prevents_boundary_aliasing() {
        assert_ne!(
            credit_shm_name_compact("ab", "c"),
            credit_shm_name_compact("a", "bc")
        );
    }

    /// Different `ns` (a different deployment) and different `id` (a different
    /// edge) both yield different names — the tenant/edge partition.
    #[test]
    fn name_partitions_by_both_ns_and_id() {
        assert_ne!(credit_shm_name("a", "e"), credit_shm_name("b", "e"));
        assert_ne!(credit_shm_name("a", "e"), credit_shm_name("a", "f"));
    }

    /// `record_published_n` is the `n`-at-a-time increment, and
    /// `record_published` delegates to it — ONE increment implementation.
    ///
    /// The returned value is the count the call CREATED, against a hand oracle
    /// rather than against a later read (which would be a different
    /// instant, and is exactly what the RMW's own result exists to avoid).
    #[test]
    fn record_published_n_is_the_one_increment_and_returns_the_count_it_created() {
        let w = CreditShared::new(8);
        assert_eq!(w.record_published_n(0), 0, "a zero increment is a no-op");
        assert_eq!(w.record_published(), 1, "one publish creates count 1");
        assert_eq!(w.record_published_n(3), 4, "three more create count 4");
        assert_eq!(w.record_published_n(2), 6);
        assert_eq!(w.outstanding(), 6, "and the word holds what was returned");
        // The `n` form is the slot-debt credit-back, so it has to be
        // exactly reversible by the drain it pairs with.
        w.record_drained(6);
        assert_eq!(w.outstanding(), 0);
    }

    /// A LOCAL [`CreditWord`] and a bare [`CreditShared`] are the same word: the
    /// enum is a handle, not a second implementation.
    ///
    /// The oracle is the FULL operation set the block mirror uses — the defer
    /// read, the publish increment, the `n` credit-back and the saturating
    /// drain — because the whole reason this type exists is that those four
    /// have exactly one implementation between the local and mapped arms.
    #[test]
    fn a_local_credit_word_is_the_shared_word_under_every_mirror_operation() {
        let w = CreditWord::local(2);
        assert!(!w.is_mapped(), "a local word is not a mapped one");
        assert_eq!((w.depth(), w.outstanding()), (2, 0));
        assert!(w.try_claim(), "an empty depth-2 word has room");
        assert_eq!(w.record_published(), 1);
        assert!(w.try_claim(), "one of two slots taken still leaves room");
        assert_eq!(w.record_published(), 2);
        assert!(!w.try_claim(), "at depth the producer must defer");
        assert!(w.is_full());
        w.record_drained(1);
        assert!(w.try_claim(), "a drain frees exactly one slot");
        // Saturating, not wrapping: an over-drain floors at zero rather than
        // wrapping to u64::MAX and deferring the producer forever.
        w.record_drained(99);
        assert_eq!(w.outstanding(), 0);
    }

    /// Two clones of one LOCAL word are one word — the property every wiring
    /// site depends on (the publisher's increment and the subscriber's drain
    /// hold separate clones of the same handle).
    #[test]
    fn cloned_credit_word_handles_share_one_word() {
        let a = CreditWord::local(4);
        let b = a.clone();
        a.record_published_n(3);
        assert_eq!(
            b.outstanding(),
            3,
            "the clone sees the original's publishes"
        );
        b.record_drained(3);
        assert_eq!(
            a.outstanding(),
            0,
            "and the original sees the clone's drain"
        );
    }

    /// A drain RINGS the wake epoch and an idle drain does not — the property
    /// the cross-process park rests on, and the one a raw `fetch_sub` at a
    /// decrement site would silently drop.
    #[test]
    fn a_freeing_drain_rings_the_wake_epoch_and_an_idle_one_does_not() {
        let w = CreditWord::local(4);
        w.record_published_n(2);
        let before = w.wake_seq_snapshot();
        w.record_drained(0);
        assert_eq!(
            w.wake_seq_snapshot(),
            before,
            "a zero drain frees nothing, so it must not tax a parked producer with a wake"
        );
        w.record_drained(1);
        assert!(
            w.wake_seq_snapshot() > before,
            "a drain that really freed a slot must ring the wake word"
        );
    }

    /// The edge id is the three-component key, separator-delimited, and the
    /// REACHABLE boundary shift cannot alias.
    ///
    /// The final arm records the residual the separator does NOT close — a
    /// component carrying a LITERAL 0x1F — as a fact rather than leaving it to
    /// be rediscovered. It is asserted EQUAL on purpose: writing it as `assert_ne!`
    /// is how this test first shipped, and it failed, which is what put the
    /// scope paragraph on `credit_edge_id`.
    #[test]
    fn edge_id_oracle_and_boundary_safety() {
        assert_eq!(
            credit_edge_id("/scan", "planner", "cloud"),
            "credit\u{1f}/scan\u{1f}planner\u{1f}cloud"
        );
        // THE reachable hazard: a shift of the split point between adjacent
        // components, over ordinary (separator-free) names.
        assert_ne!(
            credit_edge_id("/a/b", "c", "d"),
            credit_edge_id("/a", "b/c", "d"),
            "a boundary shift between topic and consumer must not alias"
        );
        assert_ne!(
            credit_edge_id("/scan", "plan", "ner_in"),
            credit_edge_id("/scan", "planner", "_in"),
            "a boundary shift between consumer and input must not alias"
        );
        // Distinct inputs on ONE consumer are distinct edges (the multi-consumer
        // block topic the per-edge granularity exists for).
        assert_ne!(
            credit_edge_id("/scan", "planner", "a"),
            credit_edge_id("/scan", "planner", "b")
        );
        // The RESIDUAL, pinned as a known equality: a literal separator inside a
        // component IS ambiguous. Unreachable in practice (see the fn docs); if
        // a component ever becomes free-form this must be length-prefixed.
        assert_eq!(
            credit_edge_id("a\u{1f}b", "c", "d"),
            credit_edge_id("a", "b", "c\u{1f}d"),
            "documented residual: the recipe does not escape a literal 0x1F"
        );
    }

    /// The credit plane's TWO kill switches resolve over
    /// the SHARED grammar, and each warns under ITS OWN name.
    ///
    /// The grammar is `crate::kill_switch`'s: `0` disables SILENTLY, unset and `1`
    /// enable silently, anything else warns and stays ON. Fail-safe in that
    /// direction on purpose — a typo must not silently disable a latency plane,
    /// because the result is correct output at collapsed latency, which nothing
    /// downstream can notice.
    ///
    /// The per-consumer wrapper rule (`kill_switch.rs`) exists because these two
    /// switches and the barrier's third one share one parser: a single wrapper
    /// would warn under one variable's name no matter which the operator
    /// actually set, and an operator who greps for the name they typed would
    /// find nothing. So the ONE assertion that matters here is WHICH name each
    /// warn carries, and each is driven separately.
    #[test]
    #[traced_test]
    fn the_credit_kill_switches_resolve_over_the_shared_grammar() {
        // `0` disables, silently.
        assert!(resolve_credit_wake_disabled(Some("0")));
        // Everything that means ON, silently.
        for on in [None, Some("1")] {
            assert!(
                !resolve_credit_wake_disabled(on),
                "{on:?} keeps the plane on"
            );
        }
        assert!(
            !logs_contain(CREDIT_WAKE_ENV),
            "a valid value must resolve SILENTLY — a warn on the healthy path is \
             the flood this repo's latches exist to prevent"
        );
        // Garbage warns and stays ON.
        assert!(
            !resolve_credit_wake_disabled(Some("yes")),
            "an unrecognized value must FAIL SAFE (stay on), never disable"
        );
        assert!(
            logs_contain(CREDIT_WAKE_ENV),
            "and the warn must name CERULION_CREDIT_WAKE — the variable the \
             operator actually set"
        );
        assert!(
            !logs_contain(CREDIT_OS_SYNC_ENV),
            "and must NOT name the sibling switch: naming the wrong variable is \
             exactly the drift the per-consumer wrapper rule forbids"
        );
    }

    /// The macOS os_sync switch, driven separately so its warn's name is
    /// asserted against a log that the sibling switch has not written to.
    #[cfg(target_os = "macos")]
    #[test]
    #[traced_test]
    fn the_credit_os_sync_switch_warns_under_its_own_name() {
        assert!(resolve_credit_os_sync_disabled(Some("0")));
        assert!(!resolve_credit_os_sync_disabled(None));
        assert!(
            !logs_contain(CREDIT_OS_SYNC_ENV),
            "valid values resolve silently"
        );
        assert!(
            !resolve_credit_os_sync_disabled(Some("maybe")),
            "garbage fails safe"
        );
        assert!(
            logs_contain(CREDIT_OS_SYNC_ENV),
            "the warn names CERULION_CREDIT_OS_SYNC"
        );
        assert!(
            !logs_contain("CERULION_BARRIER_OS_SYNC"),
            "and never the BARRIER's switch — the credit plane's tier decision \
             is its own, which is the whole reason this variable exists rather \
             than the credit park delegating to the barrier's gate"
        );
    }

    /// [`ProducerSlot`] is a lossless, ordered newtype
    /// over its edge-local index, and [`PARKED_MASK_BITS`] is the ONE place the
    /// mask width is written.
    ///
    /// # What this pins, and what the COMPILER pins
    ///
    /// The type's whole reason to exist — that a global rank can no longer be
    /// passed where an edge-local slot is wanted — is a COMPILE-TIME property,
    /// so no runtime assertion can carry it; the oracle for that is the
    /// supervisor sweep site (`graph_cmd.rs`, `clear_parked_producer(slot)`),
    /// where handing it the `u32` `dead_rank` sitting in the same scope no
    /// longer type-checks. That is the mutation this type is verified against.
    ///
    /// What IS testable, and is here: the round trip loses nothing (a slot
    /// wrapping index `i` reports `i`), `fits_mask` agrees with the mask width
    /// on BOTH sides of the boundary rather than against a literal 32, and the
    /// ordering derive is by index (the supervisor lists slots in a stable
    /// order, and an `Ord` derived over a different field would reorder its
    /// output silently).
    #[test]
    fn producer_slot_is_a_lossless_ordered_index_and_the_mask_width_is_named_once() {
        assert_eq!(PARKED_MASK_BITS, 32, "the mask is a u32");
        for i in [0u32, 1, 5, 31, 32, 63, u32::MAX] {
            assert_eq!(ProducerSlot::new(i).index(), i, "round trip is lossless");
        }
        // `fits_mask` is the boundary, pinned on BOTH sides and derived from
        // the named width — not from a literal that could drift away from it.
        assert!(ProducerSlot::new(PARKED_MASK_BITS - 1).fits_mask());
        assert!(!ProducerSlot::new(PARKED_MASK_BITS).fits_mask());
        assert!(!ProducerSlot::new(u32::MAX).fits_mask());
        assert!(
            ProducerSlot::new(0) < ProducerSlot::new(1),
            "ordering is by index"
        );
        assert_eq!(ProducerSlot::new(7), ProducerSlot::new(7));
        assert_ne!(ProducerSlot::new(7), ProducerSlot::new(8));
    }

    /// The parked bitmask is per-EDGE-producer-index: set / exit / sweep touch
    /// exactly their own bit and nothing else.
    ///
    /// Deliberately drives NO beyond-mask `park_enter` — that whole arm lives in
    /// the traced test below, because `park_enter`'s `WARNED` latch is
    /// PROCESS-global and libtest runs this binary's tests in one process: a
    /// sibling tripping the latch first makes the `logs_contain` assertion
    /// vacuously false. (It failed exactly that way when the two shared the
    /// coverage.) `clear_parked_producer` stays here — it warns nothing.
    #[test]
    fn parked_bitmask_oracle() {
        let c = CreditShared::new(4);
        assert_eq!(c.parked_mask(), 0);
        assert!(c.park_enter(ProducerSlot::new(0)));
        assert!(c.park_enter(ProducerSlot::new(5)));
        assert_eq!(c.parked_mask(), 0b10_0001, "bits 0 and 5 set");
        // THE ACCEPT-SIDE BOUNDARY: index 31 is the LAST in-mask producer, so
        // the refusal must be `>= 32` and not `>= 31`. Without this line a
        // boundary shifted by one is invisible — every other index driven here
        // is well inside the mask, and the beyond-mask arm below only probes
        // 32/40/63, which BOTH spellings refuse.
        assert!(
            c.park_enter(ProducerSlot::new(31)),
            "index 31 is the last in-mask producer and must be ACCEPTED"
        );
        assert_eq!(
            c.parked_mask(),
            0b1000_0000_0000_0000_0000_0000_0010_0001,
            "bit 31 set alongside 0 and 5"
        );
        c.park_exit(ProducerSlot::new(31));
        assert_eq!(c.parked_mask(), 0b10_0001, "and released again");
        c.park_exit(ProducerSlot::new(0));
        assert_eq!(
            c.parked_mask(),
            0b10_0000,
            "exit clears exactly its own bit"
        );
        // Supervisor sweep: clears exactly the dead producer; idempotent;
        // unset-bit and beyond-mask indices are no-ops.
        c.clear_parked_producer(ProducerSlot::new(5));
        assert_eq!(c.parked_mask(), 0, "sweep clears the dead producer's bit");
        c.clear_parked_producer(ProducerSlot::new(5));
        c.clear_parked_producer(ProducerSlot::new(31));
        c.clear_parked_producer(ProducerSlot::new(40));
        assert_eq!(
            c.parked_mask(),
            0,
            "sweeps of unset/beyond-mask indices are no-ops"
        );
        // The RAII guard holds its bit for its life and clears it on drop.
        {
            let _g = ParkedEdgeGuard::enter(&c, ProducerSlot::new(7)).expect("index 7 is in-mask");
            assert_eq!(c.parked_mask(), 1 << 7);
        }
        assert_eq!(c.parked_mask(), 0, "guard drop cleared the bit");
    }

    /// A beyond-mask producer index sets NO bit, yields NO guard, and the
    /// degrade is LOUD (once per process).
    ///
    /// The ONLY test in this binary that drives a beyond-mask `park_enter`, so
    /// the process-global `WARNED` latch is guaranteed un-tripped when the
    /// capture is armed — see `parked_bitmask_oracle`'s note.
    #[traced_test]
    #[test]
    fn a_beyond_mask_producer_index_is_bit_less_and_warns_loudly() {
        let c = CreditShared::new(4);
        assert!(
            !c.park_enter(ProducerSlot::new(32)),
            "index 32 exceeds the bitmask — no bit"
        );
        assert!(
            logs_contain("exceeds the per-edge parked bitmask"),
            "the beyond-mask degrade must warn loudly"
        );
        assert!(!c.park_enter(ProducerSlot::new(63)));
        assert_eq!(
            c.parked_mask(),
            0,
            "beyond-mask indices never touch the mask"
        );
        assert!(
            ParkedEdgeGuard::enter(&c, ProducerSlot::new(40)).is_none(),
            "beyond-mask index yields no guard (caller keeps timeout cadence)"
        );
        assert_eq!(c.parked_mask(), 0);
    }

    /// `reinit` zeroes every observation and BUMPS the epoch — a re-armed edge is
    /// distinguishable from a fresh one, which is what a dead-consumer release
    /// has to be able to see.
    #[test]
    fn reinit_resets_state_and_bumps_the_epoch() {
        let c = CreditShared::new(2);
        assert_eq!(c.epoch(), 0, "a bare `new` has not been armed");
        c.record_published();
        c.record_published();
        assert!(c.park_enter(ProducerSlot::new(3)));
        assert!(c.wake_seq_snapshot() == 0, "publishing frees no credit");
        c.record_drained(1);
        assert!(c.wake_seq_snapshot() > 0);

        c.reinit(5);
        assert_eq!(c.outstanding(), 0, "reinit clears the outstanding count");
        assert_eq!(c.depth(), 5, "reinit stamps the new depth");
        assert_eq!(c.wake_seq_snapshot(), 0, "reinit re-zeros the epoch word");
        assert_eq!(c.parked_mask(), 0, "reinit sweeps the whole parked mask");
        assert_eq!(c.epoch(), 1, "reinit BUMPS the re-arm epoch");
        c.reinit(5);
        assert_eq!(c.epoch(), 2, "each re-arm is distinguishable");
    }

    /// The wake epoch is bumped by EXACTLY the sites that free credit, and by
    /// nothing else — a read never rings the word, and neither does a publish or
    /// an empty drain. This is the "atomics-only when nobody parks" contract
    /// stated where it is hermetically checkable (the SYSCALL is gated on the
    /// mask, which is 0 throughout here).
    #[test]
    fn wake_seq_bump_site_oracle() {
        let c = CreditShared::new(4);
        assert_eq!(c.parked_mask(), 0, "nobody parks — no syscall is reachable");

        let before = c.wake_seq_snapshot();
        assert!(c.try_claim());
        assert!(!c.is_full());
        let _ = c.outstanding();
        assert_eq!(
            c.wake_seq_snapshot(),
            before,
            "a READ never bumps the epoch"
        );

        c.record_published();
        assert_eq!(
            c.wake_seq_snapshot(),
            before,
            "a publish frees no credit and must not bump"
        );

        c.record_drained(0);
        assert_eq!(
            c.wake_seq_snapshot(),
            before,
            "an empty drain frees no credit and must not bump"
        );

        c.record_drained(1);
        assert_eq!(
            c.wake_seq_snapshot(),
            before.wrapping_add(1),
            "one freeing drain bumps exactly once"
        );
        c.record_drained(3);
        assert_eq!(
            c.wake_seq_snapshot(),
            before.wrapping_add(2),
            "the bump is per CALL, not per frame"
        );

        // The explicit ring is available to outside callers and bumps too.
        c.note_credit_freed();
        assert_eq!(c.wake_seq_snapshot(), before.wrapping_add(3));
    }

    /// A `depth == 0` create is REFUSED, not stamped. A zero-depth word reads
    /// full unconditionally, so its producer defers forever with no drain able
    /// to recover it — the refusal is the only thing standing between a
    /// plan-stamping slip and a silently wedged edge.
    ///
    /// The oracle is the ERROR KIND plus the two things the message has to name
    /// (the consequence, and WHERE the bug is), not a bare `is_err()`: this
    /// value can only arrive from the plan's own stamping, so a message that
    /// blames the operator's graph sends the reader to a file that is fine.
    #[test]
    fn a_zero_depth_create_is_refused_rather_than_wedging_its_producer() {
        let ns = format!("c2a_zero_depth_{}", std::process::id());
        let err = MappedCredit::create_owned(&ns, "e", 0)
            .expect_err("depth 0 must be refused at the create seam");
        assert_eq!(
            err.kind(),
            std::io::ErrorKind::InvalidInput,
            "a zero depth is a bad ARGUMENT, not an I/O failure: {err}"
        );
        let text = err.to_string();
        assert!(
            text.contains("defer entry to EVERY tick forever"),
            "the refusal must name the CONSEQUENCE (a permanent defer): {text}"
        );
        assert!(
            text.contains("GraphTopology::validate") && text.contains("plan-stamping bug"),
            "the refusal must name where the bug IS, not blame the graph: {text}"
        );
        // Anti-tautology: depth 1 (the smallest legal ceiling) still creates.
        let ok = MappedCredit::create_owned(&ns, "e", 1).expect("depth 1 is legal");
        assert_eq!(ok.depth(), 1);
        assert!(ok.try_claim(), "and the legal word is claimable");
    }

    /// A named-but-SHORT segment is refused rather than mapped. `create_owned`
    /// does `shm_open` THEN `ftruncate`, so an owner that died between the two
    /// leaves a ZERO-LENGTH object under a perfectly good name.
    ///
    /// The hazard is built as ZERO-LENGTH deliberately: an 8-byte truncate
    /// would not work, because on macOS a POSIX SHM object is PAGE-GRANULAR,
    /// so `fstat` would report 4096 and the open would correctly succeed.
    /// A zero-length object is the shape that is short on
    /// EVERY platform, and it is also the only one that is genuinely dangerous:
    /// a sub-page truncate still allocates page 0, so the struct fits.
    ///
    /// Lives in the crate's own `mod tests` (not `tests/credit_test.rs`) because
    /// building the hazard requires the PRIVATE `credit_shm_name` — the harness
    /// has to create the object under exactly the name `open_unowned` derives.
    #[cfg(unix)]
    #[test]
    fn open_refuses_a_short_segment_rather_than_mapping_past_its_end() {
        use std::ffi::CString;
        let ns = format!("c2a_short_{}", std::process::id());
        let id = "shortseg";
        let name = CString::new(credit_shm_name(&ns, id)).expect("name has no NUL");

        // Hand-build the crash shape: the NAME exists and the object was never
        // sized (no `ftruncate`) — exactly an owner that died between the two
        // calls. Unlink first so a previous failed run cannot make the O_EXCL
        // create fail for an unrelated reason.
        // SAFETY: FFI over a name this test derived; result deliberately ignored.
        unsafe { libc::shm_unlink(name.as_ptr()) };
        // SAFETY: FFI create of a named SHM object; `name` is a valid C string.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "harness could not create the short segment: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: `fd` is the descriptor just created.
        unsafe { libc::close(fd) };

        let outcome = MappedCredit::open_unowned(&ns, id);
        // Clean up BEFORE asserting, so a failure cannot also leak the object.
        // SAFETY: FFI unlink of the name this test created.
        unsafe { libc::shm_unlink(name.as_ptr()) };

        let err = outcome.expect_err("a short segment must be refused, never mapped");
        // The KIND is the load-bearing assertion: without the check, Linux
        // maps it and SIGBUSes on first touch (no error at all), while macOS
        // surfaces mmap's own errno — neither of which is `InvalidData`.
        assert_eq!(
            err.kind(),
            std::io::ErrorKind::InvalidData,
            "the refusal must be `this object is the wrong shape`, not NotFound and not a raw \
             mmap errno: {err}"
        );
        let text = err.to_string();
        assert!(
            text.contains("short of the") && text.contains("SIGBUS"),
            "the refusal must name the shape problem AND what mapping it would have cost: {text}"
        );
    }
}

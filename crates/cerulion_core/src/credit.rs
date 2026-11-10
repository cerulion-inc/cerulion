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
//! load / `fetch_add` / `try_update` / store), so a crashed peer can never
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
//! # When the count is exact, and where it stops
//!
//! A producer counts a frame on the word BEFORE the frame becomes visible:
//! every send path of `CerulionPublisher` counts on each block word, then
//! sends, and takes the count back when the send errs
//! ([`CreditShared::record_published_with_undo`],
//! [`CreditShared::record_unpublished`]).
//! iceoryx2 publishes a frame to a consumer with a `Release` store inside
//! `send()`, the consumer's pop is an `Acquire` load of it and its drain comes
//! after the pop, so a drain never finds the word without the count of a frame
//! it removes, however the two processes are scheduled.
//!
//! The take-back is exact under iceoryx2's default safe overflow (on) and
//! default degradation action (`Warn`): with both, a `send()` that returns
//! `Err` delivered the frame to no connection. Cerulion sets neither. No
//! publisher sets a degradation handler, and safe overflow comes from the
//! iceoryx2 configuration the process loads, which an operator `iceoryx2.toml`
//! can change: with `enable-safe-overflow = false`, a `send()` to several
//! consumers can return `Err` after some of their queues took the frame, and
//! the take-back then leaves each of those words one frame below its queue.
//! `credit_block_iox2_test::the_block_publisher_keeps_safe_overflow_and_the_default_degradation`
//! reads safe overflow from a block topic's service built under the test
//! configuration, which carries iceoryx2's compiled-in defaults, and scans the
//! workspace for a degradation handler, so it pins those defaults and no
//! operator file.
//!
//! What this does NOT cover. Each case leaves the word off by the frames
//! involved for as long as the edge runs:
//!
//! - **Frames the word never counts.** Native history delivered to a late
//!   joiner (graph build warns about a `block` topic with `history_size > 0`),
//!   and a writer outside the graph on a `multi_publisher_topics` topic. Their
//!   drains leave the word BELOW the queue, so the producer can overrun the
//!   declared depth. A drain that removes more frames than the word counts
//!   clips the word at 0, and is reported once per regime and counted by the
//!   consumer (`CerulionSubscriber::block_drains_clipped`); a drain the word's
//!   count still covers is indistinguishable from a counted drain.
//! - **Pops that record no drain.** `CerulionSubscriber::try_receive_one`,
//!   `try_receive_one_owned` and `spin_view_until_seq` take from the queue
//!   without touching the word, so a `block` input read through them leaves
//!   the word ABOVE the queue.
//! - **A send that returns `Ok` without reaching this consumer's queue**, such
//!   as one to a consumer whose port connected after the send updated its
//!   connections: the frame stays counted and no drain ever pays for it.
//! - **Several in-graph producers across processes on one edge.** Graph
//!   planning refuses that shape (`CreditBar::MultipleProducers`) rather than
//!   counting it.
//!
//! One window is transient rather than lasting: a frame moving from the queue
//! into a per-set Sync matcher slot is drained at its pop and credited back as
//! held by the slot re-derivation just after, so between the two the word reads
//! one frame low. A publish admitted in that instant cannot evict a counted
//! frame, because the frame being moved has already left the queue.
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
//! **Per-consumer slot release.** Each word is ONE block consumer's slot
//! (`credit_edge_id` keys it by topic, consumer node and consumer input), so a
//! release acts on that consumer alone and leaves every other consumer of the
//! topic, and every other topic, as it was. Two releases exist:
//!
//! - [`CreditShared::release_slot`] is STICKY: it marks the slot released,
//!   zeroes every count and rings the wake word. From then on
//!   [`CreditShared::is_full`] is `false` and [`CreditShared::record_published`]
//!   no longer counts, so the producer stops deferring on this consumer for
//!   the rest of the run. The supervisor calls it when the consumer's process
//!   dies; the runtime calls it when the consumer node stops with a fatal
//!   error.
//! - [`CreditShared::forgive_outstanding`] moves every counted frame into the
//!   slot's forgiven backlog and rings the wake word without marking the
//!   slot. The producer gets `depth` more publishes before the slot can defer
//!   it again, and a publish that overflows the consumer's queue evicts a
//!   forgiven frame. The consumer's later drains pay the backlog off before
//!   any counted frame, so draining a forgiven frame never frees the credit
//!   of a frame published after the release, and a forgiven frame the
//!   consumer's matcher pops into a slot stays forgiven. On a local word, a
//!   drain that finds the consumer's queue empty also zeroes the queued count
//!   and the backlog ([`CreditWord::record_drained_to_empty`]), which clears
//!   what the backlog bound misses on a word with several connections. The
//!   `block_timeout_ms` release uses it. Graph load refuses that key on a
//!   topic with several producers and on an edge whose producer runs in
//!   another process or executor unit, and a cross-process binding carries no
//!   timeout, so a graph forgives only a local word with one connection,
//!   whose producer and consumer fire one at a time on one thread.
//!
//! Whoever releases emits the loud event; this module only moves the word.
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
    /// The slot's four frame counts, packed so that every transition is one
    /// RMW on one `u64` (`Counts` names them):
    ///
    /// - bits 0..32, `queued`: frames published and not yet drained, less the
    ///   ones a timeout release forgave. The shared mirror of the consumer's
    ///   iceoryx2 queue depth: the producer bumps it on every successful send,
    ///   the consumer's drain takes it down (saturating).
    /// - bits 32..48, `backlog`: frames [`Self::forgive_outstanding`] forgave
    ///   that are still in the consumer's queue.
    /// - bits 48..56, `held`: frames the consumer's matcher holds in its slots
    ///   ([`Self::record_slot_held`]), counted against `depth`.
    /// - bits 56..64, `held_forgiven`: held frames a release forgave, either
    ///   in a slot when it ran or popped from the backlog into a slot after it.
    ///
    /// The producer's gate compares `queued + held` with `depth`
    /// ([`Self::outstanding`]).
    ///
    /// # Memory ordering
    ///
    /// Every operation is ONE `try_update` (or `store`) on this word, so a
    /// reader sees the four counts of one instant, and the publish (the
    /// producer), the drain and the slot moves (the consumer) and the
    /// releases (the producer, or the supervisor) apply in the word's single
    /// modification order: none can land inside another. Writes are `Release`
    /// (the timeout release `AcqRel`) and every read `Acquire`, so the
    /// producer's gate read observes every drain that happened before it. The
    /// local word (one heap value shared by a co-located edge's consumer and
    /// any number of producer threads) and the mapped page (two processes) run
    /// this same arithmetic.
    counts: AtomicU64,
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
    /// Sticky slot release: `0` while the consumer holds the slot, non-zero
    /// once [`Self::release_slot`] gave it up. Read by [`Self::is_full`] and by
    /// the publish increment; set once per run, never cleared on a live page.
    released: AtomicU32,
    /// How many publisher connections feed the consumer's input. iceoryx2
    /// gives each connection its own queue of `depth` frames, so the
    /// consumer's queues hold at most `depth * connections` frames, and a
    /// publish takes the forgiven backlog down to what still fits beside the
    /// counted frames ([`Self::record_published_n`]). Stamped `1` at create;
    /// [`Self::set_connections`] raises it for a topic with several producers.
    connections: AtomicU32,
}

// SHM mapping across processes requires a stable, atomics-only `repr(C)` layout.
// Lock the size so a future non-atomic field (padding / bool / pointer) fails the
// build rather than silently breaking the cross-process mapping.
const _: () = assert!(
    core::mem::size_of::<CreditShared>() == 32,
    "CreditShared layout changed: SHM cross-process mapping requires the 32-byte atomics-only repr(C)"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, counts) == 0,
    "counts must sit at offset 0"
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
    core::mem::offset_of!(CreditShared, released) == 24,
    "released must sit at offset 24, the sticky per-consumer slot release"
);
const _: () = assert!(
    core::mem::offset_of!(CreditShared, connections) == 28,
    "connections must sit at offset 28, the publisher connection count"
);
const _: () = assert!(
    core::mem::align_of::<CreditShared>() >= 8,
    "CreditShared must be >=8-byte aligned so its `counts` field (offset 0) is a valid 8-byte-aligned atomic target"
);
const _: () = assert!(
    core::mem::size_of::<CreditShared>() <= CREDIT_BYTES,
    "CreditShared must fit the region MappedCredit maps"
);

/// The four counts [`CreditShared`]'s first word packs (its `counts` field
/// docs give the bit ranges). Each transition is a pure function from one
/// `Counts` to the next, which the word applies as one RMW.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Counts {
    /// Frames published and not drained, less the forgiven ones.
    queued: u32,
    /// Forgiven frames still in the consumer's queue.
    backlog: u16,
    /// Frames held in the consumer's matcher slots, counted against depth.
    held: u8,
    /// Held frames a release forgave, in a slot or popped from the backlog.
    held_forgiven: u8,
}

/// `n` as a `u32`, saturating: a count past `u32::MAX` is past any depth.
fn saturate_u32(n: u64) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

/// `n` as a `u8`, saturating: a slot count past `u8::MAX` is past the two
/// slots a matcher has.
fn saturate_u8(n: u64) -> u8 {
    u8::try_from(n).unwrap_or(u8::MAX)
}

impl Counts {
    fn unpack(v: u64) -> Self {
        Self {
            queued: (v & 0xFFFF_FFFF) as u32,
            backlog: ((v >> 32) & 0xFFFF) as u16,
            held: ((v >> 48) & 0xFF) as u8,
            held_forgiven: (v >> 56) as u8,
        }
    }

    fn pack(self) -> u64 {
        u64::from(self.queued)
            | (u64::from(self.backlog) << 32)
            | (u64::from(self.held) << 48)
            | (u64::from(self.held_forgiven) << 56)
    }

    /// Frames counted against depth: queued and held, forgiven ones excluded.
    fn outstanding(self) -> u64 {
        u64::from(self.queued) + u64::from(self.held)
    }

    /// Frames a release forgave that the consumer still holds.
    fn forgiven(self) -> u64 {
        u64::from(self.backlog) + u64::from(self.held_forgiven)
    }

    /// `n` frames entered the consumer's queue. Its queues hold `capacity()`
    /// frames; past that the overflow evicts the oldest queued frames, which
    /// are the forgiven ones, so the backlog keeps only what fits beside the
    /// queued count. `capacity` is read only while a backlog exists.
    fn published(self, n: u64, capacity: impl Fn() -> u64) -> Self {
        let queued = self.queued.saturating_add(saturate_u32(n));
        let backlog = if self.backlog == 0 {
            0
        } else {
            let room = capacity().saturating_sub(u64::from(queued));
            // A room past `u16::MAX` exceeds any backlog, which keeps it all.
            u16::try_from(room).map_or(self.backlog, |room| self.backlog.min(room))
        };
        Self {
            queued,
            backlog,
            ..self
        }
    }

    /// A frame [`Self::published`] counted reached no queue: one queued
    /// frame comes off, saturating at zero. The backlog and the held counts
    /// stay.
    fn unpublished(self) -> Self {
        Self {
            queued: self.queued.saturating_sub(1),
            ..self
        }
    }

    /// `removed` frames left the consumer's queue, oldest first: the
    /// forgiven frames go first, then counted ones (saturating at zero).
    fn drained(self, removed: u64) -> Self {
        let paid = if crate::oracle_mutant::active("block_forgiven_drain_spends_credit") {
            0
        } else {
            u16::try_from(removed).map_or(self.backlog, |r| r.min(self.backlog))
        };
        Self {
            backlog: self.backlog - paid,
            queued: self
                .queued
                .saturating_sub(saturate_u32(removed - u64::from(paid))),
            ..self
        }
    }

    /// The consumer's queue was seen empty: no frame is queued, counted or
    /// forgiven. Held frames are out of the queue and keep their counts.
    fn emptied(self) -> Self {
        if crate::oracle_mutant::active("block_empty_resync_off") {
            return self;
        }
        Self {
            queued: 0,
            backlog: 0,
            ..self
        }
    }

    /// Popped frames now sit in the consumer's slots: `counted` of them count
    /// against depth again, and `forgiven` of them were frames of the backlog
    /// a release forgave, which stay forgiven. Each count saturates in its own
    /// field.
    fn slot_held(self, counted: u64, forgiven: u64) -> Self {
        Self {
            held: self.held.saturating_add(saturate_u8(counted)),
            held_forgiven: self.held_forgiven.saturating_add(saturate_u8(forgiven)),
            ..self
        }
    }

    /// `k` frames left the consumer's slots: the forgiven held frames go
    /// first, then counted ones (saturating at zero).
    fn slot_freed(self, k: u64) -> Self {
        let paid = u8::try_from(k).map_or(self.held_forgiven, |k| k.min(self.held_forgiven));
        Self {
            held_forgiven: self.held_forgiven - paid,
            held: self.held.saturating_sub(saturate_u8(k - u64::from(paid))),
            ..self
        }
    }

    /// One held frame left a slot and a frame popped from the queue took its
    /// place. The leaving frame is a forgiven one when a held frame is
    /// forgiven, as [`Self::slot_freed`] pays them first, and the entering
    /// frame is forgiven when `forgiven_entering`. A counted frame for a
    /// counted one, or a forgiven frame for a forgiven one, changes nothing.
    fn slot_replaced(self, forgiven_entering: bool) -> Self {
        let leaving_forgiven = self.held_forgiven != 0;
        if leaving_forgiven == forgiven_entering
            || crate::oracle_mutant::active("block_replaced_head_unpaid")
        {
            return self;
        }
        if forgiven_entering {
            Self {
                held: self.held.saturating_sub(1),
                held_forgiven: self.held_forgiven.saturating_add(1),
                ..self
            }
        } else {
            Self {
                held: self.held.saturating_add(1),
                held_forgiven: self.held_forgiven - 1,
                ..self
            }
        }
    }

    /// The timeout release: every counted frame becomes forgiven. The
    /// backlog is capped at `capacity`, the most the queues can hold.
    fn forgive(self, capacity: u64) -> Self {
        let backlog = (u64::from(self.backlog) + u64::from(self.queued)).min(capacity);
        Self {
            queued: 0,
            backlog: u16::try_from(backlog).unwrap_or(u16::MAX),
            held: 0,
            held_forgiven: self.held_forgiven.saturating_add(self.held),
        }
    }
}

/// The take-back of one count
/// [`CreditShared::record_published_with_undo`] took: the packed counts its
/// RMW replaced and the packed counts it stored.
/// [`CreditShared::record_unpublished`] consumes it when the frame's send
/// reaches no queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PublishUndo {
    replaced: u64,
    stored: u64,
}

/// What one drain RMW did to a credit word, read off the counts it replaced
/// and the counts it stored ([`CreditShared::record_drained`]).
///
/// The drain removes the forgiven backlog first, then queued frames, and
/// saturates at 0, so `removed` frames split three ways: `paid` forgiven
/// frames, the queued frames the drain took off, and `clipped` frames past
/// both. Held frames take no part.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Drained {
    /// Forgiven backlog frames this drain paid: the first frames it removed,
    /// so the last frame removed is a forgiven one only when every removed
    /// frame was. `0` on a word no timeout release forgave.
    pub paid: u64,
    /// Frames removed past the forgiven backlog and the queued count, which
    /// the saturating subtraction could not take off once the word reached 0.
    /// The publisher counts a frame before its send makes it visible, so a
    /// clip means the drain removed frames the word never counted (a late
    /// joiner's history replay, a writer outside the graph on a
    /// `multi_publisher_topics` topic, or a count another drain already
    /// took). The consumer's block probe reports it once per regime
    /// (`CerulionSubscriber::block_drains_clipped`).
    pub clipped: u64,
}

impl Drained {
    /// The split of a drain of `removed` frames whose RMW replaced `prev`
    /// with `next`. In `u64`, so a `removed` past `u32::MAX`, where the queued
    /// subtraction saturates, still clips by the frames it did not take off.
    fn between(removed: u64, prev: Counts, next: Counts) -> Self {
        let paid = u64::from(prev.backlog - next.backlog);
        let taken = u64::from(prev.queued - next.queued);
        Self {
            paid,
            clipped: removed - paid - taken,
        }
    }
}

impl CreditShared {
    /// Create a credit word for a consumer input whose declared depth is
    /// `depth`, with zero frames outstanding.
    ///
    /// `const` so a [`CreditShared`] can be placed in a `static` /
    /// const-initialized region.
    pub const fn new(depth: u32) -> Self {
        Self {
            counts: AtomicU64::new(0),
            depth: AtomicU32::new(depth),
            wake_seq: AtomicU32::new(0),
            parked: AtomicU32::new(0),
            epoch: AtomicU32::new(0),
            released: AtomicU32::new(0),
            connections: AtomicU32::new(1),
        }
    }

    /// Apply `step` to the counts as ONE RMW (`order` on success, `Acquire`
    /// on every load) and return the counts it replaced and the counts it
    /// stored. `step` may run more than once when another side wins the race;
    /// only its last run is stored.
    fn update_counts(
        &self,
        order: Ordering,
        mut step: impl FnMut(Counts) -> Counts,
    ) -> (Counts, Counts) {
        let mut next = Counts::default();
        let prev = match self.counts.try_update(order, Ordering::Acquire, |v| {
            next = step(Counts::unpack(v));
            Some(next.pack())
        }) {
            // `step` always returns a value, so the update always lands: the
            // `Err` arm carries the same previous value and is never taken.
            Ok(v) | Err(v) => v,
        };
        (Counts::unpack(prev), next)
    }

    /// The most frames the consumer's queues hold: `depth * connections`.
    fn queue_capacity(&self) -> u64 {
        u64::from(self.depth.load(Ordering::Acquire))
            .saturating_mul(u64::from(self.connections.load(Ordering::Acquire).max(1)))
    }

    /// Stamp how many publisher connections feed the consumer's input (`0`
    /// stamps `1`). The runtime calls it at wiring time, before the first
    /// publish, with the number of producers of the topic.
    pub fn set_connections(&self, connections: u32) {
        self.connections
            .store(connections.max(1), Ordering::Release);
    }

    /// The publisher connection count [`Self::set_connections`] stamped
    /// (`Acquire`).
    pub fn connections(&self) -> u32 {
        self.connections.load(Ordering::Acquire)
    }

    /// Re-initialize a freshly-mapped segment: zero the counts, the wake epoch
    /// and the parked mask, stamp `depth` and one connection, and BUMP the
    /// re-arm epoch.
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
        self.counts.store(0, Ordering::Release);
        self.depth.store(depth, Ordering::Release);
        self.wake_seq.store(0, Ordering::Release);
        self.parked.store(0, Ordering::Release);
        self.released.store(0, Ordering::Release);
        self.connections.store(1, Ordering::Release);
        self.epoch.fetch_add(1, Ordering::Release);
    }

    /// `true` once the consumer's queue is at its declared depth — the producer
    /// must DEFER entry to its tick (`Acquire` load, so the producer observes
    /// every drain that happens-before).
    ///
    /// This is the mapped form of `BlockDeferEdge`'s
    /// `outstanding() >= threshold`, unchanged in shape: the count is the
    /// queued and held frames, forgiven ones excluded.
    ///
    /// A word stamped `depth == 0` reads full unconditionally; see the
    /// `reinit` note on why that is a plan-stamping bug rather than a
    /// reachable configuration.
    ///
    /// A released slot ([`Self::release_slot`]) is never full.
    pub fn is_full(&self) -> bool {
        !self.is_released() && self.outstanding() >= u64::from(self.depth.load(Ordering::Acquire))
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

    /// Record that one frame is being published to this edge's topic. Returns
    /// the `outstanding` count THIS publish created.
    ///
    /// The publisher takes this count through
    /// [`record_published_with_undo`](Self::record_published_with_undo)
    /// BEFORE `send()` makes the frame visible
    /// (`CerulionPublisher::send_counted`), and takes it back with
    /// [`record_unpublished`](Self::record_unpublished) when the send reaches
    /// no queue. The order is what keeps a consumer in another process exact:
    /// the count is sequenced before the queue push, the push is a `Release`
    /// store and the consumer's pop an `Acquire` load of it, and the drain is
    /// sequenced after the pop, so the drain of a frame always finds that
    /// frame's count on the word. A count taken after `send()` returns can land
    /// after the drain of the frame it counts; the drain then saturates at 0
    /// and the late count leaves the word one frame above an empty queue.
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
    /// [`try_claim`](Self::try_claim) gate). The RMW returns the counts it
    /// stored and the transition is atomic, so its count is exactly the one
    /// that existed at the instant of the publish, whatever happens next.
    ///
    /// The queued count saturates at `u32::MAX`, past any depth, and the
    /// reported value is what the word holds after the RMW.
    pub fn record_published(&self) -> u64 {
        self.record_published_n(1)
    }

    /// Record that `n` frames were published to this edge's queue: the
    /// `n`-at-a-time form of [`record_published`](Self::record_published),
    /// which delegates here so there is ONE increment implementation. The
    /// publish path moves one frame at a time.
    ///
    /// Returns the count against depth ([`Self::outstanding`]) this call
    /// created, for the same race-free reason
    /// [`record_published`](Self::record_published) gives: the RMW's own
    /// result is the only reading that describes the instant of the
    /// increment, and a later load is a different instant.
    ///
    /// `n == 0` is still an RMW returning the current value, a correct no-op
    /// rather than a special case.
    ///
    /// # The forgiven backlog
    ///
    /// The consumer's queues hold `depth * connections` frames. When the
    /// queued count and the forgiven backlog together pass that, iceoryx2's
    /// overflow evicts the oldest queued frames, and the forgiven ones are the
    /// oldest, so the same RMW takes the backlog down to
    /// `depth * connections - queued`. On one connection that is the exact
    /// eviction count. On several it is a lower bound, because one connection
    /// can overflow while another has room: the backlog can then stay above
    /// the forgiven frames really queued, and later drains pay that excess
    /// instead of counted frames. The count then reads high, by as much as
    /// `depth`: the slot can read full with the consumer's queue empty and
    /// hold the producer with nothing left to drain. On a local word, the only
    /// kind with several connections (a split edge with several producers
    /// mints no word), the consumer's next drain that finds its queue empty
    /// zeroes the queued count and the backlog
    /// ([`CreditWord::record_drained_to_empty`]). The count never reads low,
    /// so the producer never publishes past `depth`. Graph load refuses
    /// `block_timeout_ms` on a topic with more than one in-graph producer or
    /// listed in `multi_publisher_topics:`, and on an edge that counts on a
    /// [`CreditWord::Mapped`] word, so a graph forgives only a local word with
    /// one connection, where this bound is exact.
    ///
    /// Deliberately issues NO wake: nothing ever parks waiting for the count to
    /// RISE (the release direction is [`record_drained`](Self::record_drained),
    /// which does ring).
    ///
    /// On a released slot ([`Self::release_slot`]) this counts nothing and
    /// returns `0`: nobody drains a released slot, so a count would only climb.
    pub fn record_published_n(&self, n: u64) -> u64 {
        if self.is_released() {
            return 0;
        }
        let (_, next) = self.update_counts(Ordering::Release, |c| {
            c.published(n, || self.queue_capacity())
        });
        next.outstanding()
    }

    /// Count one frame as [`record_published`](Self::record_published) does,
    /// and return the token that takes exactly this count back. The publisher
    /// calls this on every block word BEFORE `send()` makes the frame visible
    /// (`CerulionPublisher::send_counted`) and hands the token to
    /// [`record_unpublished`](Self::record_unpublished) when the send reaches
    /// no queue.
    ///
    /// The token holds the packed counts the RMW replaced and the counts it
    /// stored, so a take-back can restore the forgiven backlog the count
    /// clamped as well as the queued count it raised.
    ///
    /// `None` on a released slot ([`Self::release_slot`]): the count counts
    /// nothing there, so there is nothing to take back.
    #[must_use = "the token is the only way to take this count back when the send fails"]
    pub fn record_published_with_undo(&self) -> Option<PublishUndo> {
        if self.is_released() {
            return None;
        }
        let (prev, next) = self.update_counts(Ordering::Release, |c| {
            c.published(1, || self.queue_capacity())
        });
        Some(PublishUndo {
            replaced: prev.pack(),
            stored: next.pack(),
        })
    }

    /// Take back the count [`record_published_with_undo`](Self::record_published_with_undo)
    /// took for a send that reached no queue. Returns the count against depth
    /// ([`Self::outstanding`]) after the take-back.
    ///
    /// The inverse of the count, for the one publish whose `send()` returned
    /// an error: with safe overflow on and the default degradation action, an
    /// iceoryx2 send that errs pushed the frame to no connection, so no
    /// consumer can ever drain it and the count it took must not stand.
    ///
    /// When the word still holds the counts the count stored, ONE
    /// compare-exchange puts back the counts it replaced, the forgiven backlog
    /// included. That is the arm a local word takes: its producer and consumer
    /// fire one at a time on one thread, so nothing moves the word between the
    /// count and the take-back. A word equal to the stored counts is exact to
    /// restore even after other writes in between, because their net effect
    /// on the word is then nothing.
    ///
    /// When another writer moved the word since the count (a consumer in
    /// another process drained it, or [`Self::release_slot`] zeroed it), the
    /// take-back subtracts one queued frame, saturating at 0: a drain of a
    /// frame the word never counted can take the count away first, and a
    /// wrapping subtraction would then read `u32::MAX`, which defers the
    /// producer forever. That subtraction is exact because such a word holds
    /// no backlog: graph load refuses `block_timeout_ms` on an edge that
    /// counts on a [`CreditWord::Mapped`] word and on a topic with several
    /// producers, and a released slot holds no counts.
    ///
    /// Issues no wake on either arm: the frame never reached a queue, so no
    /// room a producer could be parked for has been freed.
    pub fn record_unpublished(&self, undo: PublishUndo) -> u64 {
        if self
            .counts
            .compare_exchange(
                undo.stored,
                undo.replaced,
                Ordering::Release,
                Ordering::Acquire,
            )
            .is_ok()
        {
            return Counts::unpack(undo.replaced).outstanding();
        }
        let (_, next) = self.update_counts(Ordering::Release, |c| {
            debug_assert_eq!(
                c.backlog, 0,
                "a take-back subtracts only on a word with no forgiven backlog"
            );
            c.unpublished()
        });
        next.outstanding()
    }

    /// Record that `removed` samples were drained from the consumer's queue —
    /// decrement the mirror and FREE the credit (bumping the wake epoch and
    /// waking any parked producer).
    ///
    /// The forgiven backlog is paid first: each connection's queue is FIFO
    /// and the frames a timeout release forgave are its oldest, so the first
    /// `backlog` frames a drain removes are forgiven ones and free no credit.
    /// Such a drain still rings, because the consumer moved, which restarts
    /// the `block_timeout_ms` clock.
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
    ///
    /// Returns what the drain did, read off this one RMW ([`Drained`]): the
    /// forgiven frames it paid, and the frames it removed past both the
    /// backlog and the queued count, which the subtraction clipped at 0.
    pub fn record_drained(&self, removed: u64) -> Drained {
        if removed == 0 {
            return Drained::default();
        }
        let (prev, next) = self.update_counts(Ordering::Release, |c| c.drained(removed));
        self.note_credit_freed();
        Drained::between(removed, prev, next)
    }

    /// [`Self::record_drained`] and the empty-queue resync in ONE RMW: the
    /// drain removed `removed` frames and its last `receive` found the
    /// consumer's queue empty, so the queued count and the backlog become 0
    /// and the held counts stay. Rings when `removed` is non-zero, as a drain
    /// does, or when the resync changed a count; an idle drain of an empty,
    /// zero word is one load. Private to this module: [`CreditWord`] calls it
    /// on a local word only.
    ///
    /// Returns what the drain half did, as [`Self::record_drained`] does; the
    /// queued count and the backlog the resync zeroes left the queue unread,
    /// so they are neither paid nor clipped.
    fn record_drained_and_emptied(&self, removed: u64) -> Drained {
        let mut changed = false;
        let mut drain = Drained::default();
        let _ = self
            .counts
            .try_update(Ordering::Release, Ordering::Acquire, |v| {
                let current = Counts::unpack(v);
                let drained = current.drained(removed);
                drain = Drained::between(removed, current, drained);
                let next = drained.emptied();
                changed = next != current;
                changed.then(|| next.pack())
            });
        if removed != 0 || changed {
            self.note_credit_freed();
        }
        drain
    }

    /// Record that frames the consumer popped from its queue now sit in its
    /// matcher's slots, unserved, in ONE RMW. `counted` of them count against
    /// `depth` again (the pop's [`record_drained`](Self::record_drained) took
    /// them off). `forgiven` of them were frames a timeout release forgave,
    /// whose pop paid the backlog: they go to the held-forgiven count and
    /// still count nothing, so the producer keeps room for `depth` frames
    /// published after the release. Held frames are counted apart from the
    /// queued ones, because a held frame is out of the queue and the eviction
    /// bound in [`record_published_n`](Self::record_published_n) reads the
    /// queue alone.
    ///
    /// Issues no wake: nothing parks waiting for the count to rise. Counts
    /// nothing on a released slot, or when both counts are `0`.
    pub fn record_slot_held(&self, counted: u64, forgiven: u64) {
        if (counted == 0 && forgiven == 0) || self.is_released() {
            return;
        }
        self.update_counts(Ordering::Release, |c| c.slot_held(counted, forgiven));
    }

    /// Record that `k` held frames left the consumer's slots (served, skipped,
    /// discarded, or dropped with the subscriber). Held frames a timeout
    /// release forgave are paid first, then counted ones (saturating). Rings
    /// the wake word like [`record_drained`](Self::record_drained); `k == 0`
    /// is a no-op.
    pub fn record_slot_freed(&self, k: u64) {
        if k == 0 {
            return;
        }
        self.update_counts(Ordering::Release, |c| c.slot_freed(k));
        self.note_credit_freed();
    }

    /// Record that a held frame left the consumer's slots and a frame the
    /// consumer popped from its queue took its slot, in ONE RMW, after the
    /// pop's [`record_drained`](Self::record_drained). A held forgiven frame
    /// leaves first, and the entering frame is held forgiven when
    /// `forgiven_entering` (its pop paid the backlog a timeout release
    /// forgave). A counted frame for a counted one stores nothing and rings
    /// nothing, so on a word no release forgave the replacement is the pop
    /// alone. Rings the wake word only when the count falls (a counted frame
    /// leaves and a forgiven one enters). Counts nothing on a released slot.
    pub fn record_slot_replaced(&self, forgiven_entering: bool) {
        if self.is_released() {
            return;
        }
        let mut freed = false;
        let _ = self
            .counts
            .try_update(Ordering::Release, Ordering::Acquire, |v| {
                let current = Counts::unpack(v);
                let next = current.slot_replaced(forgiven_entering);
                freed = next.outstanding() < current.outstanding();
                (next != current).then(|| next.pack())
            });
        if freed {
            self.note_credit_freed();
        }
    }

    /// Give up this consumer's slot for the rest of the run: mark it
    /// released, zero every count, and ring the wake word so a parked
    /// producer re-derives its gate at once.
    ///
    /// Returns `true` for the call that released the slot and `false` when it
    /// was already released, so the caller emits its loud event once.
    ///
    /// Only THIS word changes. Every other block consumer of the topic holds
    /// its own word and keeps its own count.
    ///
    /// A publish that read the slot unreleased just before this call can still
    /// land its increment after the zeroing store; [`Self::is_full`] reads the
    /// mark first, so that stray count never defers anyone.
    pub fn release_slot(&self) -> bool {
        if self
            .released
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return false;
        }
        self.counts.store(0, Ordering::Release);
        self.note_credit_freed();
        true
    }

    /// Has [`Self::release_slot`] given this slot up (`Acquire`)?
    pub fn is_released(&self) -> bool {
        self.released.load(Ordering::Acquire) != 0
    }

    /// Forgive every frame counted against `depth`, leaving the slot armed:
    /// one RMW moves the queued count into the backlog and the held count
    /// into the held-forgiven count, then the wake word rings. Returns the
    /// count that was forgiven.
    ///
    /// The forgiven frames stay where they are. The producer may now publish
    /// `depth` more, and a publish that overflows the consumer's queue evicts
    /// its oldest frame, a forgiven one
    /// ([`record_published_n`](Self::record_published_n)). A drain or a slot
    /// exit pays the forgiven frames off before any counted frame, and a
    /// forgiven frame the matcher pops into a slot stays forgiven
    /// ([`record_slot_held`](Self::record_slot_held)), so the slot holds the
    /// producer again once `depth` frames published after the release are
    /// queued or held, and none of those is evicted. The `block_timeout_ms`
    /// release uses this.
    pub fn forgive_outstanding(&self) -> u64 {
        let capacity = self.queue_capacity();
        let (prev, _) = self.update_counts(Ordering::AcqRel, |c| c.forgive(capacity));
        let forgiven = prev.outstanding();
        if forgiven != 0 {
            self.note_credit_freed();
        }
        forgiven
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
    /// Linux and Android: shared `FUTEX_WAKE` on `wake_seq`. macOS ≥ 14.4:
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
        #[cfg(any(target_os = "linux", target_os = "android"))]
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

    /// Frames counted against `depth` on this edge (`Acquire`): queued and
    /// held, forgiven ones excluded. A test/diagnostic accessor
    /// (Principle #3), never a control input; the gate is
    /// [`try_claim`](Self::try_claim).
    pub fn outstanding(&self) -> u64 {
        Counts::unpack(self.counts.load(Ordering::Acquire)).outstanding()
    }

    /// Frames a timeout release forgave that the consumer still holds, queued
    /// or in a slot (`Acquire`), as the word bounds them: on a word with
    /// several connections it can read above the frames really there (see
    /// [`record_published_n`](Self::record_published_n)). Diagnostic only.
    pub fn forgiven(&self) -> u64 {
        Counts::unpack(self.counts.load(Ordering::Acquire)).forgiven()
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
    /// - **Linux and Android**: shared `FUTEX_WAIT` on `wake_seq` with compare `snapshot`,
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
        #[cfg(any(target_os = "linux", target_os = "android"))]
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
        #[cfg(not(any(target_os = "linux", target_os = "android", target_os = "macos")))]
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
/// Linux and Android always (shared futex); macOS iff the shared `os_sync` BACKEND is usable
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
    #[cfg(any(target_os = "linux", target_os = "android"))]
    {
        // Shared futex on the credit word; no per-host resolution, no switch
        // beyond CREDIT_WAKE_ENV (which gates the WATCH LIST, not the tier).
        true
    }
    #[cfg(target_os = "macos")]
    {
        credit_os_sync_tier_active()
    }
    #[cfg(not(any(target_os = "linux", target_os = "android", target_os = "macos")))]
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
/// - **Linux and Android**: `credit_shm_name_verbose`: `/cer_crd_<ns>_<fnv1a64(id):016x>`.
///   The raw `<ns>` in the `/dev/shm` filename is a deliberate debuggability aid
///   (an `ls /dev/shm` names the deployment).
/// - **macOS**: `credit_shm_name_compact`:
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
    #[cfg(any(target_os = "linux", target_os = "android"))]
    {
        credit_shm_name_verbose(ns, id)
    }
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    {
        credit_shm_name_compact(ns, id)
    }
}

/// The Linux and Android name shape: `/cer_crd_<ns>_<fnv1a64(id):016x>` - `ns` verbatim
/// (deployment-visible in `/dev/shm`), `id` hashed. See `credit_shm_name`.
#[cfg(any(target_os = "linux", target_os = "android", test))]
fn credit_shm_name_verbose(ns: &str, id: &str) -> String {
    let h = fnv1a64(id.as_bytes());
    format!("/cer_crd_{ns}_{h:016x}")
}

/// The macOS-safe name shape: `/cer_crd_<fnv1a64(ns 0x1f id):016x>` - 25 chars,
/// under the macOS 31-char `PSHMNAMLEN` cap for ANY `(ns, id)`. The 0x1F unit
/// separator keeps the `(ns, id)` split unambiguous. See `credit_shm_name`.
#[cfg(any(not(any(target_os = "linux", target_os = "android")), test))]
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
/// [`Deref`](std::ops::Deref), invisible to every call site. The one
/// operation whose arms differ is [`CreditWord::record_drained_to_empty`],
/// which resyncs a local word only.
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
/// the publish path pays one RMW on the word either way. That price
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
    /// (Principle #3), never a control input; every operation but
    /// [`Self::record_drained_to_empty`] is identical on both arms.
    pub fn is_mapped(&self) -> bool {
        matches!(self, Self::Mapped(_))
    }

    /// Record a drain of `removed` frames whose last `receive` found the
    /// consumer's queue empty, on every connection.
    ///
    /// On a [`Self::Local`] word the same RMW also zeroes the queued count and
    /// the forgiven backlog; the held counts stay. Nothing is queued, so this
    /// is exact while no publish counted on the word runs between that empty
    /// `receive` and the RMW, and the runtime's local wiring guarantees it:
    /// every producer and consumer of a `block` topic fires one at a time on
    /// the thread that steps their level loop, an executor unit keeps them
    /// together, and a publish counts its frame and sends it inside its
    /// producer's fire, so no count is ever left waiting for its push. It clears
    /// the excess the backlog bound leaves on a word with several connections
    /// ([`CreditShared::record_published_n`]): once the consumer has drained
    /// everything it was sent, the slot counts only the frames its matcher's
    /// slots hold. A graph never forgives such a word: graph load refuses
    /// `block_timeout_ms` on a topic with several producers.
    ///
    /// On a [`Self::Mapped`] word it is [`CreditShared::record_drained`]. The
    /// producer runs in another process, so a frame can be sent and counted
    /// between the empty `receive` and the RMW, and a zeroing would drop its
    /// count. A graph never forgives a mapped word (a cross-process binding
    /// carries no timeout, and graph load refuses `block_timeout_ms` on such
    /// an edge), so it holds no backlog for a zeroing to clear.
    ///
    /// Returns what the drain did, as [`CreditShared::record_drained`] does:
    /// the forgiven frames it paid, `0` on a mapped word, and the frames it
    /// clipped at 0. On a local word both come from the drain half of the
    /// RMW: the queued count and the backlog the resync zeroes were never
    /// removed, so they count as neither.
    pub fn record_drained_to_empty(&self, removed: u64) -> Drained {
        match self {
            Self::Local(shared) => shared.record_drained_and_emptied(removed),
            Self::Mapped(mapped) => mapped.record_drained(removed),
        }
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
        assert_eq!(core::mem::size_of::<CreditShared>(), 32);
        assert!(core::mem::align_of::<CreditShared>() >= 8);
        assert_eq!(core::mem::offset_of!(CreditShared, counts), 0);
        assert_eq!(core::mem::offset_of!(CreditShared, depth), 8);
        assert_eq!(core::mem::offset_of!(CreditShared, wake_seq), 12);
        assert_eq!(core::mem::offset_of!(CreditShared, parked), 16);
        assert_eq!(core::mem::offset_of!(CreditShared, epoch), 20);
        assert_eq!(core::mem::offset_of!(CreditShared, released), 24);
        assert_eq!(core::mem::offset_of!(CreditShared, connections), 28);
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

    /// The Linux and Android (verbose) shape oracle - `ns` verbatim, `id` FNV-hashed. Driven
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
        // The drain of the same frames takes the count back to zero.
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

    /// `record_unpublished` takes back the one count its token recorded and
    /// returns the count it left. Oracle, by hand, on a depth-4 word holding
    /// 3: a counted publish reads 4 and its take-back leaves 3. On a word at
    /// 0: a counted publish reads 1, a drain of 1 takes the word to 0 before
    /// the take-back, and the take-back then leaves it at 0 and returns 0. A
    /// wrapping subtraction there would read `u32::MAX`, past any depth.
    #[test]
    fn a_take_back_removes_one_count_and_stops_at_zero() {
        let w = CreditShared::new(4);
        assert_eq!(w.record_published_n(3), 3);
        let undo = w
            .record_published_with_undo()
            .expect("an armed slot counts the publish");
        assert_eq!(w.outstanding(), 4, "the count before the send");
        assert_eq!(w.record_unpublished(undo), 3, "4 -> 3");
        assert_eq!(w.outstanding(), 3);

        let w = CreditShared::new(4);
        let undo = w
            .record_published_with_undo()
            .expect("an armed slot counts the publish");
        assert_eq!(w.outstanding(), 1);
        w.record_drained(1);
        assert_eq!(w.outstanding(), 0, "a drain took the count first");
        assert_eq!(w.record_unpublished(undo), 0, "a take-back on a word at 0");
        assert_eq!(w.outstanding(), 0, "leaves the word at 0, never u32::MAX");
        assert!(w.try_claim(), "so the producer is admitted");
    }

    /// A take-back rings no wake on either arm: the frame never reached a
    /// queue, so it freed no room a parked producer waits for. Oracle:
    /// `wake_seq` reads the same before and after the restore arm (the word
    /// still holds what the count stored) and the subtract arm (a second
    /// count moved the word: 2 -> 1), where a drain of one frame bumps it by
    /// one.
    #[test]
    fn a_take_back_rings_no_wake() {
        let w = CreditShared::new(2);
        let undo = w
            .record_published_with_undo()
            .expect("an armed slot counts the publish");
        let before = w.wake_seq_snapshot();
        assert_eq!(w.record_unpublished(undo), 0, "the restore arm");
        assert_eq!(w.wake_seq_snapshot(), before, "no ring on a restore");
        let undo = w
            .record_published_with_undo()
            .expect("an armed slot counts the publish");
        w.record_published();
        assert_eq!(w.record_unpublished(undo), 1, "the subtract arm");
        assert_eq!(w.wake_seq_snapshot(), before, "no ring on a subtract");
        assert_eq!(w.record_drained(1), Drained::default());
        assert_eq!(
            w.wake_seq_snapshot(),
            before.wrapping_add(1),
            "the control: a drain of one counted frame rings once"
        );
    }

    /// On a word no release forgave, `record_drained` pays nothing and
    /// returns as clipped the frames its subtraction could not take off
    /// because the word reached 0. Oracle vectors, by hand, as `(word before,
    /// removed) -> (clipped, word after)`: `(2, 1) -> (0, 1)`, `(1, 1) -> (0,
    /// 0)`, `(1, 99) -> (98, 0)`, `(0, 1) -> (1, 0)`, `(3, 0) -> (0, 3)`, and
    /// a removal past `u32::MAX`, `(3, u64::MAX) -> (u64::MAX - 3, 0)`.
    #[test]
    fn a_drain_returns_the_frames_it_could_not_take_off() {
        let vectors: [(u64, u64, u64, u64); 6] = [
            (2, 1, 0, 1),
            (1, 1, 0, 0),
            (1, 99, 98, 0),
            (0, 1, 1, 0),
            (3, 0, 0, 3),
            (3, u64::MAX, u64::MAX - 3, 0),
        ];
        for (before, removed, clipped, after) in vectors {
            let w = CreditShared::new(4);
            assert_eq!(w.record_published_n(before), before);
            assert_eq!(
                w.record_drained(removed),
                Drained { paid: 0, clipped },
                "a drain of {removed} on a word at {before} pays nothing and clips {clipped}"
            );
            assert_eq!(
                w.outstanding(),
                after,
                "and leaves the word at {after} (drain of {removed} on {before})"
            );
        }
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

    /// `release_slot` is sticky and once-only: it zeroes the count, rings the
    /// wake word, and from then on the slot is never full and counts no
    /// publish. Another word is untouched.
    #[test]
    fn a_released_slot_is_never_full_and_counts_nothing() {
        let w = CreditWord::local(2);
        let other = CreditWord::local(2);
        w.record_published_n(2);
        other.record_published_n(2);
        assert!(w.is_full() && other.is_full());
        let seq = w.wake_seq_snapshot();
        assert!(w.release_slot(), "the first release releases");
        assert!(!w.release_slot(), "a second release is a no-op");
        assert!(w.is_released());
        assert_eq!(w.outstanding(), 0);
        assert!(
            w.wake_seq_snapshot() > seq,
            "the release rings the wake word"
        );
        assert!(!w.is_full() && w.try_claim());
        assert_eq!(w.record_published(), 0, "a released slot counts nothing");
        assert_eq!(w.outstanding(), 0);
        assert!(other.is_full(), "another word keeps its count");
        assert_eq!(other.outstanding(), 2);
    }

    /// `forgive_outstanding` moves the count into the backlog and rings, but
    /// leaves the slot armed: it fills and defers again.
    #[test]
    fn a_forgiven_slot_stays_armed() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        let seq = w.wake_seq_snapshot();
        assert_eq!(w.forgive_outstanding(), 2);
        assert!(w.wake_seq_snapshot() > seq);
        assert!(!w.is_released());
        assert_eq!(w.forgive_outstanding(), 0, "nothing left to forgive");
        assert_eq!(
            w.wake_seq_snapshot(),
            seq + 1,
            "and an empty forgive rings nothing"
        );
        w.record_published_n(2);
        assert!(w.is_full(), "the slot holds the producer again");
    }

    /// The pack round-trips every count, and no count bleeds into another.
    #[test]
    fn the_four_counts_pack_into_one_word_and_back() {
        let c = Counts {
            queued: u32::MAX,
            backlog: 0x1234,
            held: 2,
            held_forgiven: 0xAB,
        };
        assert_eq!(Counts::unpack(c.pack()), c);
        assert_eq!(Counts::unpack(0), Counts::default());
        let q = Counts {
            queued: u32::MAX,
            ..Counts::default()
        }
        .published(5, || 0);
        assert_eq!(
            (q.queued, q.backlog),
            (u32::MAX, 0),
            "the queued count saturates instead of carrying into the backlog"
        );
    }

    /// A depth-2 slot: forgive frames 1 and 2 while both are queued, publish 3
    /// (the overflow evicts 1), the consumer drains 2. The drain pays the
    /// backlog, so 3 is still counted and the producer is held after 4: frame
    /// 5 would evict 3.
    #[test]
    fn a_drain_after_a_release_pays_the_forgiven_frames_first() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        assert_eq!(w.forgive_outstanding(), 2);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 2));
        assert_eq!(w.record_published(), 1, "frame 3");
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (1, 1),
            "frame 3 evicted forgiven frame 1 from the depth-2 queue"
        );
        w.record_drained(1);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (1, 0),
            "draining frame 2 pays the backlog, not frame 3's credit"
        );
        assert!(w.try_claim());
        assert_eq!(w.record_published(), 2, "frame 4");
        assert!(!w.try_claim(), "frame 5 must wait: 3 and 4 fill the slot");
        w.record_drained(2);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));
    }

    /// A drain larger than the backlog pays the backlog, then the counted
    /// frames, and floors at zero.
    #[test]
    fn a_drain_past_the_backlog_pays_counted_frames_and_floors_at_zero() {
        let w = CreditWord::local(4);
        w.record_published_n(3);
        w.forgive_outstanding();
        w.record_published_n(1);
        assert_eq!((w.outstanding(), w.forgiven()), (1, 3), "3 + 1 fit in 4");
        w.record_drained(10);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));
    }

    /// Frames held in the matcher's slots count against depth, are forgiven
    /// with the rest, and stay out of the eviction bound: a held frame is not
    /// in the queue, so a publish does not count it as evicted.
    #[test]
    fn held_frames_count_against_depth_and_their_forgiven_exit_frees_no_credit() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        // The matcher pops frame 1 into its head slot.
        w.record_drained(1);
        w.record_slot_held(1, 0);
        assert_eq!(w.outstanding(), 2, "the held frame still counts");
        assert!(!w.try_claim());
        assert_eq!(w.forgive_outstanding(), 2);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 2));
        assert_eq!(w.record_published(), 1, "frame 3");
        assert_eq!(
            w.forgiven(),
            2,
            "the queue holds forgiven frame 2 and frame 3: nothing was evicted"
        );
        let seq = w.wake_seq_snapshot();
        w.record_slot_freed(1);
        assert!(w.wake_seq_snapshot() > seq, "a slot exit rings");
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (1, 1),
            "the forgiven held frame left: frame 3 is still counted"
        );
        w.record_drained(1);
        assert_eq!((w.outstanding(), w.forgiven()), (1, 0), "frame 2 drained");
        assert_eq!(w.record_published(), 2, "frame 4 fills the slot");
        assert!(!w.try_claim());
        w.record_slot_held(0, 0);
        w.record_slot_freed(0);
        assert_eq!(w.outstanding(), 2, "zero slot moves change nothing");
    }

    /// On several connections the eviction bound is a lower bound: the
    /// backlog can stay above what is queued, and a drain then pays it
    /// instead of a counted frame, so the count reads high and the producer
    /// defers early. It never reads low.
    #[test]
    fn several_connections_bound_evictions_from_below_and_never_undercount() {
        let w = CreditWord::local(2);
        w.set_connections(2);
        assert_eq!(w.connections(), 2);
        // One frame on each connection, then the release.
        w.record_published_n(2);
        assert_eq!(w.forgive_outstanding(), 2);
        // Connection 1 publishes two more: its own queue (depth 2) evicts its
        // forgiven frame, which the word cannot see.
        w.record_published_n(2);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (2, 2),
            "4 frames fit 2 connections of depth 2, so no eviction is counted"
        );
        assert!(!w.try_claim(), "the two new frames hold the producer");
        // The consumer drains the forgiven frame of connection 2, then the two
        // new frames of connection 1.
        w.record_drained(1);
        assert!(!w.try_claim(), "a forgiven frame's drain frees no credit");
        w.record_drained(2);
        assert_eq!(
            w.outstanding(),
            1,
            "the missed eviction leaves the count one high, never low"
        );
        w.set_connections(0);
        assert_eq!(w.connections(), 1, "a zero stamp reads as one connection");
    }

    /// A drain that finds the queue empty zeroes a local word's queued count
    /// and backlog in the drain's own RMW, and keeps its held frames. The
    /// count is the one the several-connections test leaves high.
    #[test]
    fn a_drain_that_empties_the_queue_resyncs_a_local_word() {
        let w = CreditWord::local(2);
        w.set_connections(2);
        w.record_published_n(2);
        w.forgive_outstanding();
        w.record_published_n(2);
        w.record_drained(2);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (2, 0),
            "the drains paid the backlog: full, with the queue empty"
        );
        let seq = w.wake_seq_snapshot();
        w.record_drained_to_empty(0);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));
        assert!(
            w.wake_seq_snapshot() > seq,
            "a resync that frees credit rings"
        );
        assert!(w.try_claim());

        // A drain that removes frames and empties the queue, with a frame held
        // in a slot: the queued count and the backlog go, the held frame stays.
        w.record_published_n(2);
        w.record_drained(1);
        w.record_slot_held(1, 0);
        w.forgive_outstanding();
        w.record_published_n(1);
        assert_eq!((w.outstanding(), w.forgiven()), (1, 2));
        w.record_drained_to_empty(2);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (0, 1),
            "the forgiven held frame is out of the queue and stays"
        );
        w.record_slot_freed(1);
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));

        // An idle drain of an empty, zero word writes and rings nothing.
        let seq = w.wake_seq_snapshot();
        w.record_drained_to_empty(0);
        assert_eq!(w.wake_seq_snapshot(), seq, "an idle drain does not ring");
        assert_eq!(w.outstanding(), 0);
    }

    /// A mapped word takes the drain alone: the producer runs in another
    /// process, so a count the drain cannot see may be in flight, and the
    /// queued count is left as the drain leaves it.
    #[test]
    fn a_drain_that_empties_the_queue_leaves_a_mapped_word_to_its_counts() {
        let ns = format!("c2a_empty_{}", std::process::id());
        let _owner = MappedCredit::create_owned(&ns, "e", 2).expect("owner");
        let w = CreditWord::mapped(std::sync::Arc::new(
            MappedCredit::open_unowned(&ns, "e").expect("peer"),
        ));
        w.record_published_n(2);
        w.record_drained_to_empty(0);
        assert_eq!(w.outstanding(), 2, "no resync on a mapped word");
        let seq = w.wake_seq_snapshot();
        assert_eq!(
            w.record_drained_to_empty(1).paid,
            0,
            "a word no release forgave pays no forgiven frame"
        );
        assert_eq!(w.outstanding(), 1, "the drain itself still counts");
        assert!(w.wake_seq_snapshot() > seq, "and rings");
    }

    /// The word against a model of the consumer's queues, one per connection
    /// of `depth` frames that evicts its oldest on overflow, for depths 1, 2
    /// and 4 and 1 to 3 connections, over seeded random runs of publishes
    /// (gated by `try_claim`, on a random connection), FIFO reads that pop one
    /// frame or find the queues empty, drains to empty, and timeout releases of
    /// a full slot. Everything runs on one thread, as the local wiring does.
    /// After every step the word counts at least the unforgiven frames queued,
    /// an eviction only ever takes a forgiven frame, and a read that finds the
    /// queues empty leaves no queued or forgiven count.
    #[test]
    fn a_local_word_matches_a_model_of_its_queues() {
        use std::collections::VecDeque;
        let mut rng: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut next = move |n: u64| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng % n
        };
        for depth in [1u32, 2, 4] {
            for connections in 1..=3usize {
                let w = CreditWord::local(depth);
                w.set_connections(connections as u32);
                // Each queued frame and whether a release forgave it.
                let mut queues: Vec<VecDeque<bool>> = vec![VecDeque::new(); connections];
                let mut releases = 0u32;
                for step in 0..4000u32 {
                    let at = format!("depth {depth}, {connections} connections, step {step}");
                    match next(8) {
                        0..=2 => {
                            if w.try_claim() {
                                let q = &mut queues[next(connections as u64) as usize];
                                q.push_back(false);
                                if q.len() > depth as usize {
                                    let evicted = q.pop_front();
                                    assert_eq!(
                                        evicted,
                                        Some(true),
                                        "{at}: an eviction took an unforgiven frame"
                                    );
                                }
                                w.record_published();
                            }
                        }
                        3..=4 => match queues.iter_mut().find(|q| !q.is_empty()) {
                            Some(q) => {
                                q.pop_front();
                                w.record_drained(1);
                            }
                            None => {
                                w.record_drained_to_empty(0);
                                assert_eq!(
                                    (w.outstanding(), w.forgiven()),
                                    (0, 0),
                                    "{at}: a read that found the queues empty"
                                );
                            }
                        },
                        5 => {
                            let removed: usize = queues.iter().map(VecDeque::len).sum();
                            queues.iter_mut().for_each(VecDeque::clear);
                            w.record_drained_to_empty(removed as u64);
                            assert_eq!(
                                (w.outstanding(), w.forgiven()),
                                (0, 0),
                                "{at}: a drain to empty"
                            );
                        }
                        _ => {
                            if !w.try_claim() {
                                w.forgive_outstanding();
                                releases += 1;
                                queues.iter_mut().flatten().for_each(|f| *f = true);
                            }
                        }
                    }
                    let unforgiven = queues.iter().flatten().filter(|f| !**f).count() as u64;
                    assert!(
                        w.outstanding() >= unforgiven,
                        "{at}: the word counts {} with {unforgiven} unforgiven frames queued",
                        w.outstanding()
                    );
                }
                assert!(
                    releases > 0,
                    "depth {depth}, {connections} connections: the run released the slot"
                );
            }
        }
    }

    /// The sticky release zeroes the backlog and the held counts too.
    #[test]
    fn a_sticky_release_zeroes_every_count() {
        let w = CreditWord::local(2);
        w.record_published_n(1);
        w.record_slot_held(1, 0);
        w.forgive_outstanding();
        w.record_published_n(1);
        assert!(w.forgiven() > 0 && w.outstanding() > 0);
        assert!(w.release_slot());
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));
        w.record_slot_held(1, 1);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (0, 0),
            "a released slot counts no held frame"
        );
    }

    /// The packed counts of `w`, read as one value.
    fn counts_of(w: &CreditShared) -> Counts {
        Counts::unpack(w.counts.load(Ordering::Acquire))
    }

    /// A depth-2 slot: frames 1 and 2 are queued when the release forgives
    /// them. The matcher pops frame 1 into a slot: the pop pays the backlog,
    /// so the held frame is a forgiven one and frames 3 and 4, published
    /// after the release, both fit before the producer is held.
    #[test]
    fn a_forgiven_frame_moved_into_a_slot_stays_forgiven() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        assert_eq!(w.forgive_outstanding(), 2);
        assert_eq!(
            w.record_drained(1).paid,
            1,
            "the pop of frame 1 pays the backlog"
        );
        w.record_slot_held(0, 1);
        assert_eq!(
            counts_of(&w),
            Counts {
                queued: 0,
                backlog: 1,
                held: 0,
                held_forgiven: 1,
            }
        );
        assert_eq!((w.outstanding(), w.forgiven()), (0, 2));
        assert_eq!(w.record_published(), 1, "frame 3");
        assert_eq!((w.outstanding(), w.forgiven()), (1, 2), "nothing evicted");
        assert!(w.try_claim(), "frame 4 fits");
        assert_eq!(w.record_published(), 2, "frame 4 evicts forgiven frame 2");
        assert_eq!(
            counts_of(&w),
            Counts {
                queued: 2,
                backlog: 0,
                held: 0,
                held_forgiven: 1,
            }
        );
        assert!(!w.try_claim(), "frames 3 and 4 hold the producer");
        w.record_slot_freed(1);
        assert_eq!(
            (w.outstanding(), w.forgiven()),
            (2, 0),
            "the forgiven held frame leaves and frees no credit"
        );
    }

    /// Both slots of a depth-2 matcher hold frames the release forgave: the
    /// producer keeps its whole depth for frames published after it.
    #[test]
    fn forgiven_frames_in_both_slots_leave_the_producer_its_whole_depth() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        assert_eq!(w.forgive_outstanding(), 2);
        for frame in 1..=2 {
            assert_eq!(
                w.record_drained(1).paid,
                1,
                "frame {frame} pays the backlog"
            );
            w.record_slot_held(0, 1);
        }
        assert_eq!(
            counts_of(&w),
            Counts {
                queued: 0,
                backlog: 0,
                held: 0,
                held_forgiven: 2,
            }
        );
        assert!(w.try_claim());
        assert_eq!(w.record_published(), 1, "frame 3");
        assert!(w.try_claim());
        assert_eq!(w.record_published(), 2, "frame 4");
        assert!(!w.try_claim(), "frames 3 and 4 hold the producer");
        assert_eq!((w.outstanding(), w.forgiven()), (2, 2));
    }

    /// A drain returns how many forgiven frames it paid: the oldest
    /// `backlog` frames it removes, and none past them. The empty-queue
    /// resync's zeroing of a stale backlog is not a payment.
    #[test]
    fn a_drain_returns_the_forgiven_frames_it_paid() {
        let w = CreditWord::local(4);
        w.record_published_n(3);
        w.forgive_outstanding();
        w.record_published_n(1);
        assert_eq!((w.outstanding(), w.forgiven()), (1, 3));
        assert_eq!(w.record_drained(2).paid, 2, "two forgiven frames");
        assert_eq!(
            w.record_drained(2).paid,
            1,
            "the last forgiven frame, then frame 4"
        );
        assert_eq!(w.record_drained(1).paid, 0, "nothing forgiven is left");
        assert_eq!(w.record_drained(0).paid, 0, "an idle drain pays nothing");
        assert_eq!((w.outstanding(), w.forgiven()), (0, 0));

        let two = CreditWord::local(2);
        two.set_connections(2);
        two.record_published_n(2);
        two.forgive_outstanding();
        two.record_published_n(2);
        assert_eq!((two.outstanding(), two.forgiven()), (2, 2));
        assert_eq!(
            two.record_drained_to_empty(1),
            Drained {
                paid: 1,
                clipped: 0,
            },
            "the drain half pays one forgiven frame; the resync zeroes the other \
             and clips nothing"
        );
        assert_eq!((two.outstanding(), two.forgiven()), (0, 0));
        assert_eq!(two.record_drained_to_empty(0).paid, 0);
    }

    /// Each held count saturates at `u8::MAX` in its own field and never
    /// carries into a neighbour, and a released slot counts no held frame,
    /// forgiven or not.
    #[test]
    fn a_forgiven_hold_saturates_in_its_own_field_and_a_released_slot_counts_none() {
        assert_eq!(
            Counts::default().slot_held(0, 300),
            Counts {
                queued: 0,
                backlog: 0,
                held: 0,
                held_forgiven: 255,
            }
        );
        let near = Counts {
            queued: 7,
            backlog: 3,
            held: 250,
            held_forgiven: 1,
        };
        assert_eq!(near.slot_held(10, 0), Counts { held: 255, ..near });
        let full = Counts::default().slot_held(u64::MAX, u64::MAX);
        assert_eq!(full.pack(), 0xFFFF_0000_0000_0000);
        assert_eq!(Counts::unpack(full.pack()), full);

        let w = CreditWord::local(2);
        w.record_published_n(1);
        w.record_slot_held(0, 0);
        assert_eq!(w.outstanding(), 1, "a move of no frame changes nothing");
        assert!(w.release_slot());
        w.record_slot_held(1, 1);
        assert_eq!(counts_of(&w), Counts::default());
    }

    /// A replaced head moves at most one frame between the two held counts:
    /// a held forgiven frame leaves first, the entering frame lands in the
    /// count `forgiven_entering` names, like for like changes nothing, and
    /// the queued count and the backlog never move.
    #[test]
    fn a_replaced_head_moves_one_frame_between_the_held_counts() {
        let counts = |held, held_forgiven| Counts {
            queued: 3,
            backlog: 2,
            held,
            held_forgiven,
        };
        // (held, held_forgiven, forgiven_entering) -> (held, held_forgiven)
        for (held, held_forgiven, forgiven_entering, want) in [
            (1, 0, false, (1, 0)),
            (0, 1, true, (0, 1)),
            (0, 1, false, (1, 0)),
            (1, 0, true, (0, 1)),
            (1, 1, false, (2, 0)),
            (1, 1, true, (1, 1)),
            (0, 0, false, (0, 0)),
            (0, 0, true, (0, 1)),
            (255, 1, false, (255, 0)),
            (2, 255, true, (2, 255)),
            (1, 255, false, (2, 254)),
            (3, 0, true, (2, 1)),
        ] {
            assert_eq!(
                counts(held, held_forgiven).slot_replaced(forgiven_entering),
                counts(want.0, want.1),
                "held {held}, held forgiven {held_forgiven}, forgiven entering {forgiven_entering}"
            );
        }
    }

    /// The replacement is one RMW: a counted frame for a counted one stores
    /// and rings nothing, a counted frame entering for a forgiven one raises
    /// the count without a ring, a forgiven frame entering for a counted one
    /// lowers it and rings, and a released slot counts nothing.
    #[test]
    fn a_replaced_head_rings_only_when_its_count_falls() {
        let w = CreditWord::local(2);
        w.record_published_n(2);
        w.record_drained(1);
        w.record_slot_held(1, 0);
        let seq = w.wake_seq_snapshot();
        w.record_slot_replaced(false);
        assert_eq!(
            counts_of(&w),
            Counts {
                queued: 1,
                backlog: 0,
                held: 1,
                held_forgiven: 0,
            },
            "like for like"
        );
        assert_eq!(w.wake_seq_snapshot(), seq, "like for like rings nothing");

        // The release forgives both frames, then frames 3 and 4 evict the
        // queued forgiven frame 2, and the pop of frame 3 pays nothing.
        w.forgive_outstanding();
        w.record_published_n(2);
        assert_eq!(w.record_drained(1).paid, 0);
        let seq = w.wake_seq_snapshot();
        w.record_slot_replaced(false);
        assert_eq!(
            counts_of(&w),
            Counts {
                queued: 1,
                backlog: 0,
                held: 1,
                held_forgiven: 0,
            },
            "the forgiven head leaves and frame 3 is held counted"
        );
        assert_eq!((w.outstanding(), w.forgiven()), (2, 0));
        assert_eq!(w.wake_seq_snapshot(), seq, "a rising count rings nothing");

        let falls = CreditWord::local(2);
        falls.counts.store(
            Counts {
                queued: 0,
                backlog: 1,
                held: 1,
                held_forgiven: 0,
            }
            .pack(),
            Ordering::Release,
        );
        assert_eq!(falls.record_drained(1).paid, 1, "the pop pays the backlog");
        let seq = falls.wake_seq_snapshot();
        falls.record_slot_replaced(true);
        assert_eq!(
            counts_of(&falls),
            Counts {
                queued: 0,
                backlog: 0,
                held: 0,
                held_forgiven: 1,
            }
        );
        assert_eq!(
            falls.wake_seq_snapshot(),
            seq + 1,
            "a falling count rings once"
        );

        assert!(falls.release_slot());
        let seq = falls.wake_seq_snapshot();
        falls.record_slot_replaced(true);
        assert_eq!(counts_of(&falls), Counts::default(), "a released slot");
        assert_eq!(falls.wake_seq_snapshot(), seq);
    }

    /// One seeded run of [`a_local_word_with_slots_matches_a_model_of_its_queues_and_slots`]
    /// at `depth` on one connection: the packed word after every step.
    fn slot_model_run(depth: u32, seed: u64) -> Vec<u64> {
        use crate::transport::subscriber::survivor_forgiven;
        use std::collections::VecDeque;
        let mut rng = seed;
        let mut next = move |n: u64| {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng % n
        };
        let w = CreditWord::local(depth);
        // Each queued frame and whether a release forgave it, then the head
        // slot and the second slot.
        let mut queue: VecDeque<bool> = VecDeque::new();
        let mut head: Option<bool> = None;
        let mut second: Option<bool> = None;
        // A pop for a slot: the drain pays the backlog first, and the
        // survivor rule names the popped frame.
        let pop = |queue: &mut VecDeque<bool>| -> Option<bool> {
            let forgiven = queue.pop_front()?;
            let paid = w.record_drained(1).paid;
            let held_forgiven = survivor_forgiven(1, paid);
            assert_eq!(held_forgiven, forgiven, "the word names the popped frame");
            Some(forgiven)
        };
        // A pop into an empty slot, which the slot move records.
        let pop_into_slot = |queue: &mut VecDeque<bool>| -> Option<bool> {
            let forgiven = pop(queue)?;
            w.record_slot_held(u64::from(!forgiven), u64::from(forgiven));
            Some(forgiven)
        };
        let mut trace = Vec::new();
        let (mut releases, mut forgiven_holds, mut discards, mut replacements) =
            (0u32, 0u32, 0u32, 0u32);
        for step in 0..4000u32 {
            let at = format!("depth {depth}, step {step}");
            match next(9) {
                0..=2 => {
                    if w.try_claim() {
                        queue.push_back(false);
                        if queue.len() > depth as usize {
                            assert_eq!(
                                queue.pop_front(),
                                Some(true),
                                "{at}: an eviction took an unforgiven frame"
                            );
                        }
                        w.record_published();
                    }
                }
                3 => {
                    if head.is_none() && second.is_none() {
                        head = pop_into_slot(&mut queue);
                        forgiven_holds += u32::from(head == Some(true));
                    }
                }
                4 => {
                    if head.is_some() && second.is_none() {
                        second = pop_into_slot(&mut queue);
                        forgiven_holds += u32::from(second == Some(true));
                    }
                }
                5 => {
                    // The node reads the head, or the queue when no slot
                    // holds a frame.
                    if head.take().is_some() {
                        w.record_slot_freed(1);
                        head = second.take();
                    } else if queue.pop_front().is_some() {
                        w.record_drained(1);
                    } else {
                        w.record_drained_to_empty(0);
                    }
                }
                6 => {
                    // The matcher skips the head. The second slot's frame is
                    // promoted, or the queue refills the head: the refill
                    // pops first, then the head's exit and the refill's entry
                    // are one replacement, or the head just leaves when the
                    // queue is empty.
                    if head.take().is_some() {
                        discards += 1;
                        head = match second.take() {
                            Some(promoted) => {
                                w.record_slot_freed(1);
                                Some(promoted)
                            }
                            None => match pop(&mut queue) {
                                Some(forgiven) => {
                                    replacements += 1;
                                    w.record_slot_replaced(forgiven);
                                    Some(forgiven)
                                }
                                None => {
                                    w.record_slot_freed(1);
                                    None
                                }
                            },
                        };
                    }
                }
                _ => {
                    if !w.try_claim() {
                        w.forgive_outstanding();
                        releases += 1;
                        queue.iter_mut().for_each(|f| *f = true);
                        head = head.map(|_| true);
                        second = second.map(|_| true);
                    }
                }
            }
            let frames = || queue.iter().chain(head.iter()).chain(second.iter());
            let unforgiven = frames().filter(|f| !**f).count() as u64;
            let forgiven = frames().filter(|f| **f).count() as u64;
            assert_eq!(
                (w.outstanding(), w.forgiven()),
                (unforgiven, forgiven),
                "{at}: the word against the queue and the two slots"
            );
            trace.push(w.counts.load(Ordering::Acquire));
        }
        assert!(
            releases > 0 && forgiven_holds > 0 && discards > 0 && replacements > 0,
            "depth {depth}: {releases} releases, {forgiven_holds} forgiven holds, \
             {discards} discards, {replacements} replaced heads"
        );
        trace
    }

    /// The word against a model of one connection's queue of `depth` frames
    /// (evicting its oldest on overflow) and a matcher's two slots, for
    /// depths 1, 2 and 4, over seeded random runs of gated publishes, pops
    /// into the head and the second slot, reads, skips of the head (the second
    /// slot is promoted, or the queue's pop replaces the head in one update)
    /// and timeout releases of a full slot. On one connection the word is exact:
    /// after every step it counts the unforgiven frames queued or held, and
    /// its forgiven count is the forgiven frames queued or held. The same
    /// seed gives the same packed word after every step.
    #[test]
    fn a_local_word_with_slots_matches_a_model_of_its_queues_and_slots() {
        for depth in [1u32, 2, 4] {
            let first = slot_model_run(depth, 0x9E37_79B9_7F4A_7C15);
            let second = slot_model_run(depth, 0x9E37_79B9_7F4A_7C15);
            assert_eq!(first, second, "depth {depth}: the same seed, the same word");
        }
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
        assert!(c.release_slot());

        c.reinit(5);
        assert!(!c.is_released(), "reinit re-arms a released slot");
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
        unsafe { crate::posix_shm::shm_unlink(name.as_ptr()) };
        // SAFETY: FFI create of a named SHM object; `name` is a valid C string.
        let fd = unsafe {
            crate::posix_shm::shm_open(
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
        unsafe { crate::posix_shm::shm_unlink(name.as_ptr()) };

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

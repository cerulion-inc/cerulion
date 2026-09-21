// SPDX-License-Identifier: AGPL-3.0-only
//! A generic single-producer byte-record ring living in a real POSIX
//! shared-memory segment.
//!
//! The producer side is single-writer ALWAYS. The consumer side is scoped by
//! [`OverrunPolicy`] and the two arms are genuinely different contracts — see
//! contract 7. "SPSC" is accurate for a [`OverrunPolicy::Backpressure`] ring
//! and is the wrong word for a [`OverrunPolicy::FailLoud`] one, which supports
//! N independent readers.
//!
//! # Purpose
//!
//! This is the transport under Cerulion's out-of-process record/replay:
//! the scheduler (producer) writes fixed-size binary records into the ring and an
//! out-of-process recorder `bagd` (consumer) drains them to disk. It is a
//! GENERIC byte-record ring — the trace-specific record + manifest layer lives in
//! [`crate::trace_ring`] on top of this, and a later `logd` is intended
//! to REUSE this exact module for a second, log-record stream. So the two layers
//! are kept cleanly separated: this file knows only "fixed-size opaque records in,
//! zero-copy byte slices out".
//!
//! # Design contract (binding, as amended)
//!
//! 1. **Wait-free producer under the DEFAULT policy.** Under
//!    [`OverrunPolicy::FailLoud`] — what [`ShmRingOwner::create`] selects, and
//!    therefore what the shipped trace ring is — [`ShmRingProducer::push`] does NO
//!    alloc, NO lock, NO syscall, and NO clock read: it copies the record into its
//!    slot and `Release`-stores the write cursor. The producer NEVER reads consumer
//!    state, so a slow/dead consumer can NEVER back-pressure the producer.
//!    [`OverrunPolicy::Backpressure`] DELIBERATELY trades that away for
//!    one caller — see contract 6 — and is opt-in at create; nothing that exists
//!    today selects it, so contract 1 is unchanged for every shipping ring.
//! 2. **Overrun = fail-loud, detected by the CONSUMER.** The ring is deliberately
//!    OVERSIZED (see [`crate::trace_ring::DEFAULT_TRACE_RING_BYTES`]), so a lap
//!    means the consumer has been dead/stalled for a long time. When the consumer
//!    detects it has been lapped (`write_cursor - read_cursor > capacity`) it
//!    returns [`ShmRingError::Overrun`] (the recording layer above fails the
//!    recording loudly). There is NO drop-and-continue mode; the header's
//!    `overrun_policy` word carries [`OverrunPolicy`] and this remains the
//!    detection path under BOTH policies (a backpressure wait that TIMES OUT falls
//!    back to lapping precisely so that this loud detection still fires).
//! 3. **Zero-copy drain, torn-drain detected after the fact.** The consumer
//!    exposes the unread region as up-to-2 raw byte slices pointing INTO the
//!    mapped ring pages ([`ShmRingConsumer::drain_slices`]), so `bagd` can later
//!    `writev` straight from ring memory. Because the producer never blocks, a lap
//!    can occur DURING a drain, so the protocol is: `drain_slices()` → caller
//!    consumes/writes the slices → [`ShmRingConsumer::commit`], which re-validates
//!    against the CURRENT write cursor and returns [`ShmRingError::Overrun`] if the
//!    producer lapped into the drained region while it was being read (torn data
//!    detected AFTER the fact — the recording fails loudly; this is the accepted
//!    semantics).
//! 4. **Real POSIX SHM on macOS AND Linux** (`#[cfg(unix)]`). Unlike
//!    [`crate::barrier`] / [`crate::doorbell`], whose non-Linux `imp` STUBS
//!    cross-process SHM with an in-process registry, this ring is GENUINELY
//!    cross-process on macOS too (the consumer `bagd` is a separate process and
//!    dev is Mac-only). macOS constraints handled: the SHM name is ≤ 31 chars
//!    (`PSHMNAMLEN`) — `/cer_rg_` + 16 hex = 24 — and `ftruncate` is called EXACTLY
//!    ONCE on a fresh `O_EXCL` segment (macOS `EINVAL`s on re-truncate).
//! 5. **Fixed-size binary records.** No serde, no text.
//! 6. **`BACKPRESSURE` is opt-in, and it is the ONE place the producer reads
//!    consumer state.** A checkpoint's node state is chunked into
//!    ~1.09 M records for a 500 MB anchor, so a fixed ring cannot hold it and a
//!    lapped anchor is a LOST anchor. The writer there is a short-lived `fork`
//!    child, not the hot loop, so blocking it is harmless — it only lengthens a
//!    child lifetime that is separately bounded and reported. Under
//!    [`OverrunPolicy::Backpressure`] the consumer publishes its committed read
//!    cursor into the header and [`ShmRingProducer::push`] WAITS (bounded
//!    spin-then-sleep, never a busy spin and never an unbounded block) while the
//!    next write would overwrite an unread record. **The wait is BOUNDED and its
//!    expiry LAPS rather than dropping**: dropping a record would be silent, while
//!    lapping is caught by contract 2 and fails the recording loudly. Every wait is
//!    counted ([`ShmRingProducer::backpressure_waits`] /
//!    [`ShmRingProducer::backpressure_wait_timeouts`]) so the degradation is
//!    observable without a log (Principle #3) — the push path stays log-free, which
//!    also keeps it safe in a `fork` child, where a `tracing` dispatcher mutex held
//!    at fork time would deadlock.
//! 7. **The consumer count is policy-scoped.**
//!    Under [`OverrunPolicy::FailLoud`] a ring is ONE PRODUCER, N INDEPENDENT
//!    READERS. Every [`ShmRingConsumer`] maps its own view and keeps its read
//!    cursor as a LOCAL field ([`ShmRingConsumer::read_cursor`]);
//!    the private `publish_read_cursor` is gated on `Backpressure` and so
//!    stores NOTHING on a `FailLoud` ring, and the producer never loads the word.
//!    Two readers of one `FailLoud` ring therefore do not steal each other's
//!    records and do not advance each other's cursors: EACH sees the FULL record
//!    stream from wherever it opened, and each detects ITS OWN overrun. That is
//!    the property relied on — the always-on window recorder drains a
//!    run's trace ring while a mid-run `cerulion bag record --run` attaches to the
//!    same ring, and neither is a party to the other. Pinned by
//!    `shm_ring_test::two_failloud_consumers_each_see_the_whole_stream`.
//!
//!    The cost is DUPLICATION, and it is a property of the ARTIFACT, not of the
//!    ring: N readers means the stream is read N times, so two readers feeding ONE
//!    bag (or one retention) write every record into it TWICE. That is why bagd
//!    refuses a ring declared twice on one recorder
//!    (`reject_duplicate_ring_names`) and why its trace retention is fed from
//!    INSIDE the one drain its writer thread already does — a per-artifact rule,
//!    not a ban on a second reader of the ring.
//!
//!    Under [`OverrunPolicy::Backpressure`] the ring IS single-consumer, and that
//!    is not a convention either: the read cursor lives in ONE header word that
//!    every consumer STORES to at open and at commit, and the producer WAITS on
//!    it, so a second consumer publishes over the first's position and the
//!    producer trusts whoever published last. Do not open two consumers on a
//!    backpressure ring.
//!
//! # Ownership / lifecycle
//!
//! [`ShmRingOwner::create`] `O_EXCL`-creates + sizes + maps the segment, writes
//! the header + manifest, and mints the single [`ShmRingProducer`]
//! ([`ShmRingOwner::producer`]). The owner OWNS the name and `shm_unlink`s it on
//! `Drop`; the underlying object (and any live consumer / producer mapping)
//! persists until the last `munmap` (POSIX unlink removes the NAME only), so an
//! owner dropping mid-run never invalidates a consumer that already opened it.
//! [`ShmRingConsumer::open`] STRICT-opens an existing segment (never creates),
//! maps its OWN view, validates the header, and reads the manifest; on `Drop` it
//! `munmap`s only.
//!
//! # Cross-process coordination
//!
//! [`ShmRingOwner::create`] fully writes the header, then `Release`-stores the
//! magic as its LAST write; the consumer's validation `Acquire`-loads the magic
//! FIRST. That single Release/Acquire edge on the magic word means even an open
//! that RACES the create either fails cleanly (magic not yet published →
//! `Validation` error) or observes a fully initialized header + manifest — never a
//! half-written one, on x86 AND weakly-ordered aarch64. Create-before-open remains
//! the INTENDED protocol (the name is handed to the consumer process out-of-band
//! after `create` returns, like [`crate::barrier`] / [`crate::doorbell`]); the
//! magic edge is the safety net that makes the racing case fail-clean rather than
//! undefined. The per-record data path carries its own `Release`/`Acquire` edge
//! via the write cursor.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on
//! its `pub mod shm_ring;` declaration in `lib.rs`, so no redundant inner
//! `#![cfg(unix)]` is needed here (clippy 1.93 `duplicated_attributes`).

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// The POSIX map substrate (`barrier`, `state_arm` and `credit` sit on the same
// one) plus the name fold this module used to write out itself.
use crate::shm_map::{create_exclusive, fnv1a64, unlink, unmap, MapError, MapStep, OpenedSegment};

/// Distinctive magic identifying a Cerulion SHM ring segment: ASCII `"CER_RING"`.
const MAGIC: u64 = 0x4345_525F_5249_4E47;

/// On-ring format version. Bumped on any header/layout change.
const VERSION: u32 = 1;

/// Bounded manifest region carried inline in the header — opaque bytes written
/// once at create, before any push (the trace layer uses it for the node-id
/// table). 64 KiB.
pub const MANIFEST_CAPACITY: usize = 64 * 1024;

/// Metadata bytes preceding the manifest region. Padded to one cache line (64 B)
/// so the manifest — and therefore the record data region that follows the
/// header — starts cache-line aligned.
const HEADER_CONTROL_BYTES: usize = 64;

/// Byte offset of the inline manifest region within the header.
const MANIFEST_OFFSET: usize = HEADER_CONTROL_BYTES;

/// Total header size (control bytes + manifest). A multiple of 64, so the record
/// data region that immediately follows the header is cache-line aligned when the
/// mapping is page-aligned (which `mmap` guarantees).
const HEADER_SIZE: usize = HEADER_CONTROL_BYTES + MANIFEST_CAPACITY;

/// The SHM ring header, placed at offset 0 of the mapped segment.
///
/// `#[repr(C)]`, atomics + fixed-size fields only, so it maps identically across
/// processes. NEVER instantiated on the stack (it embeds the 64 KiB manifest) —
/// only ever accessed through a pointer into the mapped segment. Explicit padding
/// (`degraded_sections`, which was `_pad_read_cursor`) plus repr(C)'s natural
/// 4-byte hole after `overrun_policy`
/// keep `generation`/`write_cursor`/`read_cursor` 8-aligned and `manifest` at
/// offset 64; the `const _` asserts below pin every load-bearing offset.
#[repr(C)]
struct ShmRingHeader {
    /// [`MAGIC`] once fully initialized. `Release`-stored as the LAST write at
    /// create and `Acquire`-loaded FIRST by the consumer's validation, so a racing
    /// open either fails clean (magic unset) or sees the whole header (see the
    /// module doc §Cross-process coordination). Same size/align as a plain `u64`,
    /// so the pinned field offsets are unchanged.
    magic: AtomicU64,
    /// [`VERSION`].
    version: u32,
    /// Size of each record in bytes (> 0).
    record_size: u32,
    /// Number of record slots — a power of two (enforced at create).
    capacity: u32,
    /// The producer's process rank (one ring per process; Tier-0 rank 0).
    rank: u32,
    /// The ring's [`OverrunPolicy`] as a wire discriminant (0 = `FailLoud`,
    /// 1 = `Backpressure`). Written once at create; read once at open.
    overrun_policy: u32,
    // repr(C) inserts 4 bytes of padding here so `generation` is 8-aligned.
    /// Create-generation — strictly increases per create within a process and
    /// differs across process restarts (restart detection). See [`new_generation`].
    generation: u64,
    /// Monotonic count of records the producer has published. `Release`-stored by
    /// the producer, `Acquire`-loaded by the consumer — the single synchronization
    /// edge that publishes slot writes.
    write_cursor: AtomicU64,
    /// Number of valid bytes in `manifest`.
    manifest_len: u32,
    /// Which manifest sections the degrade ladder dropped,
    /// as a bit set (see `trace_ring::DegradedSections`). `0` = nothing dropped.
    ///
    /// It lives in what was `_pad_read_cursor`, following the precedent set one
    /// field down: the word keeps `read_cursor` 8-aligned exactly as the padding
    /// did, every pre-existing field keeps its pinned offset, and the segment
    /// size is unchanged.
    ///
    /// **Why it is in the HEADER and not in the manifest.** It has to survive the
    /// very thing it reports. The degrade ladder drops manifest SECTIONS when the
    /// encoding exceeds the budget — inputs included, at the bottom rung — so a
    /// marker inside the manifest body is a marker the bottom rung can drop, and
    /// the positional section parsers would read any trailing bytes as the next
    /// section's. A header word is outside all of that.
    ///
    /// **[`VERSION`] is deliberately NOT bumped**, for the same reason the read
    /// cursor below did not bump it: the change is additive into zero-filled padding, an OLD
    /// writer leaves it 0, and 0 is the conservative reading — "this recorder
    /// claims nothing was dropped", which is exactly what an older recorder
    /// means. A bump would refuse every old ring whose bytes did not change.
    degraded_sections: AtomicU32,
    /// The CONSUMER's committed read cursor, published for
    /// the backpressure producer to wait on.
    ///
    /// It lives in what was `_pad_manifest`, so EVERY pre-existing field keeps its
    /// pinned offset and the segment size is unchanged. Under
    /// [`OverrunPolicy::FailLoud`] neither side ever touches it (the word stays at
    /// its create-time zero), which is what makes the trace ring byte-identical to
    /// its earlier self; under [`OverrunPolicy::Backpressure`] the consumer
    /// `Release`-stores it whenever it sets or advances its cursor and the producer
    /// `Acquire`-loads it.
    ///
    /// **[`VERSION`] is deliberately NOT bumped.** The change is additive into
    /// zero-filled padding, so an OLD binary on either side reads 0 — and 0 is the
    /// conservative reading: a backpressure producer facing a consumer that never
    /// publishes waits and then laps, which contract 2 reports loudly. A bump would
    /// instead REFUSE every old consumer of a trace ring whose bytes did not change.
    read_cursor: AtomicU64,
    /// Opaque manifest bytes (first `manifest_len` valid), written once at create.
    manifest: [u8; MANIFEST_CAPACITY],
}

// Layout is a cross-process + on-disk-bag format contract: pin size, alignment,
// and every load-bearing field offset so a field reorder / type change fails the
// build rather than silently breaking the mapping.
const _: () = assert!(
    std::mem::size_of::<ShmRingHeader>() == HEADER_SIZE,
    "ShmRingHeader size must equal HEADER_SIZE (control + manifest)"
);
const _: () = assert!(
    std::mem::align_of::<ShmRingHeader>() == 8,
    "ShmRingHeader must be 8-aligned so its atomics are validly aligned in SHM"
);
const _: () = assert!(
    HEADER_SIZE.is_multiple_of(64),
    "data region must be cache-line aligned"
);
/// Byte offsets of the validated header fields, defined via `offset_of!` so
/// out-of-crate corruption tests stay mechanically linked to the real layout
/// (the `const _` asserts below pin the format-contract values).
pub const HEADER_OFF_RECORD_SIZE: usize = std::mem::offset_of!(ShmRingHeader, record_size);
/// Byte offset of `capacity` in the header. See [`HEADER_OFF_RECORD_SIZE`].
pub const HEADER_OFF_CAPACITY: usize = std::mem::offset_of!(ShmRingHeader, capacity);
/// Byte offset of `manifest_len` in the header. See [`HEADER_OFF_RECORD_SIZE`].
pub const HEADER_OFF_MANIFEST_LEN: usize = std::mem::offset_of!(ShmRingHeader, manifest_len);
/// Byte offset of the [`OverrunPolicy`] wire discriminant. See
/// [`HEADER_OFF_RECORD_SIZE`]; exported so a test can read the ring's declared
/// mode straight off the mapped bytes.
pub const HEADER_OFF_OVERRUN_POLICY: usize = std::mem::offset_of!(ShmRingHeader, overrun_policy);
/// Byte offset of the consumer's published read cursor. See
/// [`HEADER_OFF_RECORD_SIZE`]; exported so a test can prove a `FailLoud` ring
/// never writes it.
pub const HEADER_OFF_READ_CURSOR: usize = std::mem::offset_of!(ShmRingHeader, read_cursor);

/// The degraded-sections word's offset, pinned like every other
/// load-bearing field.
///
/// It took over `_pad_read_cursor`, so the pin is what proves the claim the field's
/// own doc makes — that it reuses padding and moves NOTHING. Without it, a future
/// reorder could shift `read_cursor` (and with it the whole manifest region) and
/// the size/align asserts would still pass.
pub const HEADER_OFF_DEGRADED_SECTIONS: usize =
    std::mem::offset_of!(ShmRingHeader, degraded_sections);

const _: () = assert!(
    HEADER_OFF_DEGRADED_SECTIONS == HEADER_OFF_MANIFEST_LEN + 4,
    "degraded_sections must sit immediately after manifest_len, in what was \
     `_pad_read_cursor` — if it moved, every offset below it moved too"
);
const _: () = assert!(
    HEADER_OFF_READ_CURSOR == HEADER_OFF_DEGRADED_SECTIONS + 4,
    "…and read_cursor must still follow it, 8-aligned"
);

const _: () = assert!(std::mem::offset_of!(ShmRingHeader, magic) == 0);
const _: () = assert!(std::mem::offset_of!(ShmRingHeader, version) == 8);
const _: () = assert!(HEADER_OFF_RECORD_SIZE == 12);
const _: () = assert!(HEADER_OFF_CAPACITY == 16);
const _: () = assert!(std::mem::offset_of!(ShmRingHeader, rank) == 20);
const _: () = assert!(HEADER_OFF_OVERRUN_POLICY == 24);
const _: () = assert!(std::mem::offset_of!(ShmRingHeader, generation) == 32);
const _: () = assert!(std::mem::offset_of!(ShmRingHeader, write_cursor) == 40);
const _: () = assert!(HEADER_OFF_MANIFEST_LEN == 48);
// The read cursor took over the tail of the old `_pad_manifest`, so it
// must land at 56 and leave `manifest` exactly where it was.
const _: () = assert!(HEADER_OFF_READ_CURSOR == 56);
const _: () = assert!(std::mem::offset_of!(ShmRingHeader, manifest) == MANIFEST_OFFSET);

/// What a producer does when its next `push` would LAP the consumer.
///
/// Chosen ONCE, at [`ShmRingOwner::create_with_policy`], and written into the
/// header so a cross-process consumer can read which contract the ring is under.
/// [`ShmRingOwner::create`] — the path the shipped trace ring takes — always
/// selects [`FailLoud`](Self::FailLoud), which is why the backpressure mode leaves the trace
/// ring byte-unchanged.
///
/// It is a MODE, not a knob: the two arms differ in whether the producer may read
/// consumer state at all (module doc contracts 1 and 6), so a ring cannot change
/// its mind mid-run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrunPolicy {
    /// The original contract, unchanged and still the default: the producer
    /// is WAIT-FREE and never reads consumer state, so a slow or dead consumer can
    /// never stall it. A lap is detected AFTER the fact by the consumer
    /// ([`ShmRingError::Overrun`]).
    FailLoud,
    /// The producer WAITS (bounded spin-then-sleep) rather than
    /// overwriting a record the consumer has not committed, and the consumer
    /// publishes its read cursor for it to wait on. For a writer that can afford to
    /// wait — a checkpoint `fork` child streaming ~1.09 M chunked records — and
    /// never for a hot loop.
    Backpressure,
}

impl OverrunPolicy {
    /// The header discriminant. Frozen: these values are on a cross-process wire.
    pub fn as_wire(self) -> u32 {
        match self {
            Self::FailLoud => 0,
            Self::Backpressure => 1,
        }
    }

    /// Decode a header discriminant. `None` for anything this build does not know,
    /// so a reader can say "a policy I cannot honour" rather than silently
    /// defaulting to the wait-free arm and lapping a writer that expected to wait.
    pub fn from_wire(raw: u32) -> Option<Self> {
        match raw {
            0 => Some(Self::FailLoud),
            1 => Some(Self::Backpressure),
            _ => None,
        }
    }

    /// `true` for [`Backpressure`](Self::Backpressure).
    pub fn is_backpressure(self) -> bool {
        matches!(self, Self::Backpressure)
    }
}

/// Default per-push ceiling on a [`OverrunPolicy::Backpressure`] wait, overridable
/// via [`ShmRingProducer::set_backpressure_wait_timeout`].
///
/// Generous on purpose: it bounds a WEDGE (a consumer that died mid-anchor), not
/// the ordinary case, and the two errors are asymmetric — expiring EARLY laps a
/// record the consumer would have read a moment later, while expiring LATE only
/// lengthens a child whose lifetime is separately watchdogged.
pub const DEFAULT_BACKPRESSURE_WAIT_TIMEOUT: Duration = Duration::from_secs(5);

/// Bounded spin rounds before a backpressure wait starts sleeping. A consumer
/// mid-`commit` frees a slot in nanoseconds, so the spin covers the common case
/// without a syscall; everything past it sleeps, so this is never a busy spin.
const BACKPRESSURE_SPIN_ROUNDS: u32 = 256;

/// Recheck cadence once a backpressure wait has stopped spinning.
const BACKPRESSURE_POLL_INTERVAL: Duration = Duration::from_micros(50);

/// PURE: must a producer at `local_write` WAIT before writing, given the
/// consumer's published `read_cursor` and the ring `capacity`?
///
/// Writing at `local_write` leaves the unread span at `local_write + 1 - read`,
/// and the consumer calls a lap `unread > capacity` — so a write is safe exactly
/// while `local_write - read < capacity`, and the producer must wait AT equality
/// (the slot it is about to take is the oldest uncommitted record's).
///
/// `saturating_sub` because a published cursor AHEAD of the local write is not a
/// state this ring can reason about (a corrupt header, or a `fork` child that
/// advanced the shared cursor while this copy did not — exactly what
/// [`ShmRingProducer::resync_after_fork`] exists to repair). Saturating to 0 means
/// "do not wait", which is the arm that cannot deadlock; the resulting write is
/// then subject to the ordinary loud overrun detection.
fn backpressure_must_wait(local_write: u64, read_cursor: u64, capacity: u64) -> bool {
    local_write.saturating_sub(read_cursor) >= capacity
}

/// PURE: how many records a producer at `local_write` may push RIGHT NOW without
/// waiting, given the consumer's published `read_cursor` and the `capacity`.
///
/// The exact complement of [`backpressure_must_wait`]: that predicate is this
/// count reaching zero, and the two are pinned against each other so they cannot
/// drift into disagreeing about the boundary.
///
/// # Why a count is worth having at all
///
/// A `BACKPRESSURE` ring's `push` BLOCKS when the ring is full, and on timeout it
/// LAPS — which fails the whole recording. That is the right behaviour for a fork
/// CHILD, which nobody is waiting for. It is the wrong behaviour on the node
/// thread: the checkpoint carrier's inline half pushes at a step boundary, so a
/// recorder that has fallen behind would stall the robot's control loop for the
/// wait timeout, and under the multi-process barrier a peer would then be blocked
/// behind it. Neither the arena's BYTE bound nor `INLINE_SAFE` covers this — the
/// ring's fullness is not a property of the state being encoded.
///
/// So the boundary asks FIRST and declines the anchor if the answer is too small,
/// which costs one skipped cadence and no wall time at all.
///
/// # Why asking first is SOUND and not a TOCTOU
///
/// The answer is a LOWER bound that can only grow: `capacity` is fixed, the
/// producer is the only writer of `local_write` (SPSC), and the consumer only ever
/// ADVANCES `read_cursor`. So a concurrent consumer can make this answer STALE
/// only by making it too SMALL. "There is room for N" therefore stays true from
/// the moment it is observed until this producer itself uses it — which is exactly
/// the direction a precheck needs, and the reason no lock or handshake is
/// involved.
///
/// `saturating_sub` for the same reason [`backpressure_must_wait`] uses it: a
/// published cursor ahead of the local write is an un-modellable state (a corrupt
/// header, or a `fork` child that advanced the shared cursor while this copy did
/// not), and saturating to `capacity` here means "plenty of room", the arm that
/// cannot deadlock — the write is then subject to the ordinary loud overrun
/// detection, exactly as the wait path leaves it.
fn backpressure_free_records(local_write: u64, read_cursor: u64, capacity: u64) -> u64 {
    capacity.saturating_sub(local_write.saturating_sub(read_cursor))
}

/// Errors from the generic SHM ring. Matches the repo `thiserror` +
/// `#[non_exhaustive]` convention.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ShmRingError {
    /// The segment could not be created / sized / mapped.
    #[error("shm_ring create failed for '{name}': {reason}")]
    Create {
        /// The POSIX SHM object name involved.
        name: String,
        /// Underlying cause (syscall errno or a parameter-validation message).
        reason: String,
    },
    /// An existing segment could not be opened / mapped.
    #[error("shm_ring open failed for '{name}': {reason}")]
    Open {
        /// The POSIX SHM object name involved.
        name: String,
        /// Underlying cause (syscall errno or "does not exist").
        reason: String,
    },
    /// The mapped header failed validation (magic/version/record_size/capacity or
    /// segment size).
    #[error("shm_ring validation failed for '{name}': {reason}")]
    Validation {
        /// The POSIX SHM object name involved.
        name: String,
        /// What was wrong with the header.
        reason: String,
    },
    /// [`ShmRingConsumer::commit`] was asked to advance past the producer's write
    /// cursor — a CONSUMER bug (committing more records than were drained). The
    /// cursor is NOT advanced: silently accepting it would push `read_cursor` past
    /// `write_cursor`, making every later `write - read` underflow into a spurious,
    /// astronomically-large [`Overrun`](Self::Overrun).
    #[error("shm_ring commit({requested}) exceeds the available record count ({available}) — committing more than was drained is a consumer bug; the read cursor was not advanced")]
    CommitBeyondAvailable {
        /// The record count passed to `commit`.
        requested: u64,
        /// How many records were actually available (`write_cursor - read_cursor`).
        available: u64,
    },
    /// The consumer was lapped by the producer — records were irrecoverably
    /// overwritten. The recording layer above treats this as a hard, loud failure
    /// (there is no drop-and-continue mode).
    #[error("shm_ring overrun: consumer lapped by producer — {records_lost} record(s) irrecoverably lost (read_cursor={read_cursor}, write_cursor={write_cursor}, capacity={capacity} records). The recorder fell too far behind; recording is unrecoverable.")]
    Overrun {
        /// How many records were overwritten before the consumer could read them.
        records_lost: u64,
        /// The consumer's read cursor when the lap was detected.
        read_cursor: u64,
        /// The producer's write cursor when the lap was detected.
        write_cursor: u64,
        /// The ring capacity in records.
        capacity: u64,
    },
}

/// Result alias for SHM ring operations.
pub type ShmRingResult<T> = Result<T, ShmRingError>;

/// Derive the POSIX SHM object name for `tag`: `/cer_rg_<fnv1a64(tag):016x>`.
///
/// Fixed-length hex (mirrors [`crate::barrier`]'s `cer_bar_` / [`crate::doorbell`]'s
/// `cer_db_`) keeps the name prefix-free vs those and vs iceoryx2's names (there is
/// a known iceoryx2 hazard with string-prefix collisions) and ≤ 31 chars for macOS
/// (`/cer_rg_` = 8 + 16 = 24). Pure (no I/O) — hermetically testable on any OS.
pub fn ring_shm_name(tag: &str) -> String {
    format!("/cer_rg_{:016x}", fnv1a64(tag.as_bytes()))
}

/// Does a POSIX SHM object of this name EXIST?
///
/// The checkpoint recorder sweeps a rank space for rings whose names
/// it DERIVES, and "is this name taken?" is a different question from "can I open
/// this ring?" — a name that exists but whose segment is truncated or wrongly
/// configured is a rank that ANSWERED and must be reported, not a rank that is
/// absent. Answering the two with one fallible open would force the caller to
/// classify by matching on an errno rendered into a string, which is exactly the
/// kind of coupling that breaks silently.
///
/// One `shm_open(O_RDONLY)` + `close`, no mapping. `false` for ANY failure — the
/// only caller treats "cannot even be seen" as absent, which is the conservative
/// reading (it can end a sweep early, never fabricate a rank).
pub fn shm_object_exists(shm_name: &str) -> bool {
    let Ok(name) = std::ffi::CString::new(shm_name) else {
        return false;
    };
    // SAFETY: FFI open of a named SHM object with a valid NUL-terminated name.
    let fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDONLY, 0) };
    if fd < 0 {
        return false;
    }
    // SAFETY: closing the descriptor this call just opened.
    unsafe { libc::close(fd) };
    true
}

/// The largest power of two ≤ `n` (0 for `n == 0`).
///
/// Lives HERE, in the generic layer, because "capacity must be a power of two" is
/// THIS module's rule (enforced in [`ShmRingOwner::create_with_policy`]) and every
/// record layer above derives its capacity from a byte budget by applying it — the
/// trace ring's [`crate::trace_ring::default_capacity_records`] and the state ring's
/// [`crate::state_ring::capacity_records_for_bytes`]. One rule, one implementation.
pub(crate) fn prev_power_of_two(n: u64) -> u64 {
    if n == 0 {
        0
    } else if n.is_power_of_two() {
        n
    } else {
        n.next_power_of_two() >> 1
    }
}

/// Mint a fresh create-generation.
///
/// Seeded from wall-clock nanos so two DIFFERENT processes (a restart) get
/// different generations — restart detection — and forced strictly-increasing
/// within one process (via a process-global high-water mark) so even two creates
/// in the same nanosecond, or a backward clock step, still yield distinct values.
/// This is a CREATE-path helper only; it never runs on the wait-free push path.
fn new_generation() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    static LAST_GEN: AtomicU64 = AtomicU64::new(0);
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let mut prev = LAST_GEN.load(Ordering::Relaxed);
    loop {
        let cand = now.max(prev.wrapping_add(1));
        match LAST_GEN.compare_exchange_weak(prev, cand, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return cand,
            Err(p) => prev = p,
        }
    }
}

/// An owned `mmap` of a SHM segment. `Drop` `munmap`s. Shared by the owner and its
/// producer (same process, one mapping) via `Arc`; a consumer holds its OWN
/// `Mapping` (a separate `mmap` of the same object).
#[derive(Debug)]
struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: the mapping is a `MAP_SHARED` region accessed only through the SPSC
// discipline (the `write_cursor` atomic gives the cross-thread/-process ordering
// for the data slots; the header metadata is write-once-before-open). Moving/
// sharing the handle across threads is sound. Raw ptr + len are otherwise inert.
unsafe impl Send for Mapping {}
unsafe impl Sync for Mapping {}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: unmap exactly the region this `Mapping` was built from; the
        // handle owns it and nothing references it after this.
        unsafe {
            unmap(self.ptr as *mut libc::c_void, self.len);
        }
    }
}

/// Render a [`MapError`] into the `reason` text this module's CREATE arm has
/// always produced. `total` is the size the failing call was given.
///
/// The step→text mapping lives at the call sites' own layer (rather than on
/// `MapStep`) because the two arms disagree: a create's `shm_open` names its
/// `O_CREAT|O_EXCL` flags, an open's names `O_RDWR`.
fn create_reason(e: MapError, total: usize) -> String {
    match e.step {
        MapStep::ShmOpen => format!("shm_open(O_CREAT|O_EXCL): {}", e.err),
        MapStep::Ftruncate => format!("ftruncate({total}): {}", e.err),
        MapStep::Mmap => format!("mmap({total}): {}", e.err),
        // Unreachable on the create path (it reads no size), rendered rather
        // than `unreachable!`'d so a future step can never panic a recorder.
        MapStep::Fstat => format!("fstat: {}", e.err),
    }
}

/// Total mapped size for a ring of `capacity` records of `record_size` bytes.
fn segment_size(record_size: u32, capacity: u32) -> usize {
    HEADER_SIZE + (record_size as usize) * (capacity as usize)
}

/// The APPARENT size of a ring segment: what `ftruncate` reserves
/// and what a `/dev/shm` free-space check must be measured against.
///
/// Public because a caller that wants to know whether a ring will FIT has to ask
/// somebody, and the alternative is a second copy of this layout arithmetic in
/// the CLI — the class of duplication the single-copy rule exists to refuse. It is the same
/// expression `segment_size` uses, so the two cannot drift.
///
/// APPARENT, not resident: creation is `ftruncate` + `mmap` with no populate and
/// no memset, and slots are written strictly sequentially, so a fresh ring costs
/// one page and converges to this figure only at its first lap.
#[must_use]
pub fn apparent_segment_bytes(record_size: u32, capacity: u32) -> u64 {
    segment_size(record_size, capacity) as u64
}

/// Interpret `base` as the ring header. SAFETY: `base` must point at the start of
/// a mapped ring segment of at least [`HEADER_SIZE`] bytes.
unsafe fn header<'a>(base: *const u8) -> &'a ShmRingHeader {
    &*(base as *const ShmRingHeader)
}

// ===========================================================================
// Owner (create side)
// ===========================================================================

/// The owner of a SHM ring segment: creates + sizes + maps it, writes the header
/// and manifest, and mints the single [`ShmRingProducer`]. `Drop` `shm_unlink`s the
/// NAME (the mapping — and any live consumer / producer — survives until the last
/// `munmap`).
#[derive(Debug)]
#[must_use = "the owner shm_unlinks the ring name on drop — bind it to a named local for the desired lifetime"]
pub struct ShmRingOwner {
    mapping: Arc<Mapping>,
    name: std::ffi::CString,
    name_str: String,
    record_size: u32,
    capacity: u32,
    rank: u32,
    generation: u64,
    policy: OverrunPolicy,
    /// Single-producer guard: the producer can be minted exactly once.
    producer_taken: bool,
}

impl ShmRingOwner {
    /// The owner's mapped region, as `(base, len)`.
    ///
    /// Exists because a `fork` child must not inherit a ring
    /// its parent is producing into, so the carrier needs the exact bounds to exclude.
    /// Returns the region this handle mapped — never a consumer's separate mapping of
    /// the same object.
    pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
        (self.mapping.ptr as *mut std::ffi::c_void, self.mapping.len)
    }

    /// Create + map a fresh SHM ring for `name_tag` holding `capacity` records of
    /// `record_size` bytes each, tagged with the producer's `rank`, and write
    /// `manifest` (opaque bytes ≤ [`MANIFEST_CAPACITY`]) into the header.
    ///
    /// `capacity` MUST be a non-zero power of two and `record_size` MUST be > 0.
    /// A pre-existing orphan of the same name (crashed prior run) is `shm_unlink`ed
    /// first so the `O_EXCL` create yields a fresh, zero-filled segment.
    ///
    /// The ring is [`OverrunPolicy::FailLoud`] — the wait-free contract.
    /// This is the ONLY constructor the trace ring uses, which is what keeps that
    /// ring byte- and behaviour-identical; a caller that needs the
    /// producer to WAIT asks for it explicitly via
    /// [`create_with_policy`](Self::create_with_policy).
    pub fn create(
        name_tag: &str,
        record_size: u32,
        capacity: u32,
        rank: u32,
        manifest: &[u8],
    ) -> ShmRingResult<Self> {
        Self::create_degraded(name_tag, record_size, capacity, rank, manifest, 0)
    }

    /// [`Self::create`] carrying the degraded-sections marker
    /// (see [`Self::create_with_policy_and_degraded`] for why it must be written at
    /// create rather than after it).
    pub fn create_degraded(
        name_tag: &str,
        record_size: u32,
        capacity: u32,
        rank: u32,
        manifest: &[u8],
        degraded_sections: u32,
    ) -> ShmRingResult<Self> {
        Self::create_with_policy_and_degraded(
            name_tag,
            record_size,
            capacity,
            rank,
            manifest,
            OverrunPolicy::FailLoud,
            degraded_sections,
        )
    }

    /// [`create`](Self::create), with the ring's [`OverrunPolicy`] chosen
    /// explicitly.
    ///
    /// The policy is written into the header, so it is what a cross-process
    /// consumer reads back and what decides whether that consumer publishes its
    /// read cursor at all.
    pub fn create_with_policy(
        name_tag: &str,
        record_size: u32,
        capacity: u32,
        rank: u32,
        manifest: &[u8],
        policy: OverrunPolicy,
    ) -> ShmRingResult<Self> {
        Self::create_with_policy_and_degraded(
            name_tag,
            record_size,
            capacity,
            rank,
            manifest,
            policy,
            0,
        )
    }

    /// [`Self::create_with_policy`] plus the degraded-sections
    /// marker, written BEFORE `MAGIC` is published.
    ///
    /// The marker has to be in place before the ring is visible, not after. `MAGIC`
    /// is `Release`-stored as the LAST write at create precisely so a racing open
    /// sees either nothing or a complete header — so a marker stamped AFTER create
    /// returns leaves a window in which a consumer reads a valid ring whose marker
    /// still says `0`, "nothing dropped". That is not a harmless window: it is the
    /// exact reading that makes `bagd` write neither manifest key for a degraded
    /// rank, which is the silent derived-rim bug the marker exists to prevent, and
    /// `cerulion bagd --run`'s attach path opens rings it did not create.
    ///
    /// So the degrade ladder accumulates its bits on the way DOWN and the ring is
    /// created once, already marked.
    #[allow(clippy::too_many_arguments)]
    pub fn create_with_policy_and_degraded(
        name_tag: &str,
        record_size: u32,
        capacity: u32,
        rank: u32,
        manifest: &[u8],
        policy: OverrunPolicy,
        degraded_sections: u32,
    ) -> ShmRingResult<Self> {
        let name_str = ring_shm_name(name_tag);
        // ---- parameter validation (before any syscall) ----
        if record_size == 0 {
            return Err(ShmRingError::Create {
                name: name_str,
                reason: "record_size must be > 0".to_string(),
            });
        }
        if capacity == 0 {
            return Err(ShmRingError::Create {
                name: name_str,
                reason: "capacity must be > 0".to_string(),
            });
        }
        if !capacity.is_power_of_two() {
            return Err(ShmRingError::Create {
                name: name_str,
                reason: format!("capacity {capacity} must be a power of two"),
            });
        }
        if manifest.len() > MANIFEST_CAPACITY {
            return Err(ShmRingError::Create {
                name: name_str,
                reason: format!(
                    "manifest {} bytes exceeds MANIFEST_CAPACITY {MANIFEST_CAPACITY}",
                    manifest.len()
                ),
            });
        }

        let name = std::ffi::CString::new(name_str.clone()).map_err(|e| ShmRingError::Create {
            name: name_str.clone(),
            reason: format!("invalid shm name: {e}"),
        })?;
        let total = segment_size(record_size, capacity);

        // Unlink-first (clear a crashed-prior-run orphan so the create yields a
        // FRESH zero-filled object) + `O_EXCL` create + `ftruncate` EXACTLY ONCE
        // (macOS EINVALs on a re-truncate) + `mmap` — all of it, cleanup arms
        // included, in `shm_map::create_exclusive`.
        let mapping = match create_exclusive(&name, total) {
            Ok(addr) => Mapping {
                ptr: addr as *mut u8,
                len: total,
            },
            Err(e) => {
                return Err(ShmRingError::Create {
                    name: name_str,
                    reason: create_reason(e, total),
                })
            }
        };

        let generation = new_generation();
        // ---- write the header + manifest into the fresh (zero-filled) segment ----
        let base = mapping.ptr;
        // SAFETY: `base` is a freshly-mapped, exclusively-owned segment ≥ `total`
        // bytes; writing the header fields + copying the manifest is in-bounds.
        unsafe {
            let hdr = base as *mut ShmRingHeader;
            (*hdr).version = VERSION;
            (*hdr).record_size = record_size;
            (*hdr).capacity = capacity;
            (*hdr).rank = rank;
            (*hdr).overrun_policy = policy.as_wire();
            (*hdr).generation = generation;
            (*hdr).write_cursor.store(0, Ordering::Relaxed);
            (*hdr).read_cursor.store(0, Ordering::Relaxed);
            // The degrade ladder's accumulated bits, written
            // HERE — before `MAGIC` below publishes the header — so no consumer can
            // ever observe a valid ring whose marker is not yet true. `0` is
            // "nothing dropped", which is what every non-degrading create passes.
            (*hdr)
                .degraded_sections
                .store(degraded_sections, Ordering::Relaxed);
            (*hdr).manifest_len = manifest.len() as u32;
            std::ptr::copy_nonoverlapping(
                manifest.as_ptr(),
                base.add(MANIFEST_OFFSET),
                manifest.len(),
            );
            // Magic is Release-stored LAST; the consumer's validation Acquire-loads
            // it FIRST. That edge guarantees — on weakly-ordered targets too — that
            // an open racing this create either sees an unset magic (fails
            // validation cleanly) or sees every header/manifest write above.
            // Create-before-open remains the intended protocol (module doc).
            (*hdr).magic.store(MAGIC, Ordering::Release);
        }

        tracing::debug!(
            name = %name_str,
            record_size,
            capacity,
            rank,
            generation,
            manifest_len = manifest.len(),
            overrun_policy = ?policy,
            "shm_ring created"
        );

        Ok(Self {
            mapping: Arc::new(mapping),
            name,
            name_str,
            record_size,
            capacity,
            rank,
            generation,
            policy,
            producer_taken: false,
        })
    }

    /// Mint the single [`ShmRingProducer`]. Returns `None` if a producer has already
    /// been minted (single-producer contract — one producer per ring).
    pub fn producer(&mut self) -> Option<ShmRingProducer> {
        if self.producer_taken {
            return None;
        }
        self.producer_taken = true;
        let base = self.mapping.ptr;
        Some(ShmRingProducer {
            _mapping: Arc::clone(&self.mapping),
            header: base as *const ShmRingHeader,
            data: unsafe { base.add(HEADER_SIZE) },
            record_size: self.record_size as usize,
            capacity: self.capacity as u64,
            local_write: 0,
            policy: self.policy,
            wait_timeout: DEFAULT_BACKPRESSURE_WAIT_TIMEOUT,
            waits: 0,
            wait_nanos: 0,
            wait_timeouts: 0,
        })
    }

    /// The POSIX SHM object name — hand this to the consumer process.
    pub fn name(&self) -> &str {
        &self.name_str
    }

    /// The per-record size in bytes.
    pub fn record_size(&self) -> u32 {
        self.record_size
    }

    /// The ring capacity in records.
    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    /// The producer rank recorded in the header.
    pub fn rank(&self) -> u32 {
        self.rank
    }

    /// The create-generation recorded in the header.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// The [`OverrunPolicy`] this ring was created under.
    pub fn overrun_policy(&self) -> OverrunPolicy {
        self.policy
    }
}

impl Drop for ShmRingOwner {
    fn drop(&mut self) {
        // Best-effort unlink of the name this owner created and owns — it
        // removes the NAME only; the object (and any live consumer / producer
        // mapping) persists until the last munmap.
        unlink(&self.name);
        tracing::debug!(name = %self.name_str, "shm_ring owner dropped (name unlinked)");
    }
}

// ===========================================================================
// Producer (push side)
// ===========================================================================

/// The single wait-free producer for a SHM ring. Minted once via
/// [`ShmRingOwner::producer`]. Holds an `Arc` to the mapping so it stays valid even
/// if the owner drops first.
#[derive(Debug)]
#[must_use = "a producer with no pushes records nothing"]
pub struct ShmRingProducer {
    _mapping: Arc<Mapping>,
    header: *const ShmRingHeader,
    data: *mut u8,
    record_size: usize,
    capacity: u64,
    /// The producer's LOCAL next-sequence counter. It OWNS this counter and only
    /// `Release`-stores it into the header on each push — it never re-reads the
    /// shared write cursor, except at the one explicit repair point
    /// [`ShmRingProducer::resync_after_fork`].
    local_write: u64,
    /// This ring's mode. Read on every push, so it is a field rather than a header
    /// load; it cannot change for the ring's life (see [`OverrunPolicy`]).
    policy: OverrunPolicy,
    /// Per-push ceiling on a backpressure wait. Stored regardless of policy — a
    /// `FailLoud` producer simply never enters the wait it bounds.
    wait_timeout: Duration,
    /// Pushes that had to wait for room (Principle #3).
    waits: u64,
    /// Total nanoseconds spent waiting for room.
    wait_nanos: u64,
    /// Waits that hit [`wait_timeout`](Self::set_backpressure_wait_timeout) and
    /// LAPPED anyway. Never silent loss: the write that follows is caught by the
    /// consumer's ordinary overrun detection.
    wait_timeouts: u64,
}

// SAFETY: exactly one producer exists per ring (single-producer contract) and it
// holds exclusive write access to the data slots + the write cursor. Moving it to
// another thread is sound; it is not `Sync` (push takes `&mut self`).
unsafe impl Send for ShmRingProducer {}

impl ShmRingProducer {
    /// Publish one record. Under [`OverrunPolicy::FailLoud`] this is WAIT-FREE:
    /// it copies `record` into slot `(local_write % capacity)` then
    /// `Release`-stores `local_write + 1` — no alloc, no lock, no syscall, no clock
    /// read, and no read of consumer state.
    ///
    /// Under [`OverrunPolicy::Backpressure`] it first WAITS while the write would
    /// overwrite an uncommitted record (module doc contract 6). The room check is
    /// one `Acquire` load and one compare, so a push that has room still allocates
    /// nothing, takes no lock and reads no clock; only a push that must actually
    /// wait reaches the clock and the sleep.
    ///
    /// # Panics
    ///
    /// Panics if `record.len() != record_size` — a caller bug, fail-loud in EVERY
    /// build mode. A `debug_assert` would let a release-mode short `record` feed
    /// `copy_nonoverlapping` a length past the caller's slice: an out-of-bounds
    /// READ (UB) reachable from a safe pub fn. The guard is one predictable
    /// register compare; the happy path stays wait-free and alloc-free (panic
    /// formatting runs only on the failure path — pinned by the zero-alloc test).
    pub fn push(&mut self, record: &[u8]) {
        assert_eq!(
            record.len(),
            self.record_size,
            "record length must equal the ring record_size"
        );
        if self.policy.is_backpressure() {
            self.await_room();
        }
        let idx = (self.local_write % self.capacity) as usize;
        // SAFETY: `idx < capacity`, so `idx * record_size` is within the data
        // region; the assert above guarantees the source slice is exactly
        // `record_size` bytes (no OOB read); source and destination do not
        // overlap (user buffer vs SHM slot).
        unsafe {
            std::ptr::copy_nonoverlapping(
                record.as_ptr(),
                self.data.add(idx * self.record_size),
                self.record_size,
            );
        }
        self.local_write += 1;
        // Release-store publishes the slot write to the consumer's Acquire load.
        // SAFETY: `header` points at the mapped header (kept alive by `_mapping`).
        unsafe {
            (*self.header)
                .write_cursor
                .store(self.local_write, Ordering::Release)
        };
    }

    /// Publish one record ONLY if it needs no wait — `true` when it was published,
    /// `false` when a [`OverrunPolicy::Backpressure`] ring had no room.
    ///
    /// The point of the refusal is WHERE it is called from. A `BACKPRESSURE` push
    /// blocks up to [`DEFAULT_BACKPRESSURE_WAIT_TIMEOUT`] and then LAPS, and some
    /// callers run on a robot's node thread at a step boundary — most sharply, the
    /// ones that publish a REFUSAL record after the room precheck already answered
    /// "not enough room". Those callers must be able to decline a record rather than
    /// wait for the thing they just measured as absent; without this they turn a
    /// recorder that fell behind into a stalled control loop, which is the exact
    /// failure [`free_records`](Self::free_records) was added to prevent.
    ///
    /// Under [`OverrunPolicy::FailLoud`] there is nothing to refuse — that ring never
    /// waits, by contract — so this is `push` and always `true`. The distinction is
    /// deliberate: this is "would this push WAIT?", never "will this push lap?".
    ///
    /// # Panics
    ///
    /// As [`push`](Self::push): a `record.len() != record_size` is a caller bug.
    pub fn try_push(&mut self, record: &[u8]) -> bool {
        // FIRST, before the room question. A caller bug must not be masked by ring
        // state: with the checks the other way round, a malformed record fed to a FULL
        // backpressure ring quietly returned `false` — indistinguishable from ordinary
        // backpressure — and only started panicking once the recorder drained enough
        // for the same bug to reach `push`. That makes the documented panic contract
        // conditional on a race, and hides the caller error for exactly as long as the
        // ring is under pressure.
        assert_eq!(
            record.len(),
            self.record_size,
            "record length must equal the ring record_size"
        );
        if self.policy.is_backpressure()
            && backpressure_must_wait(self.local_write, self.read_load(), self.capacity)
        {
            return false;
        }
        self.push(record);
        true
    }

    /// Total records this producer has published (its local write cursor).
    pub fn pushed(&self) -> u64 {
        self.local_write
    }

    /// This ring's [`OverrunPolicy`].
    pub fn overrun_policy(&self) -> OverrunPolicy {
        self.policy
    }

    /// Override the per-push backpressure wait ceiling (default
    /// [`DEFAULT_BACKPRESSURE_WAIT_TIMEOUT`]).
    ///
    /// It bounds ONE push's wait, not a run: a producer facing a dead consumer
    /// pays it once per record, each time lapping and each time counted. Settable
    /// on a `FailLoud` producer too, where it is inert because that producer never
    /// waits — stated rather than silently ignored.
    pub fn set_backpressure_wait_timeout(&mut self, timeout: Duration) {
        self.wait_timeout = timeout;
    }

    /// The per-push backpressure wait ceiling currently in force.
    pub fn backpressure_wait_timeout(&self) -> Duration {
        self.wait_timeout
    }

    /// How many pushes had to WAIT for room. Zero on a `FailLoud` producer, and
    /// zero on a backpressure producer whose consumer kept up.
    pub fn backpressure_waits(&self) -> u64 {
        self.waits
    }

    /// Total nanoseconds spent waiting for room, across all pushes.
    pub fn backpressure_wait_nanos(&self) -> u64 {
        self.wait_nanos
    }

    /// How many waits EXPIRED and lapped anyway — the degradation from "no loss"
    /// back to "loud loss". Nonzero means the consumer stopped keeping up for
    /// longer than [`backpressure_wait_timeout`](Self::backpressure_wait_timeout),
    /// and the records written past those waits are what the consumer will report
    /// as [`ShmRingError::Overrun`].
    pub fn backpressure_wait_timeouts(&self) -> u64 {
        self.wait_timeouts
    }

    /// Re-read the LOCAL write cursor from the header — the repair for a producer
    /// copy duplicated by `fork`.
    ///
    /// # The hazard
    ///
    /// `fork` duplicates this struct, `local_write` included, while the ring pages
    /// are `MAP_SHARED` and therefore NOT copied. So a child that pushes advances
    /// the SHARED write cursor while the parent's copy of `local_write` stays where
    /// it was. The parent's next `push` then writes at a STALE index — over the
    /// child's records — and `Release`-stores a cursor LOWER than the one already
    /// published, which corrupts the ring for every consumer: records vanish, and
    /// `write_cursor` goes backwards. [`ShmRingOwner`]'s `producer_taken` guard
    /// cannot see this; it is per-address-space, and `fork` duplicates it too.
    ///
    /// # The invariant this restores
    ///
    /// After the call, `local_write` equals the highest cursor ANY copy of this
    /// producer has published, so the next `push` takes a slot no published record
    /// occupies and stores a cursor that only ever moves forward. The `Acquire`
    /// load pairs with the child's `Release` store, so the parent also observes the
    /// child's slot bytes.
    ///
    /// # Contract
    ///
    /// The producer role is handed to the child for the child's lifetime: the
    /// parent must touch the ring NOT AT ALL between `fork` and reap, and call this
    /// on reap. Calling it while the child is still pushing resyncs to a cursor
    /// that is stale again the moment it is read.
    ///
    /// Returns the resynced cursor (also readable via [`pushed`](Self::pushed)).
    pub fn resync_after_fork(&mut self) -> u64 {
        // SAFETY: `header` points at the mapped header (kept alive by `_mapping`).
        let published = unsafe { (*self.header).write_cursor.load(Ordering::Acquire) };
        self.local_write = published;
        published
    }

    /// How many records this producer may push RIGHT NOW without blocking —
    /// `None` on a ring whose policy is not `BACKPRESSURE`.
    ///
    /// `None` is the by-construction half of contract 1 ("the producer NEVER
    /// reads consumer state"): under the default policy there is no consumer
    /// cursor to consult, so the question has no answer rather than a guessed
    /// one. `BACKPRESSURE` rings already read that cursor on every blocking
    /// push, so this exposes what they know rather than granting a new
    /// capability.
    ///
    /// See `backpressure_free_records` (this module's private pure helper —
    /// named in prose, since a doc LINK from a public item to a private one
    /// fails the docs gate) for why the answer is sound to act on: it is a lower
    /// bound a concurrent consumer can only raise. The hazard it exists to close
    /// is a blocking push on a robot's node thread.
    pub fn free_records(&self) -> Option<u64> {
        if self.policy != OverrunPolicy::Backpressure {
            return None;
        }
        Some(backpressure_free_records(
            self.local_write,
            self.read_load(),
            self.capacity,
        ))
    }

    /// `Acquire`-load the consumer's published read cursor. Consumer state — read
    /// ONLY on the backpressure path (module doc contracts 1 and 6).
    fn read_load(&self) -> u64 {
        // SAFETY: `header` points at the mapped header (kept alive by `_mapping`).
        unsafe { (*self.header).read_cursor.load(Ordering::Acquire) }
    }

    /// Wait until writing at `local_write` would not overwrite an uncommitted
    /// record, or until this push's wait ceiling expires.
    ///
    /// Bounded spin-then-sleep: a consumer mid-`commit` frees a slot in
    /// nanoseconds, so the spin phase covers the common case with no syscall, and
    /// every iteration past it SLEEPS — this is never a busy spin and never an
    /// unbounded block. On expiry it returns and lets the caller write anyway,
    /// which laps; that is deliberate (module doc contract 6) because the
    /// alternative — dropping the record — would be silent, while a lap is caught
    /// by the consumer and fails the recording loudly.
    ///
    /// Both syscalls it can reach (`clock_gettime` via `Instant`, `nanosleep` via
    /// `sleep`) are async-signal-safe, so this is usable in a `fork` child.
    fn await_room(&mut self) {
        if !backpressure_must_wait(self.local_write, self.read_load(), self.capacity) {
            return;
        }
        let start = Instant::now();
        let mut spins: u32 = 0;
        loop {
            if !backpressure_must_wait(self.local_write, self.read_load(), self.capacity) {
                break;
            }
            let elapsed = start.elapsed();
            if elapsed >= self.wait_timeout {
                self.wait_timeouts += 1;
                break;
            }
            if spins < BACKPRESSURE_SPIN_ROUNDS {
                spins += 1;
                std::hint::spin_loop();
            } else {
                std::thread::sleep(
                    BACKPRESSURE_POLL_INTERVAL.min(self.wait_timeout.saturating_sub(elapsed)),
                );
            }
        }
        self.waits += 1;
        self.wait_nanos += start.elapsed().as_nanos() as u64;
    }
}

// ===========================================================================
// Consumer (drain side)
// ===========================================================================

/// A consumer of a SHM ring. STRICT-opens an existing segment (never creates),
/// maps its OWN view, and maintains a LOCAL read cursor. `Drop` `munmap`s only
/// (it does not own the name).
///
/// **How many of these a ring may have is POLICY-SCOPED** (module doc contract
/// 7): exactly one on a [`OverrunPolicy::Backpressure`] ring, where the read
/// cursor is a shared header word a producer waits on; N INDEPENDENT readers on
/// a [`OverrunPolicy::FailLoud`] ring — every trace ring — where the cursor
/// below is local, nothing is published, and each reader sees the full stream
/// and detects its own overrun. The second arm is load-bearing: a run's
/// always-on window recorder and a mid-run `bag record --run` read one trace
/// ring without coordinating.
#[derive(Debug)]
#[must_use = "a consumer that is never drained reads nothing"]
pub struct ShmRingConsumer {
    _mapping: Mapping,
    header: *const ShmRingHeader,
    data: *const u8,
    record_size: usize,
    capacity: u64,
    /// The consumer's LOCAL read cursor (records consumed + committed).
    read_cursor: u64,
    // Immutable metadata snapshot read once at open.
    generation: u64,
    rank: u32,
    overrun_policy: u32,
    manifest: Vec<u8>,
    name_str: String,
}

// SAFETY: this consumer holds its OWN mapping and its OWN read cursor and only
// reads data slots the producer has published (via the Acquire load on the write
// cursor). Moving it to another thread is sound; not `Sync`. Note what this does
// NOT rest on: the number of consumers. Soundness here is per-instance, which is
// why a `FailLoud` ring can carry N of them (module doc contract 7) — the
// one-consumer rule is a BACKPRESSURE-cursor rule, not a `Send` premise.
unsafe impl Send for ShmRingConsumer {}

impl ShmRingConsumer {
    /// STRICT-open an existing SHM ring by its full object name (from
    /// [`ShmRingOwner::name`]). Never creates. Validates magic / version /
    /// record_size / capacity / segment size, then reads the manifest.
    pub fn open(shm_name: &str) -> ShmRingResult<Self> {
        let name = std::ffi::CString::new(shm_name).map_err(|e| ShmRingError::Open {
            name: shm_name.to_string(),
            reason: format!("invalid shm name: {e}"),
        })?;
        // STRICT open-existing: O_RDWR, NO O_CREAT — a missing object is an error.
        let seg = OpenedSegment::open(&name).map_err(|e| ShmRingError::Open {
            name: shm_name.to_string(),
            reason: format!("shm_open(O_RDWR): {}", e.err),
        })?;

        // fstat to LEARN the segment size (the owner ftruncated it) — this
        // module's rings are variably sized, which is why the open half of
        // `shm_map` is stepwise rather than one call.
        let total = seg.size().map_err(|e| ShmRingError::Open {
            name: shm_name.to_string(),
            reason: format!("fstat: {}", e.err),
        })? as usize;
        if total < HEADER_SIZE {
            // `seg`'s Drop closes the descriptor.
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!("segment {total} bytes is smaller than the header ({HEADER_SIZE})"),
            });
        }

        let ptr = seg.map_shared(total).map_err(|e| ShmRingError::Open {
            name: shm_name.to_string(),
            reason: format!("mmap({total}): {}", e.err),
        })?;
        let mapping = Mapping {
            ptr: ptr as *mut u8,
            len: total,
        };

        // ---- validate the header ----
        let base = mapping.ptr as *const u8;
        // SAFETY: `base` maps ≥ HEADER_SIZE bytes (checked above).
        let hdr = unsafe { header(base) };
        // Acquire pairs with the create-site's Release store of the magic (its
        // LAST write), so once the magic matches, every other header/manifest
        // field below is guaranteed visible — even on a racing open (module doc
        // §Cross-process coordination).
        let magic = hdr.magic.load(Ordering::Acquire);
        if magic != MAGIC {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!("bad magic 0x{magic:016x} (expected 0x{MAGIC:016x})"),
            });
        }
        if hdr.version != VERSION {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!("unsupported version {} (expected {VERSION})", hdr.version),
            });
        }
        let record_size = hdr.record_size;
        let capacity = hdr.capacity;
        if record_size == 0 {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: "record_size is 0".to_string(),
            });
        }
        if capacity == 0 || !capacity.is_power_of_two() {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!("capacity {capacity} is not a non-zero power of two"),
            });
        }
        let expected = segment_size(record_size, capacity);
        if total < expected {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!(
                    "segment {total} bytes < expected {expected} (header + {capacity}×{record_size})"
                ),
            });
        }
        let manifest_len = hdr.manifest_len as usize;
        if manifest_len > MANIFEST_CAPACITY {
            return Err(ShmRingError::Validation {
                name: shm_name.to_string(),
                reason: format!("manifest_len {manifest_len} exceeds MANIFEST_CAPACITY"),
            });
        }
        let generation = hdr.generation;
        let rank = hdr.rank;
        let overrun_policy = hdr.overrun_policy;
        // Copy the manifest out of the mapping (small, one-time).
        // SAFETY: `manifest_len ≤ MANIFEST_CAPACITY`, all within the header.
        let manifest =
            unsafe { std::slice::from_raw_parts(base.add(MANIFEST_OFFSET), manifest_len).to_vec() };

        let header_ptr = base as *const ShmRingHeader;
        // SAFETY: the data region starts at HEADER_SIZE and spans capacity×record_size.
        let data = unsafe { base.add(HEADER_SIZE) };

        tracing::debug!(
            name = %shm_name,
            record_size,
            capacity,
            rank,
            generation,
            "shm_ring opened by consumer"
        );

        let consumer = Self {
            _mapping: mapping,
            header: header_ptr,
            data,
            record_size: record_size as usize,
            capacity: capacity as u64,
            read_cursor: 0,
            generation,
            rank,
            overrun_policy,
            manifest,
            name_str: shm_name.to_string(),
        };
        // A backpressure producer waits on this word, so a consumer
        // publishes its cursor whenever it SETS it — here (0) as well as on every
        // advance. Publishing 0 at open is correct under the SPSC contract (one
        // consumer, starting at record 0) and is what lets a producer that filled
        // the ring before anyone attached make progress.
        consumer.publish_read_cursor();
        Ok(consumer)
    }

    /// STRICT-open an existing SHM ring **at the producer's CURRENT write
    /// cursor** — the mid-run attach seam.
    ///
    /// Identical to [`open`](Self::open) in every respect except the initial read
    /// cursor: this sets `read_cursor := write_cursor` at open, so the consumer
    /// sees ONLY records the producer commits AFTER the attach instant.
    ///
    /// # Why this exists
    ///
    /// [`open`](Self::open) starts at record 0, and
    /// [`drain_slices`](Self::drain_slices) treats `write_cursor - read_cursor >
    /// capacity` as [`ShmRingError::Overrun`]. A ring that has already LAPPED
    /// therefore cannot be opened-and-drained at all: the first drain fails hard.
    /// A recorder attaching to a long-running graph is exactly that case (a
    /// 1 kHz graph with 10 firing nodes laps the default ≈1 M-record trace ring in
    /// ≈95 s), so `open` is not merely lossy for it — it is unusable.
    ///
    /// # What the attach instant means (stated, not implied)
    ///
    /// Records committed BEFORE the attach are **unrecoverable and are NOT
    /// reported as loss** — they were never in this consumer's window. The
    /// caller is the only party that can know that and must say so in whatever
    /// artifact it produces (for the recorder: an `attached_mid_run` marker).
    /// From the attach point ON, accounting is UNCHANGED and exact: falling
    /// more than `capacity` records behind still raises
    /// [`ShmRingError::Overrun`] at the next [`drain_slices`](Self::drain_slices),
    /// and a producer lapping DURING a drain is still caught after the fact by
    /// [`commit`](Self::commit) — both measured from the attach cursor.
    ///
    /// The cursor is sampled with the same `Acquire` load the drain path uses,
    /// AFTER header validation, so a record committed between `shm_open` and the
    /// sample is simply one more pre-attach record. Node-identity resolution is
    /// unaffected: the manifest is written once at create and read at open, so a
    /// late attacher resolves indices exactly as an at-zero one does.
    pub fn open_at_live(shm_name: &str) -> ShmRingResult<Self> {
        let mut consumer = Self::open(shm_name)?;
        consumer.read_cursor = consumer.write_load();
        // The cursor JUMPED, so republish it — a backpressure producer
        // blocked because nobody had read anything is released by exactly this.
        consumer.publish_read_cursor();
        tracing::debug!(
            name = %shm_name,
            live_cursor = consumer.read_cursor,
            "shm_ring opened AT LIVE — records before this cursor are not in this consumer's window"
        );
        Ok(consumer)
    }

    /// `Release`-store this consumer's read cursor into the header — but ONLY on a
    /// [`OverrunPolicy::Backpressure`] ring.
    ///
    /// The policy gate is what keeps a `FailLoud` ring — every trace ring — byte-
    /// for-byte what it was before backpressure existed: the word stays at its create-time
    /// zero, nothing stores to it, and the producer never loads it. An UNKNOWN
    /// policy discriminant is treated as not-backpressure, which is the safe arm:
    /// publishing into a word a peer of another version may be using for something
    /// else is worse than a producer that waits and then laps loudly.
    ///
    /// `Release` pairs with the producer's `Acquire` load, so a producer that sees
    /// the advanced cursor also sees that this consumer finished reading the slots
    /// it is about to reuse.
    fn publish_read_cursor(&self) {
        if !matches!(
            OverrunPolicy::from_wire(self.overrun_policy),
            Some(OverrunPolicy::Backpressure)
        ) {
            return;
        }
        // SAFETY: `header` points at the mapped header for this consumer's lifetime.
        unsafe {
            (*self.header)
                .read_cursor
                .store(self.read_cursor, Ordering::Release)
        };
    }

    /// `Acquire`-load the producer's write cursor.
    fn write_load(&self) -> u64 {
        // SAFETY: `header` points at the mapped header for this consumer's lifetime.
        unsafe { (*self.header).write_cursor.load(Ordering::Acquire) }
    }

    /// Number of unread, unlapped records: `write_cursor - read_cursor`. A value
    /// greater than `capacity` means the consumer has been lapped (an overrun that
    /// [`drain_slices`](Self::drain_slices) reports).
    pub fn available(&self) -> u64 {
        self.write_load() - self.read_cursor
    }

    /// The consumer's local read cursor.
    pub fn read_cursor(&self) -> u64 {
        self.read_cursor
    }

    /// The producer's current write cursor (`Acquire`).
    pub fn write_cursor(&self) -> u64 {
        self.write_load()
    }

    /// Per-record size in bytes (from the header).
    pub fn record_size(&self) -> u32 {
        self.record_size as u32
    }

    /// Ring capacity in records.
    pub fn capacity(&self) -> u64 {
        self.capacity
    }

    /// Create-generation from the header (restart detection).
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Producer rank from the header.
    pub fn rank(&self) -> u32 {
        self.rank
    }

    /// The RAW overrun-policy discriminant from the header. Decode with
    /// [`OverrunPolicy::from_wire`], which returns `None` for a value this build
    /// does not know — raw rather than pre-decoded so a reader can tell "policy 0"
    /// from "a policy I cannot honour".
    pub fn overrun_policy(&self) -> u32 {
        self.overrun_policy
    }

    /// The consumer's read cursor as PUBLISHED in the header — what a backpressure
    /// producer actually waits on, as opposed to this consumer's local
    /// [`read_cursor`](Self::read_cursor).
    ///
    /// The two agree on a backpressure ring and diverge on a `FailLoud` one, where
    /// the word is never written: that divergence is the observable (Principle #3)
    /// behind "the trace ring publishes nothing".
    pub fn published_read_cursor(&self) -> u64 {
        // SAFETY: `header` points at the mapped header for this consumer's lifetime.
        unsafe { (*self.header).read_cursor.load(Ordering::Acquire) }
    }

    /// The raw manifest bytes read at open (the trace layer decodes them).
    pub fn manifest(&self) -> &[u8] {
        &self.manifest
    }

    /// Which manifest sections the writer's degrade ladder
    /// dropped, as a bit set (`0` = nothing dropped, which is also what every
    /// older ring reads).
    ///
    /// Read from the header rather than inferred from the manifest, because an
    /// absent section cannot say WHY it is absent: a rank with no wired inputs and
    /// a rank whose input section was dropped at the bottom rung decode
    /// identically, and they call for opposite answers from a replay.
    pub fn degraded_sections(&self) -> u32 {
        // SAFETY: `header` points at the mapped header (kept alive by `_mapping`).
        unsafe { (*self.header).degraded_sections.load(Ordering::Acquire) }
    }

    /// The object name this consumer opened.
    pub fn name(&self) -> &str {
        &self.name_str
    }

    /// Compute the unread region as byte offsets, detecting an up-front lap.
    /// Returns `(write_cursor, first_byte_offset, first_byte_len, second_byte_len)`
    /// — all `Copy`, so callers incur no borrow of the mapping.
    fn unread_regions(&self) -> ShmRingResult<(u64, usize, usize, usize)> {
        let write = self.write_load();
        let read = self.read_cursor;
        let unread = write - read;
        if unread > self.capacity {
            return Err(self.overrun(read, write));
        }
        let start_idx = (read % self.capacity) as usize;
        let count = unread as usize;
        let cap = self.capacity as usize;
        let first = count.min(cap - start_idx);
        let second = count - first;
        Ok((
            write,
            start_idx * self.record_size,
            first * self.record_size,
            second * self.record_size,
        ))
    }

    /// Build an [`ShmRingError::Overrun`] for the given cursors.
    fn overrun(&self, read: u64, write: u64) -> ShmRingError {
        let records_lost = (write - read).saturating_sub(self.capacity);
        tracing::error!(
            name = %self.name_str,
            read_cursor = read,
            write_cursor = write,
            capacity = self.capacity,
            records_lost,
            "shm_ring overrun: consumer lapped by producer"
        );
        ShmRingError::Overrun {
            records_lost,
            read_cursor: read,
            write_cursor: write,
            capacity: self.capacity,
        }
    }

    /// The unread region as up-to-2 raw byte slices pointing INTO the mapped ring
    /// (2 slices when the region wraps the ring end). Zero-copy: `bagd` can `writev`
    /// straight from these. Errors [`ShmRingError::Overrun`] if the consumer has
    /// ALREADY been lapped.
    ///
    /// The returned slices are valid only until the next [`commit`](Self::commit):
    /// the caller must consume/write them, then `commit(n_records)`, which
    /// re-validates that the producer did not lap into the region during the read.
    pub fn drain_slices(&mut self) -> ShmRingResult<(&[u8], &[u8])> {
        let (_write, off1, len1, len2) = self.unread_regions()?;
        // SAFETY: the offsets/lengths are within the data region; the returned
        // slices borrow `&mut self`, so the mapping outlives them. The second
        // slice always begins at the data region start (the wrap point).
        let s1 = unsafe { std::slice::from_raw_parts(self.data.add(off1), len1) };
        let s2 = unsafe { std::slice::from_raw_parts(self.data, len2) };
        Ok((s1, s2))
    }

    /// Advance the read cursor by `n_records` after a [`drain_slices`](Self::drain_slices),
    /// re-validating that the producer did not lap into the just-drained region
    /// while it was being read. Errors WITHOUT advancing the cursor:
    /// [`ShmRingError::Overrun`] on a torn drain (the recording fails loudly), or
    /// [`ShmRingError::CommitBeyondAvailable`] if `n_records` exceeds the available
    /// record count (a consumer bug — the torn-drain lap check runs first).
    pub fn commit(&mut self, n_records: u64) -> ShmRingResult<()> {
        let write = self.write_load();
        let read = self.read_cursor;
        // Torn-drain detection: if the producer has now lapped the region the
        // consumer just read from (`write - read > capacity`), the drained bytes
        // may have been overwritten mid-read — fail loudly, do NOT advance.
        if write - read > self.capacity {
            return Err(self.overrun(read, write));
        }
        // Over-commit is a hard error in EVERY build mode (this is a pub API):
        // advancing read_cursor past write_cursor would make every later
        // `write - read` underflow into a spurious huge Overrun.
        let available = write - read;
        if n_records > available {
            tracing::error!(
                name = %self.name_str,
                requested = n_records,
                available,
                "shm_ring commit beyond available — consumer bug; cursor not advanced"
            );
            return Err(ShmRingError::CommitBeyondAvailable {
                requested: n_records,
                available,
            });
        }
        self.read_cursor = read + n_records;
        // Publish AFTER advancing, and only on a successful commit — the
        // two early-return arms above leave the cursor where it was, so they must
        // leave the published word there too or a producer would reuse slots this
        // consumer has not committed.
        self.publish_read_cursor();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_shm_name_is_deterministic_and_prefixed() {
        let a = ring_shm_name("trace_rank0");
        let b = ring_shm_name("trace_rank0");
        assert_eq!(a, b, "same tag → identical name");
        assert!(
            a.starts_with("/cer_rg_"),
            "name has the /cer_rg_ prefix: {a}"
        );
    }

    #[test]
    fn ring_shm_name_fits_macos_31_char_limit() {
        // macOS PSHMNAMLEN = 31 including the leading slash. `/cer_rg_` + 16 hex = 24.
        for tag in ["", "x", "a-very-long-tag-that-would-blow-a-naive-scheme"] {
            let n = ring_shm_name(tag);
            assert!(n.len() <= 31, "name {n:?} ({} chars) must be ≤ 31", n.len());
            assert_eq!(n.len(), 24, "fixed-length hex name is always 24 chars: {n}");
        }
    }

    #[test]
    fn ring_shm_name_fnv_oracle() {
        // FNV-1a-64 hand oracle (same algorithm as barrier/doorbell name helpers):
        //   fnv1a64("a")     = 0xaf63dc4c8601ec8c
        //   fnv1a64("topic") = 0x520c8b7d6934ac64
        assert_eq!(ring_shm_name("a"), "/cer_rg_af63dc4c8601ec8c");
        assert_eq!(ring_shm_name("topic"), "/cer_rg_520c8b7d6934ac64");
    }

    #[test]
    fn ring_shm_name_differs_by_tag() {
        assert_ne!(ring_shm_name("a"), ring_shm_name("b"));
    }

    #[test]
    fn layout_constants_are_pinned() {
        // Runtime restatement of the compile-time asserts.
        assert_eq!(std::mem::size_of::<ShmRingHeader>(), 65600);
        assert_eq!(HEADER_SIZE, 65600);
        assert_eq!(MANIFEST_OFFSET, 64);
        assert_eq!(MANIFEST_CAPACITY, 65536);
        assert_eq!(HEADER_SIZE % 64, 0);
        // The read cursor took over the tail of the old
        // `_pad_manifest`. Its arrival must move NOTHING — same header size, same
        // manifest offset, same pinned field offsets — or every trace ring written
        // by another build stops mapping.
        assert_eq!(HEADER_OFF_READ_CURSOR, 56);
        assert_eq!(HEADER_OFF_OVERRUN_POLICY, 24);
    }

    /// The wire discriminants are a CROSS-PROCESS format, so they are pinned by
    /// value, not by declaration order: a reordered enum that renumbered them would
    /// make a running consumer read the wrong contract off a live ring.
    #[test]
    fn overrun_policy_wire_values_are_frozen_and_round_trip() {
        assert_eq!(OverrunPolicy::FailLoud.as_wire(), 0);
        assert_eq!(OverrunPolicy::Backpressure.as_wire(), 1);
        for p in [OverrunPolicy::FailLoud, OverrunPolicy::Backpressure] {
            assert_eq!(
                OverrunPolicy::from_wire(p.as_wire()),
                Some(p),
                "{p:?} round-trips through the header word"
            );
        }
        assert!(!OverrunPolicy::FailLoud.is_backpressure());
        assert!(OverrunPolicy::Backpressure.is_backpressure());
        // An unknown discriminant decodes to None rather than silently to the
        // wait-free arm — a reader must be able to say "a policy I cannot honour".
        for raw in [2u32, 3, u32::MAX] {
            assert_eq!(OverrunPolicy::from_wire(raw), None, "raw {raw} is unknown");
        }
    }

    /// The wait predicate, pinned on BOTH sides of its boundary against a hand
    /// oracle. The boundary is the whole rule: the consumer calls
    /// `unread > capacity` a lap, so a write is safe exactly while
    /// `local_write - read < capacity` and must wait AT equality. A predicate
    /// widened to `>` laps by one record; one narrowed to `> capacity - 1` (i.e.
    /// `>= capacity - 1`) stalls a ring that had room.
    #[test]
    fn backpressure_wait_predicate_is_a_threshold_pinned_on_both_sides() {
        const CAP: u64 = 8;
        // Empty ring, and every fill short of full: never wait.
        for unread in 0..CAP {
            assert!(
                !backpressure_must_wait(100 + unread, 100, CAP),
                "unread={unread} of capacity {CAP} still has room"
            );
        }
        // Exactly full: the next slot is the oldest uncommitted record's.
        assert!(
            backpressure_must_wait(100 + CAP, 100, CAP),
            "at capacity the producer MUST wait"
        );
        // Already lapped (only reachable after a timed-out wait): still waits.
        assert!(backpressure_must_wait(100 + CAP + 1, 100, CAP));
        // A published cursor AHEAD of the local write (a corrupt header, or an
        // un-resynced fork copy) saturates to "no wait" rather than underflowing
        // into a permanent stall.
        assert!(!backpressure_must_wait(5, 9, CAP));
        // Capacity 1: room only when the single slot is committed.
        assert!(!backpressure_must_wait(0, 0, 1));
        assert!(backpressure_must_wait(1, 0, 1));
        assert!(!backpressure_must_wait(1, 1, 1));
    }

    /// The free-record COUNT, against a hand oracle at
    /// every fill of a capacity-8 ring plus the two saturating edges.
    ///
    /// Written as absolute expectations rather than as a formula, because a
    /// formula here would be the implementation restated — and the whole value of
    /// the count is that a caller can act on the NUMBER, not just on "full or
    /// not".
    #[test]
    fn backpressure_free_records_counts_the_room_a_producer_actually_has() {
        const CAP: u64 = 8;
        // Empty ring: the whole capacity is free.
        assert_eq!(backpressure_free_records(100, 100, CAP), 8);
        // Each committed-but-unread record costs exactly one slot.
        assert_eq!(backpressure_free_records(101, 100, CAP), 7);
        assert_eq!(backpressure_free_records(104, 100, CAP), 4);
        assert_eq!(backpressure_free_records(107, 100, CAP), 1);
        // Exactly full: no room, which must be ZERO and not one.
        assert_eq!(backpressure_free_records(108, 100, CAP), 0);
        // Already lapped: still zero, never a wrapped-around huge number.
        assert_eq!(backpressure_free_records(109, 100, CAP), 0);
        // A published cursor AHEAD of the local write saturates to "plenty of
        // room" — the same arm the wait predicate takes for the same input, and
        // the one that cannot deadlock.
        assert_eq!(backpressure_free_records(5, 9, CAP), CAP);
        // Capacity 1, both states.
        assert_eq!(backpressure_free_records(0, 0, 1), 1);
        assert_eq!(backpressure_free_records(1, 0, 1), 0);
    }

    /// The count and the wait predicate are the SAME rule, so they must agree at
    /// every point — `must_wait` iff the count is zero.
    ///
    /// Pinned as a relation rather than trusted, because they are two functions
    /// over the same three numbers and a later edit to either boundary would
    /// otherwise let the precheck admit a push the wait path then blocks on:
    /// exactly the stall the precheck exists to prevent, reintroduced silently.
    #[test]
    fn the_free_count_is_zero_exactly_when_the_producer_must_wait() {
        const CAP: u64 = 8;
        for read in [0u64, 1, 100] {
            for offset in 0..=(CAP + 2) {
                let write = read + offset;
                assert_eq!(
                    backpressure_free_records(write, read, CAP) == 0,
                    backpressure_must_wait(write, read, CAP),
                    "disagreement at write={write} read={read} cap={CAP}"
                );
            }
        }
        // And on the saturating edge, where both must answer "there is room".
        assert!(!backpressure_must_wait(5, 9, CAP));
        assert!(backpressure_free_records(5, 9, CAP) > 0);
    }

    #[test]
    fn segment_size_is_header_plus_records() {
        assert_eq!(segment_size(40, 8), HEADER_SIZE + 320);
        assert_eq!(segment_size(1, 1), HEADER_SIZE + 1);
    }

    #[test]
    fn new_generation_is_strictly_increasing_within_process() {
        let g1 = new_generation();
        let g2 = new_generation();
        let g3 = new_generation();
        assert!(
            g1 < g2 && g2 < g3,
            "generations strictly increase: {g1} {g2} {g3}"
        );
    }

    /// The `create_reason` renderer must keep the EXACT `reason` text this
    /// module produced before the `shm_map` extraction — the strings are what an
    /// operator reads out of a failed recording, and the step→text mapping is
    /// the one thing the shared substrate deliberately does NOT own.
    ///
    /// (The mechanic those steps name — a failed `mmap` reporting its OWN errno
    /// rather than a close-clobbered "Success" — moved with the code it pins, to
    /// `shm_map`'s `map_shared_reports_the_mmap_errno_not_the_close_result`.)
    #[test]
    fn create_reason_renders_each_step_the_way_it_always_did() {
        let mk = |step| MapError {
            step,
            err: std::io::Error::from_raw_os_error(libc::EBADF),
        };
        let bad = std::io::Error::from_raw_os_error(libc::EBADF).to_string();
        assert_eq!(
            create_reason(mk(MapStep::ShmOpen), 4096),
            format!("shm_open(O_CREAT|O_EXCL): {bad}")
        );
        assert_eq!(
            create_reason(mk(MapStep::Ftruncate), 4096),
            format!("ftruncate(4096): {bad}")
        );
        assert_eq!(
            create_reason(mk(MapStep::Mmap), 4096),
            format!("mmap(4096): {bad}")
        );
    }
}

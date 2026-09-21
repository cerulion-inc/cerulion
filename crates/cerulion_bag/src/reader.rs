// SPDX-License-Identifier: AGPL-3.0-only
//! [`BagReader`] — reads a Cerulion bag back via the `mcap` crate.
//!
//! The writer is hand-rolled (zero-copy `writev`), but the reader delegates to
//! the `mcap` crate: bags on disk ARE standard MCAP, so the crate's
//! `MessageStream` / `Summary` do the parsing, chunk decompression (a no-op for
//! our always-uncompressed chunks), and chunk-CRC validation.
//!
//! # Crash recovery (verdict: NATIVE)
//!
//! The `mcap` crate reads a truncated (torn-tail / no-summary) file up to the
//! last complete record: [`recover_messages`](BagReader::recover_messages)
//! streams with `Options::IgnoreEndMagic` and yields every complete message.
//! No linear-scan fallback is needed.
//!
//! How the bag ENDED is reported as a [`BagCompleteness`], because the stream
//! alone cannot distinguish two very different endings: a torn tail errors
//! terminally, but a crash landing exactly on a record boundary ends the
//! stream cleanly — and since the writer flushes at chunk granularity, a
//! recorder crash lands there OFTEN. Finalization is therefore detected
//! independently by fingerprinting the epilogue (the Footer record frame +
//! both magics).

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Mutex;

use cerulion_core::trace_ring::{TraceRingRecord, TRACE_RECORD_SIZE};
use indexmap::IndexMap;

use crate::error::{BagError, BagResult};
use crate::schema::{SchemaDescriptor, RESERVED_PREFIX, SCHEDULER_TRACE_TOPIC, SCHEMA_ENCODING};

/// The byte range of one recorded frame WITHIN a bag's mapped bytes (an offset +
/// length into [`BagReader::bytes`]), resolved on demand by [`BagReader::frame`].
///
/// Memory bound: replay serves recorded payloads as borrowed `&[u8]` slices
/// into the memory-mapped bag instead of owning a `Vec<u8>` per frame. A span is
/// 16 bytes regardless of the frame's payload size, so the per-topic span index
/// scales with FRAME COUNT, never payload bytes — replay peak RSS stays
/// independent of bag size (the map itself is page-cache-backed, kernel-evictable
/// under pressure; only the working set is resident).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameSpan {
    /// Byte offset of the frame's first byte within [`BagReader::bytes`].
    pub offset: usize,
    /// Frame length in bytes (the full recorded wire frame: 32-byte header + payload).
    pub len: usize,
}

// ===========================================================================
// Advise-behind — bag-map memory POLICY (zero effect on replayed bytes)
// ===========================================================================
//
// A finalized bag is IMMUTABLE and consumed once, monotonically per stream. Its
// true working set during replay is O(read-ahead window), yet a naive mmap read
// leaves every touched page resident — a 200 GB bag peaks RSS at the machine's RAM
// ceiling. This module lets replay hand the kernel a MONOTONIC min-cursor
// watermark; pages strictly BEHIND it (already consumed by every live stream)
// are `madvise`d away in coarse batches.
//
// PRINCIPLE #7: the mapping is a read-only view of an immutable file, so
// `MADV_DONTNEED`/`MADV_FREE` cannot change one replayed byte — a re-touched
// page simply re-faults the SAME bytes from disk. Eviction is pure memory
// policy; the replay verdict is byte-identical with it active, disabled, or
// compiled out (proven by the Principle-#7 pin in the engine test).

/// Default eviction batch: only `madvise` once the min cursor has advanced this
/// far past the last-evicted offset, so the syscall fires ~once per this many
/// bytes streamed — never on the per-record path. Coarse on purpose (the
/// per-record cost stays a single atomic load + a branch).
pub const ADVISE_BEHIND_BATCH_BYTES: usize = 256 * 1024 * 1024;

/// The de-pin window: the MAX bytes the replay loop's DATA cursor
/// (`FrameFeed::oldest_needed_offset`) is allowed to lag behind the user-frame
/// walk HEAD before eviction stops waiting for it.
///
/// The replay-loop watermark is the min over three cursors — the user-frame
/// feed's oldest still-buffered offset plus every rank's fire/boundary trace
/// frontier. The trace cursors self-heal (an exhausted walk reports the
/// data-section end), but the DATA cursor can FREEZE at a low offset for the
/// whole remainder of a replay: an under-consumed produced topic (a diverging
/// candidate) or a large inter-arrival gap leaves that topic's queue front
/// pinned while the walk races ahead, so the k-way min never advances and the
/// mapped resident set climbs ~monotonically (the 200 GB-class RSS blowup on a
/// lossy bag). Because the feed retains only `FrameSpan` OFFSETS (never a live
/// map borrow) and every borrowed frame slice is dropped before the advise
/// call, a page behind a buffered-but-unconsumed span is safe to evict — it
/// re-faults byte-identical if the lagging consumer ever reaches it (Principle
/// #7, exactly the guarantee eviction already rests on). So the loop evicts
/// behind `max(oldest_needed, walk_head − ADVISE_MAX_LAG_BYTES)`, capping the
/// lagging-cursor contribution at this window and accepting a BOUNDED re-fault.
///
/// Chosen ≥ the batch so a healthy walk-ahead skew (`O(topics × burst)`, KB–MB)
/// never triggers a re-fault. The de-pin bites only once a topic's oldest
/// unconsumed frame lags the walk head by more than this window: a frozen
/// consumer (a diverging / under-producing candidate) always, and — on a bag
/// LARGER than this window — a legitimately low-rate or latched consumed topic
/// (e.g. a once-published `/tf_static` racing high-rate topics) too. The latter
/// is NOT pathological: it pays only a BOUNDED re-fault of its lagging pages
/// once it finally consumes them, and the verdict is unaffected (Principle #7).
/// No BALANCED bag (every topic consumed near its production rate) reaches it. A
/// test seam ([`BagReader::set_advise_behind_lag_for_test`]) lowers the window so
/// a small bag's replay exercises the de-pin.
pub const ADVISE_MAX_LAG_BYTES: usize = 256 * 1024 * 1024;

/// One recorded advise-behind DECISION — the test probe surface (see
/// [`BagReader::enable_advise_behind_probe`]). Records the page-aligned region
/// handed to `madvise` plus the min-cursor watermark that authorized it, so a
/// test can assert `end <= watermark` (never evict past the slowest cursor).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdviseCall {
    /// Page-aligned start offset (exclusive of nothing — the region is `[start, end)`).
    pub start: usize,
    /// Page-aligned end (exclusive). Invariant: `end <= watermark`.
    pub end: usize,
    /// The min-cursor watermark at call time.
    pub watermark: usize,
}

/// Process-cached page size (`sysconf(_SC_PAGESIZE)`) — 16 KiB on aarch64
/// macOS, 4 KiB on x86-64; `madvise` requires a page-aligned start, so a fixed
/// 4096 would be WRONG on 16 KiB-page hosts.
fn page_size() -> usize {
    static PAGE: AtomicUsize = AtomicUsize::new(0);
    let cached = PAGE.load(Ordering::Relaxed);
    if cached != 0 {
        return cached;
    }
    // SAFETY: `sysconf` is a pure query with no pointer args.
    let ps = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    let ps = if ps > 0 { ps as usize } else { 4096 };
    PAGE.store(ps, Ordering::Relaxed);
    ps
}

/// PURE watermark math (no syscall, no state): given the last page-aligned end
/// already evicted, the current min-cursor `watermark`, the map length, the
/// batch threshold and the page size, return the `[start, end)` region to evict
/// NOW, or `None` if the min cursor has not advanced a full batch past
/// `last_end`.
///
/// `end` = `min(watermark, map_len)` rounded DOWN to a page — so eviction stays
/// at or behind the `watermark` the caller hands in, never touching that page
/// nor anything ahead of it. (The caller's `watermark` is the k-way min of the
/// live cursors, but the DATA cursor is DE-PINNED per `ADVISE_MAX_LAG_BYTES`, so
/// the watermark may legitimately sit ABOVE a lagging data stream's still-
/// buffered front; those below-watermark pages re-fault byte-identical if that
/// stream reaches them — Principle #7.)
/// `start` = `last_end` (kept page-aligned by construction). `Some` only when
/// `end - last_end >= batch` (coarse batching) — the never-advise-past-min
/// invariant is `end <= watermark`, which holds because align-down never raises
/// a value.
fn advise_behind_region(
    last_end: usize,
    watermark: usize,
    map_len: usize,
    batch: usize,
    page: usize,
) -> Option<(usize, usize)> {
    debug_assert!(page.is_power_of_two());
    let capped = watermark.min(map_len);
    let end = capped & !(page - 1); // align DOWN (page is a power of two).
    if end <= last_end || end - last_end < batch {
        return None;
    }
    Some((last_end, end))
}

/// The eviction advice for touched-then-done pages: `MADV_DONTNEED` frees them
/// immediately on Linux; macOS has no equivalent immediate-drop for a shared
/// file map (its `MADV_DONTNEED` is a soft hint), so `MADV_FREE` — which lets
/// the kernel reclaim the pages under pressure — is the closest macOS analogue.
#[cfg(target_os = "linux")]
const EVICT_ADVICE: libc::c_int = libc::MADV_DONTNEED;
#[cfg(all(unix, not(target_os = "linux")))]
const EVICT_ADVICE: libc::c_int = libc::MADV_FREE;

/// Per-reader advise-behind state (interior-mutable — [`BagReader`] is shared
/// `&self` behind an `Arc` during replay, and the driver is called once per
/// step from the single replay thread).
struct AdviseState {
    /// Page-aligned end offset already handed to `madvise` — the low edge of
    /// the still-resident window. Monotonic within one streaming pass.
    last_end: AtomicUsize,
    /// Advice is advisory: on the FIRST `madvise` error we warn once, then stay
    /// quiet (never fail the replay, but never silently ignore forever either).
    warned: AtomicBool,
    /// `true` in production. A test seam ([`BagReader::set_advise_behind_enabled`])
    /// disables eviction to prove the replay verdict is identical with the
    /// policy off (Principle #7).
    enabled: AtomicBool,
    /// Test probe for the REPLAY-LOOP driver ([`BagReader::advise_evict_behind`],
    /// the k-way min-cursor watermark): when `Some`, every eviction DECISION is
    /// recorded. Always `None` in production (a single `Mutex` load per batch,
    /// never per record).
    probe: Mutex<Option<Vec<AdviseCall>>>,
    /// Test probe for the PER-PASS driver ([`BagReader::advise_evict_behind_scoped`]
    /// — the pre/post-loop full-file passes: completeness scan, aggregate fold,
    /// trace/consistency validate walks). Kept SEPARATE from `probe` so a test
    /// observing the replay-loop sweep never sees pre-pass evictions folded in
    /// (and vice versa) — the two drivers advance independent cursors, so their
    /// decision streams must not be conflated. Always `None` in production.
    scoped_probe: Mutex<Option<Vec<AdviseCall>>>,
    /// Eviction batch threshold — [`ADVISE_BEHIND_BATCH_BYTES`] in production; a
    /// test seam lowers it so a SMALL bag's replay exercises real eviction.
    /// Shared by both drivers (production coarseness is identical either way).
    batch: AtomicUsize,
    /// The de-pin window — [`ADVISE_MAX_LAG_BYTES`] in production; the max
    /// bytes the replay loop's DATA cursor may lag the user-frame walk head
    /// before eviction stops waiting for it (see [`ADVISE_MAX_LAG_BYTES`]). A
    /// test seam lowers it so a SMALL bag's replay exercises the de-pin. Read by
    /// the replay-loop driver only (the per-pass scoped driver walks one stream
    /// front-to-back and never lags).
    max_lag: AtomicUsize,
}

impl Default for AdviseState {
    fn default() -> Self {
        Self {
            last_end: AtomicUsize::new(0),
            warned: AtomicBool::new(false),
            enabled: AtomicBool::new(true),
            probe: Mutex::new(None),
            scoped_probe: Mutex::new(None),
            batch: AtomicUsize::new(ADVISE_BEHIND_BATCH_BYTES),
            max_lag: AtomicUsize::new(ADVISE_MAX_LAG_BYTES),
        }
    }
}

/// A PASS-SCOPED advise-behind cursor: an independent monotonic `last_end` for
/// ONE sequential full-file walk, sharing the reader's batch / enabled /
/// warned config. Each standalone pre-loop (and rare post-loop) pass — the
/// completeness scan, the `RecordedMessages`-style aggregate fold, the
/// trace/consistency validate walks — creates its OWN cursor and advises behind
/// ITS frontier via [`BagReader::advise_evict_behind_scoped`].
///
/// A hazard solved by per-walk state (NOT a shared mutable
/// `last_end`): the replay LOOP's `AdviseState` `last_end` is monotonic
/// within ONE pass. A second sequential pass restarting from offset 0 would be
/// STARVED by a high shared `last_end` (its watermark never advances a batch
/// PAST it, so it never evicts), and resetting the shared cursor to 0 for the
/// pre-pass would then REWIND the loop's own cursor. A separate per-pass cursor
/// removes the coupling entirely — the loop's `last_end` is never touched by
/// any pass, and each pass advises behind its own position.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct AdviseCursor {
    /// Page-aligned end offset this pass has already handed to `madvise`.
    last_end: usize,
}

impl AdviseCursor {
    /// A fresh cursor at offset 0 — one per sequential full-file pass.
    pub fn new() -> Self {
        Self { last_end: 0 }
    }
}

/// The bytes of an open bag: either the memory map ([`BagReader::open`], the
/// normal path — the file is mapped read-only, no heap copy) or an owned buffer
/// ([`BagReader::from_bytes`], the in-memory test path). Both `Deref` to `&[u8]`
/// so every reader method is source-agnostic.
enum BagBytes {
    /// The read-only memory map of the bag file (the whole file, demand-paged).
    Mapped(memmap2::Mmap),
    /// An owned byte buffer (tests / already-in-memory bags).
    Owned(Vec<u8>),
}

impl std::ops::Deref for BagBytes {
    type Target = [u8];
    fn deref(&self) -> &[u8] {
        match self {
            BagBytes::Mapped(m) => m,
            BagBytes::Owned(v) => v,
        }
    }
}

/// A channel as read back from a bag, with its decoded `cerulion` descriptor.
#[derive(Debug, Clone)]
pub struct BagChannel {
    /// The MCAP channel id.
    pub id: u16,
    /// The topic name.
    pub topic: String,
    /// The qualified schema name.
    pub schema_name: String,
    /// The Schema record `encoding` (`cerulion` for our channels).
    pub schema_encoding: String,
    /// The channel `message_encoding`.
    pub message_encoding: String,
    /// The decoded descriptor, if the schema encoding is `cerulion`.
    pub descriptor: Option<SchemaDescriptor>,
    /// How the topic's transport was PROVISIONED, read from the
    /// Channel record's `metadata` map.
    ///
    /// Every field is `None` on a pre-provisioning bag (the map was empty) and on any
    /// channel whose recorder declared nothing — "not recorded", never a
    /// fabricated default. See [`crate::ChannelProvisioning`].
    pub provisioning: crate::ChannelProvisioning,
}

/// A message read back from a bag.
#[derive(Debug, Clone)]
pub struct BagMessage {
    /// The channel id it was published on.
    pub channel_id: u16,
    /// The topic name.
    pub topic: String,
    /// The per-channel sequence number.
    pub sequence: u32,
    /// The record log time (ns).
    pub log_time: u64,
    /// The record publish time (ns).
    pub publish_time: u64,
    /// The (owned) payload bytes.
    pub data: Vec<u8>,
}

impl<'a> From<mcap::Message<'a>> for BagMessage {
    fn from(m: mcap::Message<'a>) -> Self {
        Self {
            channel_id: m.channel.id,
            topic: m.channel.topic.clone(),
            sequence: m.sequence,
            log_time: m.log_time,
            publish_time: m.publish_time,
            data: m.data.into_owned(),
        }
    }
}

/// An attachment read back from a bag.
#[derive(Debug, Clone)]
pub struct BagAttachment {
    /// The attachment name.
    pub name: String,
    /// The attachment media type.
    pub media_type: String,
    /// The attachment log time (ns).
    pub log_time: u64,
    /// The attachment create time (ns).
    pub create_time: u64,
    /// The (owned) attachment bytes.
    pub data: Vec<u8>,
}

/// How a bag's byte stream ended — the crash-recovery completeness signal.
///
/// Replay tooling branches on this: anything other than
/// [`Finalized`](Self::Finalized) means the recording did not close cleanly and
/// strict replay should be refused unless explicitly allowed.
#[derive(Debug)]
#[non_exhaustive]
pub enum BagCompleteness {
    /// The bag was [`finalize`](crate::BagWriter::finalize)d: the epilogue
    /// (Footer record + closing magic) is present. The summary/Statistics are
    /// trustworthy.
    Finalized,
    /// The stream ended CLEANLY at a RECORD boundary but the finalization
    /// epilogue is missing — the recorder died after a chunk flush (the common
    /// crash point, since the writer flushes at chunk granularity) or after any
    /// other top-level record, such as a mid-file channel registration
    /// or an attachment. All recovered messages are complete; there is no
    /// summary.
    ///
    /// The NAME is kept deliberately: it is public API, a chunk boundary remains
    /// by far the likeliest place to land, and the variant's meaning — "clean
    /// ending, no epilogue" — is unchanged by widening which records can precede
    /// it.
    TruncatedAtChunkBoundary,
    /// The stream ended INSIDE a record or on a failed chunk CRC; carries the
    /// error that stopped the scan. Messages recovered before the tear are
    /// complete; everything after is lost.
    TornTail(BagError),
}

impl BagCompleteness {
    /// `true` only for [`Finalized`](Self::Finalized).
    pub fn is_finalized(&self) -> bool {
        matches!(self, BagCompleteness::Finalized)
    }
}

/// Reads a Cerulion bag. Backs onto a read-only memory map of the file
/// ([`open`](Self::open)) so recorded payloads are served as borrowed page-cache
/// slices, or an owned buffer ([`from_bytes`](Self::from_bytes)) for tests.
///
/// # Torn-file hazard (mmap contract)
///
/// The mapped file is treated STRICTLY read-only. If another process truncates
/// or rewrites the bag while a [`BagReader`] holds it open, touching an evicted
/// page of the shrunken/rewritten region is undefined behavior (a `SIGBUS` on
/// most platforms) — the same hazard any `mmap` reader carries. Replay tooling
/// opens finalized bags that are not being written; concurrent modification is
/// out of contract.
pub struct BagReader {
    data: BagBytes,
    /// The handle the mapping was made from, held for the reader's lifetime.
    ///
    /// `None` for [`BagReader::from_bytes`], which has no file.
    ///
    /// Retained so a caller that needs a FACT ABOUT THE FILE — its mode, its
    /// size, its identity — can ask THIS inode rather than re-resolving the
    /// path. `cerulion bag migrate` stamps the source bag's size and SHA-256
    /// into its provenance record and copies the recording's permissions onto
    /// the migrated copy; reading any of those back by name lets a concurrent
    /// replacement of the path describe a file the migration never read. Holding
    /// the handle also keeps the mapped inode from being recycled while the map
    /// is live.
    file: Option<std::fs::File>,
    /// Advise-behind state (memory policy only — see
    /// [`advise_evict_behind`](Self::advise_evict_behind)).
    advise: AdviseState,
}

impl BagReader {
    /// Open a bag file from disk, mapping it read-only (no full-file heap copy).
    ///
    /// A zero-length file cannot be mapped (`mmap` rejects a zero length), so it
    /// falls back to an empty owned buffer — the downstream not-an-MCAP-bag gate
    /// then rejects it exactly as a truncated file would.
    ///
    /// The map is `madvise(MADV_SEQUENTIAL)`'d immediately (Unix) — the
    /// whole replay consumes it front-to-back, so this doubles the kernel
    /// read-ahead window. `MADV_SEQUENTIAL`'s drop-behind is only a WEAK
    /// heuristic without memory pressure, though (a 200 GB pre-loop scan still
    /// peaked RSS to the machine's RAM ceiling), so it is NOT the memory bound: EVERY
    /// full-file sequential pass drives EXPLICIT batched advise-behind — the
    /// pre-loop passes (completeness scan, aggregate fold, trace/consistency
    /// validate walks) each via a per-pass [`AdviseCursor`]
    /// ([`advise_evict_behind_scoped`](Self::advise_evict_behind_scoped)), and
    /// the replay loop via the k-way min-cursor
    /// [`advise_evict_behind`](Self::advise_evict_behind).
    pub fn open(path: impl AsRef<Path>) -> BagResult<Self> {
        let file = std::fs::File::open(path)?;
        let len = file.metadata()?.len();
        if len == 0 {
            return Ok(Self {
                data: BagBytes::Owned(Vec::new()),
                advise: AdviseState::default(),
                file: Some(file),
            });
        }
        // SAFETY: the only sound-usage requirement `memmap2` places on a
        // read-only `Mmap` is that the underlying file is not concurrently
        // mutated (the torn-file hazard documented on the type). Replay reads
        // finalized, quiescent bags; concurrent writers are out of contract.
        let mmap = unsafe { memmap2::Mmap::map(&file)? };
        Self::advise_sequential(&mmap);
        Ok(Self {
            data: BagBytes::Mapped(mmap),
            advise: AdviseState::default(),
            file: Some(file),
        })
    }

    /// The handle this reader's bytes were mapped from, if it has one.
    ///
    /// The point is COHERENCE, not access: a fact read through this handle
    /// describes the same inode the bag's bytes came from, which re-opening the
    /// path does not guarantee.
    pub fn file(&self) -> Option<&std::fs::File> {
        self.file.as_ref()
    }

    /// Construct over already-loaded bytes.
    pub fn from_bytes(data: Vec<u8>) -> Self {
        Self {
            data: BagBytes::Owned(data),
            advise: AdviseState::default(),
            file: None,
        }
    }

    /// Hint the whole map for front-to-back streaming (Unix). Advisory — a
    /// failure is debug-logged, never fatal. No-op for a zero-length map or a
    /// non-Unix host.
    #[cfg(unix)]
    fn advise_sequential(mmap: &memmap2::Mmap) {
        let len = mmap.len();
        if len == 0 {
            return;
        }
        // SAFETY: `madvise` reads no memory it is passed; the ptr+len describe
        // this live read-only mapping. `MADV_SEQUENTIAL` cannot mutate a byte.
        let ret = unsafe {
            libc::madvise(
                mmap.as_ptr() as *mut libc::c_void,
                len,
                libc::MADV_SEQUENTIAL,
            )
        };
        if ret != 0 {
            tracing::debug!(
                errno = std::io::Error::last_os_error().raw_os_error(),
                "madvise(MADV_SEQUENTIAL) on bag map failed (advisory — ignored)"
            );
        }
    }
    #[cfg(not(unix))]
    fn advise_sequential(_mmap: &memmap2::Mmap) {}

    /// Advise-behind: given a MONOTONIC `watermark` (the k-way min of
    /// the live replay cursors; note the DATA cursor is DE-PINNED per
    /// [`ADVISE_MAX_LAG_BYTES`], so a lagging data stream MAY still read pages
    /// below the watermark — they re-fault byte-identical, Principle #7), evict
    /// the page-aligned region behind it once the watermark has advanced a full
    /// [`ADVISE_BEHIND_BATCH_BYTES`] batch past the last eviction. Cheap per
    /// call (one atomic load + `advise_behind_region` arithmetic); the
    /// `madvise` syscall fires only per batch.
    ///
    /// Advisory + policy-only: an owned (in-memory test) bag, a disabled seam,
    /// or a non-Unix host all no-op; a `madvise` error warns ONCE then continues.
    /// PRINCIPLE #7: the map is an immutable read-only file view, so an evicted
    /// page re-faults byte-identical — the replay verdict is unaffected.
    pub fn advise_evict_behind(&self, watermark: usize) {
        let last = self.advise.last_end.load(Ordering::Relaxed);
        if let Some(call) = self.evict_region_syscall(last, watermark) {
            // Record the DECISION for the replay-loop test probe (per batch,
            // never per record; `None` in production — one uncontended lock).
            if let Ok(mut g) = self.advise.probe.lock() {
                if let Some(log) = g.as_mut() {
                    log.push(call);
                }
            }
            self.advise.last_end.store(call.end, Ordering::Relaxed);
        }
    }

    /// Per-pass advise-behind: the [`advise_evict_behind`](Self::advise_evict_behind)
    /// twin for a STANDALONE sequential full-file pass (the completeness scan,
    /// the aggregate fold, the trace/consistency validate walks — every full
    /// walk that runs BEFORE, or rarely AFTER, the replay loop). Advises behind
    /// the pass's own monotonic `watermark` using a caller-owned
    /// [`AdviseCursor`], never the replay loop's shared `AdviseState` cursor
    /// (the shared-cursor hazard — see [`AdviseCursor`]). Same primitive, same coarse
    /// batch; the ONLY difference is which cursor state advances.
    ///
    /// Advisory + policy-only + Principle #7, exactly as the loop driver.
    pub fn advise_evict_behind_scoped(&self, cursor: &mut AdviseCursor, watermark: usize) {
        if let Some(call) = self.evict_region_syscall(cursor.last_end, watermark) {
            if let Ok(mut g) = self.advise.scoped_probe.lock() {
                if let Some(log) = g.as_mut() {
                    log.push(call);
                }
            }
            cursor.last_end = call.end;
        }
    }

    /// The shared eviction CORE for both drivers: compute the page-aligned
    /// region behind `last_end` given the min-cursor `watermark`, fire the
    /// `madvise` (Unix), warn ONCE on the first error, and return the
    /// [`AdviseCall`] (so the caller records it to ITS probe + advances ITS
    /// cursor) — or `None` when no full batch has accumulated / there is no map
    /// / eviction is disabled. Defines the region math + syscall + warn-once in
    /// ONE place so the loop and per-pass drivers cannot drift.
    fn evict_region_syscall(&self, last_end: usize, watermark: usize) -> Option<AdviseCall> {
        let BagBytes::Mapped(mmap) = &self.data else {
            return None; // owned buffer — no map to evict.
        };
        if !self.advise.enabled.load(Ordering::Relaxed) {
            return None;
        }
        let (start, end) = advise_behind_region(
            last_end,
            watermark,
            mmap.len(),
            self.advise.batch.load(Ordering::Relaxed),
            page_size(),
        )?;
        #[cfg(unix)]
        {
            // SAFETY: `[start, end)` is page-aligned and strictly within the
            // live mapping (`end <= map_len`); `madvise` reads no memory it is
            // passed and cannot mutate a byte of a read-only map.
            let addr = (mmap.as_ptr() as usize + start) as *mut libc::c_void;
            let ret = unsafe { libc::madvise(addr, end - start, EVICT_ADVICE) };
            if ret != 0 && !self.advise.warned.swap(true, Ordering::Relaxed) {
                tracing::warn!(
                    errno = std::io::Error::last_os_error().raw_os_error(),
                    start,
                    len = end - start,
                    "madvise(evict-behind) on bag map failed (advisory — replay continues \
                     unaffected; this warning fires once)"
                );
            }
        }
        Some(AdviseCall {
            start,
            end,
            watermark,
        })
    }

    /// TEST SEAM: enable/disable the advise-behind eviction (default enabled).
    /// Disabling it (with `MADV_SEQUENTIAL` still set at open) lets a test prove
    /// the replay verdict is byte-identical with the eviction policy off —
    /// Principle #7. Per-reader (no process global), so parallel tests never race.
    pub fn set_advise_behind_enabled(&self, enabled: bool) {
        self.advise.enabled.store(enabled, Ordering::Relaxed);
    }

    /// TEST SEAM: start recording every advise-behind DECISION (see
    /// [`AdviseCall`]). Clears any prior log. Production never calls this, so the
    /// probe stays `None` and costs nothing.
    pub fn enable_advise_behind_probe(&self) {
        if let Ok(mut g) = self.advise.probe.lock() {
            *g = Some(Vec::new());
        }
    }

    /// TEST SEAM: take (and clear) the recorded advise-behind decisions.
    pub fn take_advise_behind_probe(&self) -> Vec<AdviseCall> {
        self.advise
            .probe
            .lock()
            .ok()
            .and_then(|mut g| g.as_mut().map(std::mem::take))
            .unwrap_or_default()
    }

    /// TEST SEAM: start recording every PER-PASS advise-behind DECISION (the
    /// [`advise_evict_behind_scoped`](Self::advise_evict_behind_scoped) driver —
    /// the pre/post-loop full-file passes). Separate from
    /// [`enable_advise_behind_probe`](Self::enable_advise_behind_probe) so a test
    /// can observe the pre-pass sweep in isolation from the replay-loop sweep.
    pub fn enable_scoped_advise_probe(&self) {
        if let Ok(mut g) = self.advise.scoped_probe.lock() {
            *g = Some(Vec::new());
        }
    }

    /// TEST SEAM: take (and clear) the recorded PER-PASS advise-behind decisions.
    pub fn take_scoped_advise_probe(&self) -> Vec<AdviseCall> {
        self.advise
            .scoped_probe
            .lock()
            .ok()
            .and_then(|mut g| g.as_mut().map(std::mem::take))
            .unwrap_or_default()
    }

    /// TEST SEAM: lower the eviction batch threshold so a SMALL bag's replay
    /// exercises real per-step eviction (production uses the coarse
    /// [`ADVISE_BEHIND_BATCH_BYTES`]). The value is aligned-down to a page by
    /// `advise_behind_region` at use, so any positive byte count is valid.
    pub fn set_advise_behind_batch_for_test(&self, batch: usize) {
        self.advise.batch.store(batch, Ordering::Relaxed);
    }

    /// The de-pin window (see [`ADVISE_MAX_LAG_BYTES`]): the max bytes
    /// the replay loop's DATA cursor may lag the user-frame walk head before
    /// eviction evicts behind `walk_head − this` instead of waiting for the
    /// lagging cursor. The replay loop reads it once per step to bound the
    /// still-buffered-but-unconsumed contribution to the k-way min watermark.
    pub fn advise_max_lag_bytes(&self) -> usize {
        self.advise.max_lag.load(Ordering::Relaxed)
    }

    /// TEST SEAM: lower the de-pin window so a SMALL bag's replay
    /// exercises the data-cursor de-pin (production uses the coarse
    /// [`ADVISE_MAX_LAG_BYTES`], which exceeds any test bag, so the de-pin is a
    /// no-op there — `walk_head − window` clamps to 0 and the consume-min wins).
    /// A tiny window makes `walk_head − window` overtake a frozen consume-min so
    /// eviction advances past a lagging topic's pinned front.
    pub fn set_advise_behind_lag_for_test(&self, max_lag: usize) {
        self.advise.max_lag.store(max_lag, Ordering::Relaxed);
    }

    /// TEST SEAM: the data-section end (the footer's `summary_start`) so an
    /// eviction-EFFECTIVENESS pin can bound the final advise watermark against
    /// the region the replay actually sweeps (the summary/index tail is never
    /// re-walked and legitimately stays un-advised). `None` for un-finalized
    /// or footer-less bags.
    pub fn data_section_end_for_test(&self) -> Option<usize> {
        self.finalized_data_end().ok()
    }

    /// The raw file bytes (the memory map, or the owned buffer).
    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    /// Resolve one [`FrameSpan`] (from [`user_message_index`](Self::user_message_index))
    /// to its borrowed frame bytes — a slice into the memory map, no copy. Panics
    /// only if handed a span that does not belong to this reader (its offset/len
    /// exceed the mapped bytes); every span this reader produces is in-bounds by
    /// construction.
    pub fn frame(&self, span: &FrameSpan) -> &[u8] {
        &self.bytes()[span.offset..span.offset + span.len]
    }

    /// The completeness of the bag's byte stream WITHOUT collecting any payloads
    /// — the memory-lean twin of [`recover_messages`](Self::recover_messages) for
    /// the replay gate. Streams every record (each message payload is touched but
    /// never retained), so peak memory is one message, not the whole file. A torn
    /// tail or bad chunk CRC stops the scan and yields
    /// [`BagCompleteness::TornTail`]; a clean end is classified
    /// [`Finalized`](BagCompleteness::Finalized) vs
    /// [`TruncatedAtChunkBoundary`](BagCompleteness::TruncatedAtChunkBoundary) by
    /// the same epilogue fingerprint as `recover_messages`.
    pub fn completeness(&self) -> BagResult<BagCompleteness> {
        let stream = mcap::MessageStream::new_with_options(
            &self.data,
            mcap::read::Options::IgnoreEndMagic.into(),
        )?;
        // This is a FULL front-to-back walk (chunk-CRC + torn-tail
        // scan) touching every page — one of the pre-loop passes that peaked
        // RSS at 200 GB scale under `MADV_SEQUENTIAL` alone. Drive explicit
        // advise-behind with a per-pass cursor.
        //
        // `mcap::MessageStream` copies each payload (`Cow::Owned`) and exposes
        // NO byte cursor, so the watermark is a CONSERVATIVE frontier: the
        // cumulative sum of consumed message-payload lengths. That is a strict
        // LOWER bound on the true read position (it ignores record framing +
        // chunk/channel/schema overhead, all of which sit BEHIND the payloads
        // the scan has already passed), so `[0, sum)` pages are provably
        // already-read — eviction never races the live scan. NOTE: the bound
        // holds because Cerulion chunks are NEVER compressed (writer.rs emits
        // uncompressed chunks only); a compressed reader would decode MORE
        // payload bytes than on-disk bytes and this sum could OVERSHOOT — if
        // compression ever lands, this watermark must switch to an on-disk
        // position source. Monotone by
        // construction; the 256 MiB batch keeps the syscall coarse.
        let mut cursor = AdviseCursor::new();
        let mut consumed = 0usize;
        for item in stream {
            match item {
                Ok(m) => {
                    consumed += m.data.len();
                    self.advise_evict_behind_scoped(&mut cursor, consumed);
                }
                Err(e) => {
                    return Ok(BagCompleteness::TornTail(BagError::from(e)));
                }
            }
        }
        Ok(if self.has_finalization_epilogue() {
            BagCompleteness::Finalized
        } else {
            BagCompleteness::TruncatedAtChunkBoundary
        })
    }

    /// Build a per-topic index of [`FrameSpan`]s over the USER channels (the
    /// reserved `__cerulion/*` channels are excluded — the trace is decoded
    /// separately), in FILE (write) order, WITHOUT copying any payload.
    ///
    /// This is the zero-copy foundation of the replay memory bound: instead
    /// of `collect`ing every frame into an owned `Vec<u8>`, replay keeps only
    /// these 16-byte spans and resolves each frame on demand via [`frame`](Self::frame).
    ///
    /// It hand-walks the record framing (`[opcode u8][len u64 LE][body]`) over the
    /// data section `[MAGIC .. summary_start)` and recurses into each Chunk's
    /// uncompressed body, using [`mcap::parse_record`] to borrow each message's
    /// payload slice directly out of the map (the `mcap` crate's `MessageStream`
    /// deliberately COPIES uncompressed payloads to give `Message` a `'static`
    /// lifetime, so it cannot be used for a zero-copy index). Cerulion bags are
    /// never compressed (the writer's chunk `compression` is always `""`); a
    /// compressed chunk is refused loudly rather than silently mis-indexed.
    ///
    /// Requires a finalized bag (a readable footer with a non-zero
    /// `summary_start`); call the completeness gate first.
    pub fn user_message_index(&self) -> BagResult<IndexMap<String, Vec<FrameSpan>>> {
        let mut walk = self.user_frames()?;
        let mut by_topic: IndexMap<String, Vec<FrameSpan>> = IndexMap::new();
        while let Some((channel_id, span)) = walk.next_user_frame()? {
            let topic = walk.topic(channel_id);
            by_topic.entry(topic.to_string()).or_default().push(span);
        }
        Ok(by_topic)
    }

    /// Stream the USER-channel frames (reserved `__cerulion/*` excluded) as
    /// validated [`FrameSpan`]s in FILE order, WITHOUT building any index — the
    /// memory-scaler frame source. The retained
    /// [`user_message_index`](Self::user_message_index) costs ~16 B/frame, which
    /// a 300 GB-scale bag turns into gigabytes; a streaming consumer holds only
    /// its own bounded look-ahead instead.
    ///
    /// Each call starts a FRESH walk of the data section — sequential re-walks
    /// are the intended pattern. Requires a finalized bag (a readable, in-range
    /// footer `summary_start`), same as
    /// [`user_message_index`](Self::user_message_index).
    pub fn user_frames(&self) -> BagResult<UserFrameWalk<'_>> {
        let bytes = self.bytes();
        Ok(UserFrameWalk {
            walker: self.frame_walker()?,
            base: bytes.as_ptr() as usize,
            total_len: bytes.len(),
        })
    }

    /// A fresh [`FrameWalker`] over this bag's finalized data section (the
    /// region `[MAGIC .. summary_start)`). Each caller gets its OWN walk —
    /// multiple sequential re-walks of the map are the intended pattern (the
    /// walk touches only record framing; the map is page-cache-hot on a repeat
    /// walk).
    fn frame_walker(&self) -> BagResult<FrameWalker<'_>> {
        let bytes = self.bytes();
        let data_end = self.finalized_data_end()?;
        // The data section starts right after the leading 8-byte magic. The map
        // base anchors the walk's ABSOLUTE file offsets (the advise-behind watermark).
        Ok(FrameWalker::new(
            &bytes[crate::record::MAGIC.len()..data_end],
            bytes.as_ptr() as usize,
        ))
    }

    /// The end of the data section (the footer's `summary_start`) for a FINALIZED
    /// bag, with the range guard [`user_message_index`](Self::user_message_index)
    /// and [`scheduler_trace`](Self::scheduler_trace) both need before they slice
    /// `[MAGIC .. data_end)`.
    ///
    /// A finalized bag always carries a summary; this guards a zeroed/undersized/
    /// oversized `summary_start` rather than trusting it blindly. The lower bound
    /// is the leading 8-byte magic (the data section starts right after it): a
    /// `summary_start` corrupted into `[0, MAGIC.len())` would otherwise panic the
    /// downstream `bytes[MAGIC.len()..data_end]` slice — the finalization gate
    /// validates only the footer FINGERPRINT (frame + magics), never the
    /// `summary_start` VALUE, so this is the first place that value is trusted.
    fn finalized_data_end(&self) -> BagResult<usize> {
        let bytes = self.bytes();
        let footer = mcap::read::footer(bytes)?;
        let data_end = footer.summary_start as usize;
        if data_end < crate::record::MAGIC.len() || data_end > bytes.len() {
            return Err(BagError::Malformed {
                reason: format!(
                    "footer summary_start ({}) is out of range for a {}-byte bag — cannot \
                     read the data section without a finalized summary",
                    footer.summary_start,
                    bytes.len()
                ),
            });
        }
        Ok(data_end)
    }

    /// Decode a single message payload as a scheduler-trace record: `Ok(None)`
    /// when `topic` is not the trace channel, a hard [`BagError::Malformed`] for a
    /// wrong-length payload on it, else the decoded [`TraceRingRecord`]. Operates
    /// on a borrowed `&[u8]` so it serves BOTH the zero-copy walks
    /// ([`scheduler_trace`](Self::scheduler_trace) /
    /// [`trace_records`](Self::trace_records) — slices into the map) and the
    /// [`recover_scheduler_trace`](Self::recover_scheduler_trace) crash path (a
    /// [`BagMessage`]'s owned data). `trace_index` is the 0-based file-order
    /// index of this record WITHIN the trace channel — carried into the
    /// wrong-length error so a corrupt record is locatable.
    fn decode_trace_slice(
        topic: &str,
        data: &[u8],
        trace_index: usize,
    ) -> BagResult<Option<TraceRingRecord>> {
        if topic != SCHEDULER_TRACE_TOPIC {
            return Ok(None);
        }
        if data.len() != TRACE_RECORD_SIZE as usize {
            return Err(BagError::Malformed {
                reason: format!(
                    "channel {SCHEDULER_TRACE_TOPIC:?} message (trace record {trace_index}) \
                     payload is {} bytes, expected exactly {TRACE_RECORD_SIZE} (a \
                     TraceRingRecord)",
                    data.len()
                ),
            });
        }
        let arr: [u8; TRACE_RECORD_SIZE as usize] = data.try_into().unwrap();
        Ok(Some(TraceRingRecord::from_bytes(&arr)))
    }

    /// The channels (from the summary), each with its decoded `cerulion`
    /// descriptor. Falls back to streaming when the summary is absent
    /// (truncated bag) — then only channels that carry at least one message
    /// appear.
    pub fn channels(&self) -> BagResult<Vec<BagChannel>> {
        if let Some(summary) = mcap::Summary::read(&self.data)? {
            let mut out: Vec<BagChannel> = Vec::with_capacity(summary.channels.len());
            for chan in summary.channels.values() {
                out.push(Self::decode_channel(chan)?);
            }
            out.sort_by_key(|c| c.id);
            return Ok(out);
        }
        // No summary: recover channels from the message stream.
        let mut seen: std::collections::BTreeMap<u16, BagChannel> =
            std::collections::BTreeMap::new();
        let stream = mcap::MessageStream::new_with_options(
            &self.data,
            mcap::read::Options::IgnoreEndMagic.into(),
        )?;
        for item in stream {
            match item {
                Ok(m) => {
                    if let std::collections::btree_map::Entry::Vacant(e) = seen.entry(m.channel.id)
                    {
                        e.insert(Self::decode_channel(&m.channel)?);
                    }
                }
                Err(_) => break, // torn tail — stop at the last complete record
            }
        }
        Ok(seen.into_values().collect())
    }

    fn decode_channel(chan: &mcap::Channel<'_>) -> BagResult<BagChannel> {
        let (schema_name, schema_encoding, descriptor) = match &chan.schema {
            Some(s) => {
                let descriptor = if s.encoding == SCHEMA_ENCODING {
                    Some(SchemaDescriptor::decode(&s.data)?)
                } else {
                    None
                };
                (s.name.clone(), s.encoding.clone(), descriptor)
            }
            None => (String::new(), String::new(), None),
        };
        Ok(BagChannel {
            id: chan.id,
            topic: chan.topic.clone(),
            schema_name,
            schema_encoding,
            message_encoding: chan.message_encoding.clone(),
            descriptor,
            provisioning: crate::ChannelProvisioning::from_metadata(chan.metadata.iter()),
        })
    }

    /// Iterate every message in FILE order — i.e. the order they were written
    /// (the writer emits messages in `write_message` call order; the `mcap`
    /// crate's `MessageStream` does not sort). Log-time monotonicity is the
    /// RECORDER's responsibility: Cerulion's recorder writes in arrival order,
    /// which is the determinism contract, so file order == arrival order.
    /// Fails hard on the first read error (use
    /// [`recover_messages`](Self::recover_messages) for a possibly-truncated
    /// bag).
    pub fn messages(&self) -> BagResult<impl Iterator<Item = BagResult<BagMessage>> + '_> {
        Ok(mcap::MessageStream::new(&self.data)?
            .map(|r| r.map(BagMessage::from).map_err(BagError::from)))
    }

    /// STREAM the messages on ONE topic out of a possibly-truncated bag,
    /// copying ONLY the payloads that match.
    ///
    /// [`recover_messages`](Self::recover_messages) collects EVERY message into
    /// a `Vec<BagMessage>`, and `BagMessage` owns its payload
    /// (`m.data.into_owned()`), so a caller that wants one reserved channel out
    /// of a multi-gigabyte recording pays for the whole bag in resident memory
    /// before it can filter. That is what this exists to avoid: the topic test
    /// runs against the borrowed `mcap::Message` BEFORE the `BagMessage`
    /// conversion, so a non-matching payload is never copied at all, and the
    /// underlying `MessageStream` decompresses one chunk at a time — the peak
    /// is one chunk plus whatever the caller chooses to keep.
    ///
    /// Same recovery semantics as `recover_messages` (`IgnoreEndMagic`, so a
    /// crash-truncated tail is readable rather than fatal). The DIFFERENCE is
    /// who classifies the ending: this yields the mid-stream read error as an
    /// `Err` ITEM and stops there, leaving the caller to decide what a torn
    /// tail means for it, because a streaming reader cannot both return a
    /// summary verdict and stay lazy. A caller that needs
    /// [`BagCompleteness`] should use `recover_messages`.
    pub fn recover_messages_on_topic<'a>(
        &'a self,
        topic: &'a str,
    ) -> BagResult<impl Iterator<Item = BagResult<BagMessage>> + 'a> {
        let stream = mcap::MessageStream::new_with_options(
            &self.data,
            mcap::read::Options::IgnoreEndMagic.into(),
        )?;
        let mut ended = false;
        Ok(stream.filter_map(move |item| {
            if ended {
                return None;
            }
            match item {
                // The topic test rides the BORROWED message, so a payload on
                // any other channel is skipped without ever being copied.
                Ok(m) if m.channel.topic == topic => Some(Ok(BagMessage::from(m))),
                Ok(_) => None,
                Err(e) => {
                    ended = true;
                    Some(Err(BagError::from(e)))
                }
            }
        }))
    }

    /// All complete messages from a possibly-truncated bag, plus how the bag
    /// ENDED ([`BagCompleteness`]). Complete messages before a torn tail or a
    /// bad chunk CRC are recovered; the corruption is REPORTED via
    /// [`BagCompleteness::TornTail`] (never silently swallowed), and a clean
    /// stream end is further classified as
    /// [`Finalized`](BagCompleteness::Finalized) vs
    /// [`TruncatedAtChunkBoundary`](BagCompleteness::TruncatedAtChunkBoundary)
    /// by fingerprinting the finalization epilogue — the stream alone cannot
    /// tell them apart. This is the crash-recovery path.
    pub fn recover_messages(&self) -> BagResult<(Vec<BagMessage>, BagCompleteness)> {
        let stream = mcap::MessageStream::new_with_options(
            &self.data,
            mcap::read::Options::IgnoreEndMagic.into(),
        )?;
        let mut out = Vec::new();
        for item in stream {
            match item {
                Ok(m) => out.push(BagMessage::from(m)),
                Err(e) => {
                    return Ok((out, BagCompleteness::TornTail(BagError::from(e))));
                }
            }
        }
        let completeness = if self.has_finalization_epilogue() {
            BagCompleteness::Finalized
        } else {
            BagCompleteness::TruncatedAtChunkBoundary
        };
        Ok((out, completeness))
    }

    /// `true` iff the file carries the finalization epilogue: both magics (the
    /// `mcap` crate's `footer()` checks them and parses the trailing footer
    /// fields) AND the Footer record's own frame (opcode `0x02`, content length
    /// 20) immediately before those fields — a fingerprint no
    /// truncated-at-a-boundary file matches.
    fn has_finalization_epilogue(&self) -> bool {
        if mcap::read::footer(&self.data).is_err() {
            return false;
        }
        // ... magic(8) | [0x02][20u64 LE] footer-frame(9) | fields(20) | magic(8)
        const TAIL: usize = 9 + 20 + 8;
        if self.data.len() < TAIL {
            return false;
        }
        let frame = &self.data[self.data.len() - TAIL..self.data.len() - TAIL + 9];
        frame[0] == 0x02 && frame[1..9] == 20u64.to_le_bytes()
    }

    /// All attachments (via the summary's attachment index).
    pub fn attachments(&self) -> BagResult<Vec<BagAttachment>> {
        let summary = match mcap::Summary::read(&self.data)? {
            Some(s) => s,
            None => return Ok(Vec::new()),
        };
        let mut out = Vec::with_capacity(summary.attachment_indexes.len());
        for index in &summary.attachment_indexes {
            let a = mcap::read::attachment(&self.data, index)?;
            out.push(BagAttachment {
                name: a.name,
                media_type: a.media_type,
                log_time: a.log_time,
                create_time: a.create_time,
                data: a.data.into_owned(),
            });
        }
        Ok(out)
    }

    /// The attachment with the given `name`, if present.
    pub fn attachment(&self, name: &str) -> BagResult<Option<BagAttachment>> {
        Ok(self.attachments()?.into_iter().find(|a| a.name == name))
    }

    /// The bag's OWN schema provenance — the custom-type text + the
    /// `schema_hash` → name bindings its recorder resolved, from the
    /// [`SCHEMA_DOCS_ATTACHMENT`](crate::SCHEMA_DOCS_ATTACHMENT) attachment.
    ///
    /// WARN-NEVER-REFUSE, following the `__cerulion/recorder.json` precedent:
    /// every failure mode answers `None`, because schema provenance
    /// is ADDITIVE — a bag whose frames are perfectly playable must not become
    /// unplayable because its schema attachment is unreadable. That is why this
    /// returns a bare `Option` and not a `BagResult`: there is no error a caller
    /// could act on differently, so there is no `Err` to hand them (unlike the
    /// neighbouring [`attachment`](Self::attachment), which genuinely does return
    /// a `Result`).
    ///
    /// - ABSENT ⇒ `None`, silent. Every bag recorded before the attachment
    ///   existed, and every bag whose topics are all built-in types, is in this
    ///   state by construction.
    /// - MALFORMED / a FUTURE catalog version ⇒ `None` + a loud `warn!` naming
    ///   the reason, so the operator learns why the types did not resolve
    ///   instead of watching an empty scene.
    /// - The attachment index itself unreadable ⇒ `None` + a `debug!` (a
    ///   non-finalized bag has no summary; the caller's own finalization gate is
    ///   the loud one).
    pub fn schema_catalog(&self) -> Option<crate::BagSchemaCatalog> {
        let attachment = match self.attachment(crate::SCHEMA_DOCS_ATTACHMENT) {
            Ok(Some(a)) => a,
            Ok(None) => return None,
            Err(e) => {
                tracing::debug!(
                    error = %e,
                    "bag: could not read the attachment index while looking for the schema \
                     catalog — treating the bag as carrying no schema provenance"
                );
                return None;
            }
        };
        match crate::BagSchemaCatalog::decode(&attachment.data) {
            Ok(c) => Some(c),
            Err(e) => {
                tracing::warn!(
                    attachment = crate::SCHEMA_DOCS_ATTACHMENT,
                    error = %e,
                    "bag: the schema-provenance attachment is unreadable — the bag still plays \
                     byte-verbatim, but any type this machine has not compiled will not resolve"
                );
                None
            }
        }
    }

    /// Decode the `__cerulion/scheduler_trace` channel to [`TraceRingRecord`]s
    /// (every record kind, in FILE order — the record-type filtering is the replay
    /// gate's job). STRICT: requires a FINALIZED bag — a readable, in-range footer
    /// `summary_start` (a stricter precondition than a
    /// `MessageStream` walk, which never reads the footer) — and fails hard on
    /// torn/corrupt framing (use
    /// [`recover_scheduler_trace`](Self::recover_scheduler_trace) after a recorder
    /// crash). A payload whose length is not exactly [`TRACE_RECORD_SIZE`] (40) is
    /// a hard error naming the channel + the record's file-order index.
    ///
    /// Memory bound: this walks the finalized bag's data section directly
    /// (via [`trace_records`](Self::trace_records)) and decodes each trace record
    /// from its BORROWED 40-byte slice — it does NOT go through the `mcap`
    /// crate's [`MessageStream`](mcap::MessageStream), which COPIES every
    /// uncompressed payload into an owned `Vec<u8>` (via [`BagMessage`]) to give
    /// `Message` a `'static` lifetime.
    ///
    /// This MATERIALIZES the whole channel (`O(record-count)` × 40 B). The
    /// memory scaler's streaming consumers use
    /// [`trace_records`](Self::trace_records) directly instead — this collector
    /// remains for small-trace tools and tests.
    pub fn scheduler_trace(&self) -> BagResult<Vec<TraceRingRecord>> {
        self.trace_records()?.collect()
    }

    /// Stream the `__cerulion/scheduler_trace` channel decoded to
    /// [`TraceRingRecord`]s (every record kind, in FILE order), WITHOUT
    /// materializing the channel: each record decodes from its borrowed 40-byte
    /// slice into the map as the iterator advances — the memory scaler's
    /// trace source (a 300 GB-scale bag's trace channel is ~40 B × hundreds of
    /// millions of records; collecting it would dominate replay's heap).
    ///
    /// Each call starts a FRESH walk of the data section — multiple sequential
    /// re-walks are the intended pattern (the framing walk is cheap and the map
    /// is page-cache-hot on a repeat walk). Same contract as
    /// [`scheduler_trace`](Self::scheduler_trace): requires a FINALIZED bag (a
    /// readable, in-range footer `summary_start` — the footer requirement the
    /// old stock-stream path did not have), fails hard on torn/corrupt framing
    /// or a wrong-length trace payload, at the same positions in the stream.
    pub fn trace_records(&self) -> BagResult<TraceRecordIter<'_>> {
        Ok(TraceRecordIter {
            walker: self.frame_walker()?,
            trace_index: 0,
            done: false,
        })
    }

    /// The crash-tolerant twin of [`scheduler_trace`](Self::scheduler_trace)
    /// for replay tooling: decodes trace records from every COMPLETE chunk of a
    /// possibly-truncated bag and surfaces how the bag ended
    /// ([`BagCompleteness`]). The 40-byte payload check still fails hard — a
    /// wrong-sized trace record is data corruption, not truncation.
    pub fn recover_scheduler_trace(&self) -> BagResult<(Vec<TraceRingRecord>, BagCompleteness)> {
        let (msgs, completeness) = self.recover_messages()?;
        let mut out = Vec::new();
        for m in &msgs {
            // `out.len()` is the trace-channel ordinal: only trace-channel
            // messages reach the decode's push, so it counts exactly them.
            if let Some(rec) = Self::decode_trace_slice(&m.topic, &m.data, out.len())? {
                out.push(rec);
            }
        }
        Ok((out, completeness))
    }
}

// ===========================================================================
// The pull-based framing walk (the ONE walk machinery)
// ===========================================================================

/// One in-progress section of a [`FrameWalker`] walk: a borrowed buffer (the
/// data section, or an uncompressed chunk body), the cursor position within it,
/// and the section name for error messages.
struct WalkSection<'a> {
    buf: &'a [u8],
    pos: usize,
    name: &'static str,
}

/// The pull-based record-framing walker over a finalized bag's data section —
/// the ONE framing walk shared by [`BagReader::user_message_index`] (records
/// user [`FrameSpan`]s) and [`BagReader::trace_records`] /
/// [`BagReader::scheduler_trace`] (decode trace records), so the zero-copy
/// readers cannot drift.
///
/// Hand-walks the record framing (`[opcode u8][len u64 LE][body]`), descending
/// into each uncompressed Chunk body via an explicit section stack (pull-based,
/// so an [`Iterator`] can suspend mid-walk — the closure-visitor form this
/// replaces could not). Each yielded payload is a slice BORROWED from the map
/// (`'a` — [`mcap::parse_record`] borrows Message/Chunk data from its input;
/// the defensive `Cow::Owned` arms refuse an unexpected copy loudly). The
/// channel→topic mapping accumulates as Channel records are encountered
/// (channels are written before the chunks that reference them); a message on
/// an unknown channel is refused.
///
/// Every section must be consumed EXACTLY: both legitimate shapes end on a
/// record boundary (the data section spans `[MAGIC .. summary_start)`, which
/// the writer closes with the DataEnd record; a chunk body is exactly the
/// framed message records the writer packed). 1–8 residue bytes cannot even
/// hold a record frame, so any leftover is corruption — refused loudly rather
/// than silently dropped (an under-indexed topic would otherwise mislabel the
/// replay diff verdict).
struct FrameWalker<'a> {
    /// The section stack: the data section at the bottom, the currently-walked
    /// chunk body (if any) on top. MCAP chunks do not nest, so depth ≤ 2 in
    /// practice; the stack keeps the walk shape-generic like the old recursion.
    stack: Vec<WalkSection<'a>>,
    /// channel id → topic, accumulated from Channel records in walk order.
    channel_topics: std::collections::HashMap<u16, String>,
    /// The map base address (`bytes.as_ptr()`), so [`file_offset`](Self::file_offset)
    /// reports ABSOLUTE bag offsets — the advise-behind watermark input.
    map_base: usize,
    /// The absolute file offset of the data section's end — the reported
    /// frontier once the walk is complete (the stack is empty).
    end_offset: usize,
}

impl<'a> FrameWalker<'a> {
    fn new(data_section: &'a [u8], map_base: usize) -> Self {
        let start = data_section.as_ptr() as usize - map_base;
        Self {
            stack: vec![WalkSection {
                buf: data_section,
                pos: 0,
                name: "data section",
            }],
            channel_topics: std::collections::HashMap::new(),
            map_base,
            end_offset: start + data_section.len(),
        }
    }

    /// The absolute bag file offset the walk has READ up to — the frontier of
    /// touched bytes (the advise-behind watermark input). Every record the
    /// walk yields is decoded/copied out at or before this offset, so pages
    /// strictly behind the MIN of all live walks' frontiers are safe to evict.
    /// Computed from the current top section (`section_file_start + pos`), which
    /// is uniform for the data section AND a chunk sub-slice (both borrow the
    /// map); the stored `end_offset` covers the walk-complete (empty-stack) case.
    fn file_offset(&self) -> usize {
        match self.stack.last() {
            Some(top) => (top.buf.as_ptr() as usize - self.map_base) + top.pos,
            None => self.end_offset,
        }
    }

    /// Advance to the next MESSAGE record, returning its channel id + payload
    /// slice (borrowed from the MAP — lifetime `'a`, not `&self`, so the caller
    /// can hold it across further walking). `Ok(None)` = the walk consumed the
    /// data section cleanly.
    fn next_message(&mut self) -> BagResult<Option<(u16, &'a [u8])>> {
        use mcap::records::Record;
        loop {
            let Some(top) = self.stack.last_mut() else {
                return Ok(None); // walk complete.
            };
            let (buf, pos, section) = (top.buf, top.pos, top.name);
            if pos + 9 > buf.len() {
                // Loud residue guard: a section ends when fewer than 9 bytes
                // (one record frame) remain — a valid section leaves EXACTLY 0.
                // Anything else means truncated/garbage framing that slipped
                // the completeness stream.
                if pos != buf.len() {
                    return Err(BagError::Malformed {
                        reason: format!(
                            "{} byte(s) of residue after the last complete record in the \
                             {section} ({} bytes total) — too short to be a record frame; the \
                             recording is corrupt",
                            buf.len() - pos,
                            buf.len()
                        ),
                    });
                }
                self.stack.pop();
                continue;
            }
            let opcode = buf[pos];
            let len = u64::from_le_bytes(buf[pos + 1..pos + 9].try_into().unwrap()) as usize;
            let body_start = pos + 9;
            let body_end = body_start
                .checked_add(len)
                .ok_or_else(|| BagError::Malformed {
                    reason: format!("record length {len} at offset {pos} overflows the bag"),
                })?;
            if body_end > buf.len() {
                return Err(BagError::Malformed {
                    reason: format!(
                        "record (opcode {opcode}) at offset {pos} claims {len} body bytes but \
                         only {} remain in the {section}",
                        buf.len() - body_start
                    ),
                });
            }
            top.pos = body_end;
            // Content-parse ONLY the record kinds the walk consumes (Channel /
            // Message / Chunk). Everything else — MessageIndex in particular —
            // is skipped by opcode AFTER the framing bounds checks above:
            // `mcap::parse_record` materializes a MessageIndex's per-message
            // offset table as an owned Vec (~16 B × messages-per-chunk, ~1 MB
            // per 4 MiB chunk of small frames), a transient the memory
            // scaler cannot afford on every re-walk of a 300 GB bag. Framing
            // integrity (lengths, residue) is still validated for EVERY record;
            // only the unconsumed kinds' CONTENT goes unparsed.
            if !matches!(
                opcode,
                crate::record::op::CHANNEL | crate::record::op::MESSAGE | crate::record::op::CHUNK
            ) {
                continue;
            }
            let body = &buf[body_start..body_end];
            match mcap::parse_record(opcode, body)? {
                Record::Channel(chan) => {
                    self.channel_topics.insert(chan.id, chan.topic.clone());
                }
                Record::Message { header, data } => {
                    if !self.channel_topics.contains_key(&header.channel_id) {
                        return Err(BagError::Malformed {
                            reason: format!(
                                "message references unknown channel id {}",
                                header.channel_id
                            ),
                        });
                    }
                    let payload: &'a [u8] = match data {
                        std::borrow::Cow::Borrowed(b) => b,
                        // `parse_record` always borrows Message data from its
                        // input; defensive — an owned payload cannot be served
                        // zero-copy.
                        std::borrow::Cow::Owned(_) => {
                            return Err(BagError::Malformed {
                                reason: "message payload slice is not borrowed from the bag map \
                                         (unexpected copy — a compressed or corrupt chunk?)"
                                    .to_string(),
                            });
                        }
                    };
                    return Ok(Some((header.channel_id, payload)));
                }
                Record::Chunk { header, data } => {
                    if !header.compression.is_empty() {
                        return Err(BagError::Malformed {
                            reason: format!(
                                "chunk uses compression '{}' — the zero-copy replay index \
                                 supports only uncompressed Cerulion bags",
                                header.compression
                            ),
                        });
                    }
                    let chunk_body: &'a [u8] = match data {
                        std::borrow::Cow::Borrowed(b) => b,
                        // `parse_record` always borrows uncompressed Chunk data
                        // from its input; defensive — see the Message arm.
                        std::borrow::Cow::Owned(_) => {
                            return Err(BagError::Malformed {
                                reason: "chunk body is not borrowed from the bag map (unexpected \
                                         copy — a compressed or corrupt chunk?)"
                                    .to_string(),
                            });
                        }
                    };
                    // Walk the chunk body next (depth-first — the same order as
                    // the old recursion), resuming this section after it.
                    self.stack.push(WalkSection {
                        buf: chunk_body,
                        pos: 0,
                        name: "chunk body",
                    });
                }
                _ => {}
            }
        }
    }

    /// The topic for a channel id [`next_message`](Self::next_message) yielded
    /// (which validated the mapping exists before yielding). Empty for an
    /// unknown id — unreachable for yielded ids.
    fn topic(&self, channel_id: u16) -> &str {
        self.channel_topics
            .get(&channel_id)
            .map_or("", String::as_str)
    }
}

/// Streaming walk over the USER-channel frames of a finalized bag — see
/// [`BagReader::user_frames`]. Yields each user message as a validated
/// [`FrameSpan`] (reserved `__cerulion/*` channels are skipped); the span's
/// offset is derived from the map base and validated in-range, so a
/// non-borrowed payload (an unexpected copy — a compressed or corrupt chunk)
/// is refused loudly, exactly as the retained index does.
///
/// Not an [`Iterator`]: the topic of a yielded frame is resolved via
/// [`topic`](Self::topic) (the channel table lives inside the walk, so an
/// `Iterator` could not lend it alongside the item).
pub struct UserFrameWalk<'a> {
    walker: FrameWalker<'a>,
    /// The map base address (span offsets are map-relative).
    base: usize,
    /// The map length (span bounds validation).
    total_len: usize,
}

impl UserFrameWalk<'_> {
    /// Advance to the next USER message, returning its channel id + validated
    /// [`FrameSpan`]. `Ok(None)` = the walk consumed the data section cleanly.
    pub fn next_user_frame(&mut self) -> BagResult<Option<(u16, FrameSpan)>> {
        while let Some((channel_id, data)) = self.walker.next_message()? {
            let topic = self.walker.topic(channel_id);
            if topic.starts_with(RESERVED_PREFIX) {
                continue; // reserved (trace/manifest) — decoded separately.
            }
            let ptr = data.as_ptr() as usize;
            let offset = ptr
                .checked_sub(self.base)
                .filter(|&o| o + data.len() <= self.total_len)
                .ok_or_else(|| BagError::Malformed {
                    reason: "message payload slice is not borrowed from the bag map \
                             (unexpected copy — a compressed or corrupt chunk?)"
                        .to_string(),
                })?;
            return Ok(Some((
                channel_id,
                FrameSpan {
                    offset,
                    len: data.len(),
                },
            )));
        }
        Ok(None)
    }

    /// The topic for a channel id [`next_user_frame`](Self::next_user_frame)
    /// yielded. Empty for an unknown id — unreachable for yielded ids (the
    /// underlying walk validates the mapping before yielding).
    pub fn topic(&self, channel_id: u16) -> &str {
        self.walker.topic(channel_id)
    }

    /// The absolute bag offset this walk has read up to — the
    /// advise-behind watermark input (see `FrameWalker::file_offset`).
    pub fn file_frontier(&self) -> usize {
        self.walker.file_offset()
    }
}

/// Streaming iterator over the `__cerulion/scheduler_trace` channel — see
/// [`BagReader::trace_records`]. Yields each [`TraceRingRecord`] decoded from
/// its borrowed 40-byte slice into the map; never materializes the channel.
/// Fused on error: after yielding an `Err`, the iterator is done (the walk
/// position is no longer trustworthy).
pub struct TraceRecordIter<'a> {
    walker: FrameWalker<'a>,
    /// File-order ordinal of the NEXT trace-channel record (0-based) — names
    /// the offending record in the wrong-length error.
    trace_index: usize,
    done: bool,
}

impl TraceRecordIter<'_> {
    /// The absolute bag offset this trace walk has read up to — the
    /// advise-behind watermark input. Each yielded trace record is copied out
    /// (`TraceRingRecord::from_bytes`) at or before this offset, so nothing
    /// borrowed into the map is retained behind the frontier.
    pub fn file_frontier(&self) -> usize {
        self.walker.file_offset()
    }
}

impl Iterator for TraceRecordIter<'_> {
    type Item = BagResult<TraceRingRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        loop {
            match self.walker.next_message() {
                Ok(Some((channel_id, data))) => {
                    let topic = self.walker.topic(channel_id);
                    match BagReader::decode_trace_slice(topic, data, self.trace_index) {
                        Ok(Some(rec)) => {
                            self.trace_index += 1;
                            return Some(Ok(rec));
                        }
                        Ok(None) => continue, // not the trace channel.
                        Err(e) => {
                            self.done = true;
                            return Some(Err(e));
                        }
                    }
                }
                Ok(None) => {
                    self.done = true;
                    return None;
                }
                Err(e) => {
                    self.done = true;
                    return Some(Err(e));
                }
            }
        }
    }
}

// ===========================================================================
// Advise-behind — pure watermark-math oracle tests
// ===========================================================================
#[cfg(test)]
mod advise_behind_tests {
    use super::{advise_behind_region, page_size};

    const P: usize = 4096; // an ordinary 4 KiB page for the pure-math oracle.
    const B: usize = 256 * 1024 * 1024; // the production batch.

    #[test]
    fn page_size_is_a_power_of_two_and_at_least_4kib() {
        let ps = page_size();
        assert!(
            ps.is_power_of_two(),
            "page size {ps} must be a power of two"
        );
        assert!(ps >= 4096, "page size {ps} implausibly small");
        assert_eq!(page_size(), ps, "cached page size must be stable");
    }

    #[test]
    fn below_batch_threshold_never_advises() {
        // Watermark advanced, but not a full batch past last_end.
        assert_eq!(advise_behind_region(0, B - 1, usize::MAX, B, P), None);
        assert_eq!(advise_behind_region(B, 2 * B - P, usize::MAX, B, P), None);
        // Boundary CONTROL (the inclusive edge): exactly AT the batch
        // threshold the region fires — proving the two None cases above are
        // the threshold's doing, not a dead helper.
        assert_eq!(advise_behind_region(0, B, usize::MAX, B, P), Some((0, B)));
    }

    #[test]
    fn end_is_watermark_aligned_down_to_a_page_never_past_min() {
        // Watermark deep into a page → end rounds DOWN to the page base, so the
        // advised region ends strictly at-or-before the min cursor (the
        // never-advise-past-min invariant).
        let wm = B + 3 * P + 1234;
        let (start, end) = advise_behind_region(0, wm, usize::MAX, B, P).unwrap();
        assert_eq!(start, 0);
        assert_eq!(end, B + 3 * P, "end must be page-aligned DOWN");
        assert!(end % P == 0, "end must be page-aligned");
        assert!(end <= wm, "must never advise past the min cursor watermark");
    }

    #[test]
    fn region_starts_exactly_where_the_last_eviction_ended() {
        // Batch 1: [0, B). Batch 2 must begin at B (no gap, no overlap).
        let (s1, e1) = advise_behind_region(0, B + P, usize::MAX, B, P).unwrap();
        assert_eq!((s1, e1), (0, (B + P) & !(P - 1)));
        let (s2, e2) = advise_behind_region(e1, e1 + B + 7 * P, usize::MAX, B, P).unwrap();
        assert_eq!(s2, e1, "next region must start where the prior ended");
        assert!(e2 > s2 && (e2 - s2) >= B);
    }

    #[test]
    fn watermark_capped_to_map_len() {
        // A watermark past the map end (usize::MAX sentinel from an empty
        // contributor) is capped to the map length, then aligned down.
        let map_len = 10 * B + 777;
        let (start, end) = advise_behind_region(0, usize::MAX, map_len, B, P).unwrap();
        assert_eq!(start, 0);
        assert_eq!(end, map_len & !(P - 1));
        assert!(end <= map_len);
    }

    #[test]
    fn zero_and_not_started_cursors_never_advise() {
        // A cursor still at offset 0 (not started) → nothing behind it.
        assert_eq!(advise_behind_region(0, 0, usize::MAX, B, P), None);
        // Watermark below last_end (a transient min dip) → None, no rewind.
        assert_eq!(advise_behind_region(2 * B, B, usize::MAX, B, P), None);
        // Empty map.
        assert_eq!(advise_behind_region(0, B, 0, B, P), None);
    }

    #[test]
    fn monotonic_batched_sweep_covers_without_gaps_or_overlap() {
        // Simulate a min cursor climbing byte-by-batch; assert the advised
        // regions tile [0, aligned_frontier) contiguously and each is >= batch.
        let map_len = 8 * B + 5 * P + 99;
        let mut last = 0usize;
        let mut covered = 0usize;
        // The cursor advances in fine steps (a page at a time is finer than a
        // real per-record step, but exercises the batching identically).
        let mut wm = 0usize;
        while wm < map_len {
            wm = (wm + 3 * P).min(map_len);
            if let Some((start, end)) = advise_behind_region(last, wm, map_len, B, P) {
                assert_eq!(start, last, "no gap between consecutive advised regions");
                assert!(end - start >= B, "each syscall covers at least one batch");
                assert!(end <= wm, "never past the current min cursor");
                covered = end;
                last = end;
            }
        }
        // Everything but the final sub-batch tail is covered.
        assert!(
            covered >= map_len - B - P,
            "the sweep left more than a tail uncovered"
        );
        assert!(covered <= (map_len & !(P - 1)));
    }
}

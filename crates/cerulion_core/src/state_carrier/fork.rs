// SPDX-License-Identifier: AGPL-3.0-only
//! The FORK CARRIER — the `fork(2)` call site, the child's encode
//! path, and the parent-side reaper THREAD that makes a disposable child safe to have.
//!
//! # What lives here, and why it is not in [`super::child`]
//!
//! `child.rs` is the file the comment-stripped source walk polices, and its ban list is
//! the child-side one: no lock, no allocation, no `tracing`, no `exit`. The code in THIS module
//! runs on both sides of the fork and legitimately does two of those things:
//!
//! * the child's per-node encode takes each node's mutex — with `try_lock`, never
//!   `lock`, which is the only spelling that is
//!   safe in a process with one thread;
//! * it allocates, deliberately. The canonical order for a hash-like container
//!   needs a sort index built before the first byte, and the child is *the process that
//!   is allowed to pay for it* — the capture budget covers exactly that. Child-side allocation is
//!   sound on glibc and libmalloc because both handle the allocator's locks across
//!   `fork` (for modern glibc it is hard-wired into `fork()` itself rather than
//!   registered through `pthread_atfork`), which is an allocator-specific EXTENSION and
//!   not POSIX — stated rather than assumed, because it is the same class of
//!   platform-specific guarantee the carrier refuses to rely on for std's hook
//!   lock.
//!
//! Widening `child.rs`'s ban list to admit those would weaken the gate for the code it
//! exists to police. So the fork sits beside it instead, and the invariant this module
//! is held to is the narrower one the whole carrier shares: nothing here takes std's
//! panic-hook lock for WRITE (`state_child_discipline_test.rs` walks the directory).
//!
//! # The fork-site discipline — and the invariant that is NEW here
//!
//! **The caller must hold NO node guard, and no other lock, when it calls
//! [`fork_capture`].** A fork child has exactly one thread, so a `MutexGuard` alive at
//! the fork instant is held FOREVER in the child's image, by nobody. Every one of the
//! child's own `try_lock`s would then fail, every node would be recorded as refused,
//! and the anchor the fork exists to produce would be silently partial on every
//! cadence — a failure with no error, no log and no signal.
//!
//! It is an invariant rather than a type because the guards are held by the caller's
//! own stack frame, which no signature here can see. `take_anchor` drops them at the
//! end of its walk for exactly this reason, and
//! `state_anchor_fork_iox2_test::a_guard_held_across_the_fork_would_make_every_anchor_partial`
//! is the arm that fails if that ordering is ever reversed.
//!
//! **The caller does NOTHING between `fork()` returning 0 and [`child_main`].** That is
//! structural here rather than a rule to follow: the branch is inside [`fork_capture`],
//! so there is no window in which a caller could insert work into the child. The child's
//! setup steps are applied by `child_main` itself.
//!
//! # The SPSC invariant, and where each half of it is enforced
//!
//! The state ring's producer role is handed to the child FOR THE CHILD'S
//! LIFETIME. The parent pushes its inline parts BEFORE the fork, touches the ring not
//! at all between fork and reap, and calls
//! [`resync_after_fork`](crate::state_ring::StateRingProducer::resync_after_fork) on
//! reap. Without the resync the parent's next push overwrites the child's records and
//! `Release`-stores a lower cursor.
//!
//! Two mechanisms hold it, and neither is a lock:
//!
//! 1. **One child at a time, graph-wide** — the arm word's claim table. A
//!    worker reaching a cadence with `busy_workers != 0` declines. That is what makes
//!    "the parent touches the ring not at all between fork and reap" true *across
//!    cadences* rather than only within one.
//! 2. **The producer stays node-thread-only.** The reaper THREAD never touches the
//!    ring: it reaps, it classifies, and it hands the outcome back through one small
//!    mutex. The NODE thread drains that mailbox at its next boundary, which is
//!    exactly where the resync and the post-mortem SKIP records belong.
//!
//! # The post-mortem is EXACT, not approximate
//!
//! A child that dies partway leaves some nodes covered and some not, and the parent
//! must report the second set without contradicting the first. `node_idx` alone cannot
//! draw that line — `enter_node` is stamped BEFORE the encode — so the child bumps
//! [`ChildBreadcrumb::note_node_accounted`] once per node it has finished putting a
//! record in the ring for (a part, or its own refusal). Positions
//! `nodes_accounted..len` are precisely the ones the parent reports, so a reader never
//! holds both a part and a SKIP for one node at one step.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use super::breadcrumb::{ChildBreadcrumb, ChildPhase, MappedBreadcrumb};
use super::child::{child_main, ChildSetup, NodeCaptureOutcome, NodeEncoder};
use super::reaper::{claimant_is_alive, ChildOutcome, ChildReaper};
use crate::state::{SkipCause, StateError};
use crate::state_arm::MappedStateArm;
use crate::state_ring::{StateRingProducer, StateRingSink};

// ===========================================================================
// memory sampling — the two numbers the boundary reads out of an atomic
// ===========================================================================

/// How much [`MemAvailable`](sample_mem_available) must be left AFTER every
/// outstanding reservation for a fork to be admitted.
///
/// A fixed floor rather than a fraction of total RAM: the quantity that must not run
/// out is the headroom the kernel needs to keep the LIVE graph running while a child
/// smears, and that is an absolute number of pages, not a proportion of a machine.
/// 512 MiB is ~6 % of an 8 GB Jetson Orin and leaves room for the peak
/// the measured CoW smear reaches (a dense 256 MiB slab front-loads its whole
/// copy into the first post-fork tick).
///
/// It is deliberately NOT tunable at runtime. A knob here would be read as a
/// performance dial and turned down on the machine where it matters most.
pub const CAPTURE_MEM_FLOOR_BYTES: u64 = 512 * 1024 * 1024;

// A floor below a single anchor's plausible smear buys nothing; one above a small
// robot's whole RAM would refuse every fork forever. Pinned at COMPILE time rather than
// in a test, because it is a property of the constant and nothing else.
const _: () = assert!(CAPTURE_MEM_FLOOR_BYTES >= 128 * 1024 * 1024);
const _: () = assert!(CAPTURE_MEM_FLOOR_BYTES <= 2 * 1024 * 1024 * 1024);

/// PURE: private ANONYMOUS resident KIBIBYTES out of a `/proc/self/status` body, or
/// `None` when this kernel does not report the field.
///
/// Split out so the whole parse — including the arms a healthy box can never take —
/// is oracle-testable with no `/proc` and no second process.
///
/// `RssAnon:` is the line that answers the question the fork gate is actually
/// asking (Linux >= 4.5). It is reported in KIBIBYTES with a `kB` suffix.
///
/// NOT `#[cfg(target_os = "linux")]`, deliberately, even though only the Linux arm
/// of [`sample_anon_rss`] calls it: gating it would make its oracle unrunnable on
/// the platform this repo is authored on, and the defect it exists to prevent — a
/// helper whose NAME said anonymous and whose BODY read total RSS — is precisely
/// the kind that survives when nobody can execute the check.
pub fn parse_rss_anon_kib(status: &str) -> Option<u64> {
    for line in status.lines() {
        let Some(rest) = line.strip_prefix("RssAnon:") else {
            continue;
        };
        // `RssAnon:\t   12345 kB` — take the FIRST whitespace-separated token after
        // the label and require it to be a number. The UNIT is checked rather than
        // assumed: reading kB as B would under-report by 1024x, which at this gate
        // means admitting a fork nobody could afford.
        let mut fields = rest.split_whitespace();
        let value: u64 = fields.next()?.parse().ok()?;
        if fields.next()? != "kB" {
            return None;
        }
        return Some(value);
    }
    None
}

/// This process's PRIVATE ANONYMOUS resident memory, in bytes, or `None` if it
/// cannot be read.
///
/// The memory-gate PROJECTION: a sound upper bound on what a capture child can cost,
/// because the child's cost is the subset of these pages the parent writes while it
/// lives. A tighter figure is possible, the "last observed `rss_delta`", which is a
/// refinement in the safe direction only — using the bound always DECLINES more often,
/// never fewer times, so it can starve an anchor but can never admit one the tighter
/// figure would have refused. The refinement is deliberately not implemented.
///
/// # PRIVATE ANONYMOUS, not total RSS — and the difference is the whole point
///
/// Total RSS
/// (`/proc/self/statm` field 2) counts every resident page including
/// file-backed and `MAP_SHARED` ones. The measured fork-cost basis says
/// `MAP_SHARED` is ≈ FREE at fork (`vma_needs_copy`, verified on real L4T) — those
/// pages are not copied and cost the child nothing — so pricing them makes the
/// number wrong in the one direction that hurts: a robot with big resident iceoryx
/// pools would read as a giant-state process and lose its capture plane EXACTLY where
/// the pools prove it is doing real work.
///
/// So Linux reads `RssAnon:` from `/proc/self/status`, which is the quantity a fork
/// actually pays for.
///
/// ## The two degradations
///
/// * **A kernel with no `RssAnon:`** (pre-4.5, 2016) falls back to total RSS. That
///   OVER-reports, which is the safe direction for the reservation this feeds —
///   `None` becomes `unwrap_or(0)` at the caller, i.e. NO reservation, letting every
///   worker fork against the same memory, which is the OOM the memory gate exists to
///   prevent. It can cause a false refusal at the arm-time gate on such a
///   kernel: a black box lost on a ten-year-old kernel, against an OOM on a live
///   robot.
/// * **macOS** has no per-region private/shared split in the `libc`-exposed Mach
///   surface (`mach_task_basic_info.resident_size` is TOTAL; `task_vm_info` is not
///   in `libc`, and two symbols do not earn a `mach2` dependency). The dev platform
///   therefore keeps the over-reporting figure, with the same trade: conservative
///   for the reservation, capable of a false arm-time refusal on a desk with large
///   SHM pools. The ROBOT is Linux, which is where this is exact.
///
/// Sampled by the reaper thread, never at the boundary: it is a `/proc` read on Linux
/// and a Mach RPC on macOS, and the boundary makes no syscall it can avoid.
pub fn sample_anon_rss() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        // THE ANSWER, when the kernel gives it: private anonymous resident KiB.
        if let Some(kib) = std::fs::read_to_string("/proc/self/status")
            .ok()
            .as_deref()
            .and_then(parse_rss_anon_kib)
        {
            return Some(kib.saturating_mul(1024));
        }
        // Pre-4.5 fallback: total RSS in PAGES from `/proc/self/statm` field 2. It
        // over-reports (see the docs above) — deliberately, because the caller's
        // `unwrap_or(0)` turns a `None` into NO reservation at all.
        let raw = std::fs::read_to_string("/proc/self/statm").ok()?;
        let pages: u64 = raw.split_whitespace().nth(1)?.parse().ok()?;
        // SAFETY: `sysconf` takes an integer name and returns a long.
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
        if page <= 0 {
            return None;
        }
        Some(pages.saturating_mul(page as u64))
    }
    #[cfg(target_vendor = "apple")]
    {
        let mut info: libc::mach_task_basic_info = unsafe { std::mem::zeroed() };
        let mut count = libc::MACH_TASK_BASIC_INFO_COUNT;
        // SAFETY: a caller-owned info struct and its element count, per
        // `<mach/task_info.h>`; `mach_task_self` names this task.
        //
        // `libc` deprecates the mach ports in favour of the `mach2` crate. Taking a
        // whole extra dependency for TWO symbols on the dev platform is the wrong
        // trade — the robot is Linux, and neither signature has changed since the
        // deprecation was added.
        #[allow(deprecated)]
        let kr = unsafe {
            libc::task_info(
                libc::mach_task_self(),
                libc::MACH_TASK_BASIC_INFO,
                &mut info as *mut _ as libc::task_info_t,
                &mut count,
            )
        };
        if kr != 0 {
            return None;
        }
        Some(info.resident_size)
    }
    #[cfg(not(any(target_os = "linux", target_vendor = "apple")))]
    {
        None
    }
}

/// Memory the kernel believes can be handed out without swapping, in bytes, or `None`
/// where this platform cannot say.
///
/// Linux reads `MemAvailable` — the kernel's OWN estimate, which already discounts the
/// unreclaimable half of the page cache, and is why it is used rather than `MemFree`.
/// macOS has no equivalent file, so the substitute is computed from
/// `host_statistics64`: free + inactive + purgeable + speculative pages, which is the
/// same set `vm_stat`-based tools treat as available.
///
/// `None` means UNKNOWN and is NOT a refusal — see [`memory_verdict`].
pub fn sample_mem_available() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let raw = std::fs::read_to_string("/proc/meminfo").ok()?;
        for line in raw.lines() {
            let Some(rest) = line.strip_prefix("MemAvailable:") else {
                continue;
            };
            let kib: u64 = rest.split_whitespace().next()?.parse().ok()?;
            return Some(kib.saturating_mul(1024));
        }
        None
    }
    #[cfg(target_vendor = "apple")]
    {
        let mut stats: libc::vm_statistics64 = unsafe { std::mem::zeroed() };
        let mut count = libc::HOST_VM_INFO64_COUNT;
        // SAFETY: a caller-owned `vm_statistics64` and its element count, per
        // `<mach/host_info.h>`; `mach_host_self` names this host. `#[allow(deprecated)]`
        // for the same reason as `sample_anon_rss` — `libc` points at the `mach2`
        // crate, and two symbols do not earn a dependency.
        #[allow(deprecated)]
        let kr = unsafe {
            libc::host_statistics64(
                libc::mach_host_self(),
                libc::HOST_VM_INFO64,
                &mut stats as *mut _ as libc::host_info64_t,
                &mut count,
            )
        };
        if kr != 0 {
            return None;
        }
        // SAFETY: `sysconf` takes an integer name and returns a long.
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
        if page <= 0 {
            return None;
        }
        // Field reads go through locals because `vm_statistics64` is `repr(packed)`
        // and a reference to a packed field is unaligned.
        let free = stats.free_count as u64;
        let inactive = stats.inactive_count as u64;
        let purgeable = stats.purgeable_count as u64;
        let speculative = stats.speculative_count as u64;
        // `free_count` INCLUDES speculative pages, so they are subtracted rather than
        // added: counting them twice would over-report availability, which is the one
        // direction this number must not err in.
        let pages = free
            .saturating_sub(speculative)
            .saturating_add(inactive)
            .saturating_add(purgeable)
            .saturating_add(speculative);
        Some(pages.saturating_mul(page as u64))
    }
    #[cfg(not(any(target_os = "linux", target_vendor = "apple")))]
    {
        None
    }
}

/// What the memory gate decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryVerdict {
    /// There is headroom for this fork.
    Proceed,
    /// There is not; the reservation must be given back and the anchor skipped
    /// [`SkipCause::LowMemory`].
    Decline,
    /// This platform cannot answer. The fork PROCEEDS — see [`memory_verdict`].
    Unknown,
}

impl MemoryVerdict {
    /// Whether the fork may go ahead.
    ///
    /// [`Unknown`](Self::Unknown) counts as yes, which is the whole of the fail-OPEN
    /// decision and is why it is a distinct variant rather than folded into
    /// `Proceed`: the caller reports it, and a run whose gate is inert must be able
    /// to say so.
    pub fn may_fork(self) -> bool {
        !matches!(self, Self::Decline)
    }
}

/// The memory gate, as a pure function of the two numbers and the floor.
///
/// # Why `reserved_bytes` is read AFTER this process's own claim
///
/// The claim is taken FIRST and the total read back INCLUDING it, which is what closes
/// the simultaneity TOCTOU: every worker reaches the same `S` by
/// arithmetic, so per-process checks would evaluate concurrently — before any peer's
/// fork has consumed anything — and three map-holding workers on an 8 GB Orin would
/// each independently observe enough headroom and all fork. Counting through one shared
/// word makes the gates observe each other, at the cost of one word and no protocol.
///
/// # `None` PROCEEDS, and that is a decision rather than an oversight
///
/// A platform that cannot report available memory would otherwise never anchor at all —
/// a cost-derived refusal by another name, and exactly the outcome the watchdog's
/// liveness rule avoids. The gate is belt-and-braces over a control that IS present
/// everywhere it matters: on Linux the child sets `oom_score_adj = 1000`, so the
/// kernel's own choice under pressure is biased onto the disposable process. The
/// residual is reported ([`MemoryVerdict::Unknown`]) rather than silently absorbed.
pub fn memory_verdict(
    reserved_bytes: u64,
    mem_available: Option<u64>,
    floor_bytes: u64,
) -> MemoryVerdict {
    let Some(available) = mem_available else {
        return MemoryVerdict::Unknown;
    };
    // Saturating, so an over-committed total reads as zero headroom rather than
    // wrapping into a very large one — the one arithmetic slip here would admit
    // every fork on exactly the machine that is already out of memory.
    if available.saturating_sub(reserved_bytes) < floor_bytes {
        MemoryVerdict::Decline
    } else {
        MemoryVerdict::Proceed
    }
}

// ===========================================================================
// the child's encode path
// ===========================================================================

/// One node of the fork set, as the CHILD sees it.
///
/// An erased view for the same reason [`super::InlineCaptureTarget`] is one: the walk
/// must iterate a heterogeneous graph, `CerulionState` is not object safe, and this
/// module must not know what a `NodeEntry` is. The runtime supplies the implementation,
/// and it is the implementation that takes the node's lock — with `try_lock`.
pub trait ForkCaptureTarget {
    /// The RING MANIFEST index this node's records are stamped with.
    ///
    /// Distinct from the node's position in the fork set, which is what the breadcrumb
    /// carries. Conflating them would stamp a record against whichever node happened
    /// to sit at that offset in the manifest.
    fn node_idx(&self) -> u32;

    /// `state_shape()` — the framing token the anchor header carries.
    fn state_shape(&self) -> Option<u64>;

    /// The scheduler's view of this node at the anchor
    /// boundary (the framework section), read by the parent BEFORE the fork.
    ///
    /// Read in the parent for two reasons. It is the only side that HAS a
    /// scheduler — the child inherits a snapshot of the address space, but the
    /// carrier hands it targets, not a runtime — and it must be a value by then
    /// anyway, because the cut is the boundary instant and a section read
    /// after the fork would be read from a copy nothing is advancing.
    ///
    /// The default is the EMPTY section, which routes to the v1 header: a target
    /// that has nothing to say writes exactly the v1 bytes.
    fn framework_state(&self) -> crate::state_restore::NodeFrameworkState {
        crate::state_restore::NodeFrameworkState::default()
    }

    /// Lock this node and encode it into `sink`.
    ///
    /// MUST use `try_lock`. A `lock` here is unbounded in a process whose other
    /// threads no longer exist: if the guard were somehow held at the fork instant it
    /// would never be released, and the child would wedge until the watchdog killed it
    /// five seconds later — reported as a stalled encoder that never ran.
    ///
    /// The breadcrumb is NOT passed: the sink handed in is a [`BumpingSink`], so the
    /// liveness signal comes from the encoder's own writes without every implementation
    /// having to remember to produce it.
    fn capture(&self, sink: &mut dyn crate::state::StateSink) -> Result<(), StateError>;
}

/// A [`StateRingSink`] that bumps the child's progress word on every write.
///
/// # This is where the liveness signal comes from, and its scope is a real limit
///
/// The child bumps at every bounded unit of work, and a
/// record pushed is one. The encoder is the only code that knows where the smaller
/// units are, and the breadcrumb is not threaded into `cer_capture` — `capture_state`
/// takes a sink and nothing else — so the sink is the seam that exists TODAY, and every
/// write through it is a bounded unit by construction.
///
/// What that covers: any encoder that writes as it walks, which is every generated one.
/// A 500 MB serde encode bumps thousands of times a second, so the watchdog's five
/// seconds bound the gap between two WRITES rather than the whole encode — which is the
/// property that makes it a liveness check rather than a duration cap.
///
/// What it does NOT cover, stated because the breadcrumb's own docs raise exactly this
/// shape: the canonical order for a hash-like container builds a sort index BEFORE
/// its first byte, so a 30 M-entry map would produce no writes for the whole sort and
/// a child doing one could be killed mid-index. No shipped encoder does that. Covering
/// it takes the encoder bumping directly, which needs the breadcrumb threaded
/// through `CerulionState::cer_capture`, and that is a trait change rather than a
/// carrier change — deliberately left out, not an oversight.
pub struct BumpingSink<'a, 'ring> {
    inner: &'a mut StateRingSink<'ring>,
    crumb: &'a ChildBreadcrumb,
}

impl<'a, 'ring> BumpingSink<'a, 'ring> {
    /// Wrap a ring sink so the child's progress word tracks the encoder's writes.
    pub fn new(inner: &'a mut StateRingSink<'ring>, crumb: &'a ChildBreadcrumb) -> Self {
        Self { inner, crumb }
    }
}

impl crate::state::StateSink for BumpingSink<'_, '_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), crate::state::SinkFull> {
        // BEFORE the write, not after: the write is the thing that can BLOCK (the ring
        // waits when it is full), so bumping afterwards would leave the progress word
        // frozen for exactly the interval the backpressure phase exists to classify — and the
        // watchdog would read a backpressured child as one that made no progress at
        // all before its push even began.
        self.crumb.bump();
        // And the PHASE, for the same interval. The bump alone says only "the encoder
        // got this far"; it is the phase that tells the watchdog WHY it then stopped.
        // Without this stamp the child sits at `Encoding` for the whole blocking push,
        // so a recorder that stopped draining is classified `Stalled` → `ChildTimeout`
        // → `blames_the_node()` — the node named in the operator's line is the one
        // thing that was working. The backpressure phase exists to prevent exactly that, and this
        // is the only place on the production path that can arm it.
        super::child::push_with_backpressure_phase(self.crumb, || self.inner.write(bytes))
    }

    fn remaining_hint(&self) -> Option<usize> {
        // Delegated rather than defaulted, so this wrapper cannot silently turn a
        // bounded sink into an unbounded one if it is ever put over a different sink.
        self.inner.remaining_hint()
    }

    fn refuse(&mut self) {
        self.inner.refuse();
    }
}

/// The `NodeEncoder` the fork child runs: one node of the fork set per call, each into
/// its own ring sink, each accounted for before the next begins.
///
/// # A refused node is recorded by the CHILD, not reported to the parent
///
/// The child is the only process that knows WHICH node refused and why — the failure
/// lives in its own address space and dies with it. So it pushes the
/// [`SkipCause::CaptureFailed`] record itself, while it still owns the producer, and
/// then accounts for the node. The parent's `CHILD_EXIT_CAPTURE_FAILED` status is
/// corroboration for the operator, never the source of the record.
pub struct ForkSetEncoder<'a, T: ForkCaptureTarget> {
    targets: &'a [T],
    producer: &'a mut StateRingProducer,
    step: u64,
}

impl<'a, T: ForkCaptureTarget> ForkSetEncoder<'a, T> {
    /// Build the encoder for one boundary's fork set.
    pub fn new(targets: &'a [T], producer: &'a mut StateRingProducer, step: u64) -> Self {
        Self {
            targets,
            producer,
            step,
        }
    }
}

impl<T: ForkCaptureTarget> NodeEncoder for ForkSetEncoder<'_, T> {
    fn len(&self) -> u32 {
        self.targets.len() as u32
    }

    fn encode(&mut self, idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome {
        let Some(target) = self.targets.get(idx as usize) else {
            // Unreachable through `child_main`, which iterates `0..len()`. Reported as
            // a failure rather than silently accounted for: an out-of-range index
            // means the fork set and the encoder disagree about their own length, and
            // accounting for a node that does not exist would tell the parent a real
            // node had been covered.
            return NodeCaptureOutcome::Failed;
        };
        let node_idx = target.node_idx();
        let outcome = match target.state_shape() {
            None => {
                // A node in the fork set that declares no shape cannot be framed, and
                // an unframed blob is one a reader must refuse. `capture_state`'s own
                // default refuses for exactly this population, so reaching here means
                // a hand implementation disagreed with itself.
                // Guarded like every other child-side ring write: this is a push, so
                // a full ring blocks here too.
                let (producer, step) = (&mut self.producer, self.step);
                super::child::push_with_backpressure_phase(crumb, || {
                    producer.push_skip(
                        step,
                        node_idx,
                        SkipCause::CaptureFailed,
                        "the capture child reached a node that declares no `state_shape()`, \
                         so its anchor cannot be framed",
                    )
                });
                NodeCaptureOutcome::Failed
            }
            Some(shape) => {
                let mut sink = self.producer.sink(self.step, node_idx);
                // The HEADER is the CARRIER's, so it goes in unwrapped: it is a fixed
                // 16 bytes (24 plus the section, under the v2 framing) and bumping
                // for it would let a child that encoded NOTHING still look like it had
                // made progress.
                //
                // A DEFERRED node's anchor carries the scheduler's
                // framework section on exactly the same terms as an inline one. The two
                // carriers emit IDENTICAL bytes by design, so a section on one
                // path and not the other would make a node's restorable timing state
                // depend on whether its struct happened to be inline-eligible.
                let framework = target.framework_state();
                // The header rides the phase guard but NOT the bump (see the note
                // above on why a header write must not look like encoder progress).
                // It needs the guard for the same reason the body does — it is the
                // node's FIRST record, so a full ring blocks here before the encoder
                // has written a byte — and it is the case where the two words must
                // disagree: frozen progress is the truth, and the phase is what says
                // the recorder caused it.
                let framed = super::child::push_with_backpressure_phase(crumb, || {
                    if framework.is_empty() {
                        crate::state_restore::AnchorBlob::write_header(shape, &mut sink)
                    } else {
                        crate::state_restore::AnchorBlob::write_header_with_framework(
                            shape, &framework, &mut sink,
                        )
                    }
                });
                let result = match framed {
                    Ok(()) => {
                        let mut bumping = BumpingSink::new(&mut sink, crumb);
                        target.capture(&mut bumping)
                    }
                    Err(e) => Err(e),
                };
                match result {
                    Ok(()) => {
                        // `finish` EMITS the final record, so it pushes — and on a full
                        // ring it blocks exactly as the body writes do. Leaving it
                        // outside the guard meant a child that encoded a whole node and
                        // then stalled at its LAST push published `Encoding`, and the
                        // watchdog blamed the node for a recorder that had stopped
                        // draining. The guard is per WRITE, not per node, so every
                        // child-side push carries it.
                        super::child::push_with_backpressure_phase(crumb, || sink.finish());
                        NodeCaptureOutcome::Done
                    }
                    Err(_) => {
                        // DROPPED, never finished — and this is the arm a mutation run
                        // found. `finish` emits the FINAL record, so a node whose
                        // encoder failed AFTER the 16-byte header went in would be
                        // published as a COMPLETE anchor carrying an empty payload:
                        // `AnchorBlob::decode` accepts it, a restore applies it, and the
                        // node comes back with its state silently blank. The SKIP record
                        // pushed below would sit beside it, so a reader would hold a
                        // valid-looking anchor AND a refusal for one node at one step.
                        //
                        // Dropping publishes nothing at all when nothing has filled a
                        // record yet (the common case — a header is 16 bytes against a
                        // 480-byte payload), and leaves an explicitly TRUNCATED stream
                        // when the encode had already flushed records. Both are accurate,
                        // and the skip below names the cause.
                        drop(sink);
                        // The detail is a fixed string rather than the error's text:
                        // the SKIP record's detail field is bounded, and the operator's
                        // durable evidence for WHY is the node's own logging on the
                        // next healthy cadence. What matters here is that the node is
                        // named and its absence has a cause.
                        let (producer, step) = (&mut self.producer, self.step);
                        super::child::push_with_backpressure_phase(crumb, || {
                            producer.push_skip(
                                step,
                                node_idx,
                                SkipCause::CaptureFailed,
                                "the node's encoder refused inside the capture child",
                            )
                        });
                        NodeCaptureOutcome::Failed
                    }
                }
            }
        };
        // AFTER the record is in the ring, on BOTH arms: `nodes_accounted` means "the
        // reader has something for this node", which a refusal satisfies exactly as a
        // part does. Bumping only on success would make the parent re-report a node
        // the child already named.
        crumb.note_node_accounted();
        outcome
    }
}

// ===========================================================================
// the fork itself
// ===========================================================================

/// What [`fork_capture`] did, in the PARENT. The child never returns from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForkOutcome {
    /// The child is running under this pid.
    Forked {
        /// The child's process id.
        pid: i32,
    },
    /// `fork(2)` itself failed (the `ForkFailed` skip cause) — `EAGAIN` (the process or
    /// user limit) or `ENOMEM`.
    ForkFailed {
        /// The `errno` the call reported, for the operator's line.
        errno: i32,
    },
}

/// `fork(2)`, and everything that must be true at the instant it is called.
///
/// **In the child this never returns**: it goes straight into [`child_main`], which
/// applies the child's setup steps and `_exit`s. Nothing runs between the branch and that
/// call, which is why the discipline is structural here rather than a rule the caller
/// has to remember.
///
/// # Contract the caller must satisfy, and which no signature can express
///
/// * **No node guard, and no other lock, may be held.** A guard alive at the fork
///   instant is held forever in the child's image by nobody, so every one of the
///   child's `try_lock`s fails and every anchor is silently partial (see the module
///   docs).
/// * The breadcrumb must be the one the panic hook points at
///   ([`set_fork_panic_breadcrumb`](super::set_fork_panic_breadcrumb)), or a child
///   panic leaves no node to report.
/// * The parent must not touch the state ring again until the child is reaped.
///
/// The breadcrumb is RE-ARMED here rather than by the caller: it is the one action that
/// must happen after the previous child's post-mortem has been read and before this
/// child's first stamp, and putting it anywhere else leaves a window where a watchdog
/// baseline carries the previous child's watermark.
pub fn fork_capture(
    crumb: &MappedBreadcrumb,
    setup: &ChildSetup,
    encoder: &mut dyn NodeEncoder,
) -> ForkOutcome {
    crumb.rearm();
    // SAFETY: `fork` in a process whose caller holds no lock (the contract above). The
    // child branch performs only async-signal-safe work until `child_main`'s discipline
    // is applied.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        // THE CHILD. Nothing between here and `child_main`.
        child_main(crumb, setup, encoder);
    }
    if pid < 0 {
        return ForkOutcome::ForkFailed {
            errno: std::io::Error::last_os_error().raw_os_error().unwrap_or(0),
        };
    }
    ForkOutcome::Forked { pid }
}

// ===========================================================================
// the reaper thread
// ===========================================================================

/// How often the reaper thread wakes.
///
/// It must be well BELOW [`STATE_STALL_TIMEOUT_NS`](super::STATE_STALL_TIMEOUT_NS),
/// because that timeout is only as sharp as the observation interval that measures it:
/// a thread waking every ten seconds would report a five-second stall between five and
/// fifteen seconds late, and would leave a completed child unreaped — holding its claim
/// slot, and therefore every worker's next cadence — for the same span. 100 ms is two
/// orders of magnitude under the timeout and costs one `waitpid` plus a handful of
/// relaxed loads per wake.
pub const REAPER_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(100);

/// How many reaper passes between memory samples and stale-claim sweeps.
///
/// Both are syscalls (a `/proc` read or a Mach RPC; a `kill(pid, 0)` per held slot) and
/// neither needs to be fresh to the poll: the memory numbers are a floor check against
/// a slowly-moving quantity, and a stale claim has already been stale for as long as
/// its owner has been dead. Ten passes is one second at the interval above.
const REAPER_SLOW_PASS_EVERY: u32 = 10;

// The interval must stay far enough under the stall timeout that the watchdog measures
// what it claims to. Pinned at compile time rather than in a test: it is a property of
// the two constants and nothing else.
const _: () = assert!(
    (REAPER_POLL_INTERVAL.as_nanos() as u64) * 10 < super::watchdog::STATE_STALL_TIMEOUT_NS
);

/// A child that has been reaped, waiting for the node thread to act on it.
#[derive(Debug, Clone)]
pub struct FinishedChild {
    /// How it ended.
    pub outcome: ChildOutcome,
    /// The anchor step the child was capturing.
    pub step: u64,
    /// The fork set's RING MANIFEST indices, in fork-set order — so the parent can
    /// translate the breadcrumb's fork-set position into the index a record carries.
    pub node_idx: Vec<u32>,
    /// How many fork-set positions the child got a record into the ring for. The first
    /// position the parent must report is exactly this one.
    pub nodes_accounted: u32,
    /// The phase the child was last in, for the operator's line.
    pub phase: ChildPhase,
    /// The highest progress the watchdog ever saw — how far a killed child got.
    pub watermark: u64,
}

impl FinishedChild {
    /// The SKIP cause the parent must record for the nodes this child never covered,
    /// or `None` when there are none to record.
    ///
    /// The mapping is the capture failure table, and every arm of it exists because the
    /// remedies differ: a panicked encoder is a bug in code that was never meant to
    /// unwind, a stall is an encoder that is not coming back, a `RingFull` stall is the
    /// RECORDER having died and blames no node at all.
    ///
    /// `GoneStatusUnknown` maps to `None` deliberately. `ECHILD` means something else
    /// reaped the child, so this process cannot know whether it finished — and the
    /// accounting word it left behind was written by a process that may have run on
    /// afterwards. Minting a cause there would be inventing a verdict; the accurate
    /// report is the loud log the caller emits plus the absence a reader can already
    /// see.
    pub fn skip_cause(&self) -> Option<SkipCause> {
        match self.outcome {
            ChildOutcome::Completed => None,
            // The child pushed its OWN per-node refusal records while it still owned
            // the producer, so a node it failed on is already accounted for. Anything
            // past the accounting word is a node it never reached.
            ChildOutcome::CaptureFailed => Some(SkipCause::CaptureFailed),
            ChildOutcome::Panicked { .. } => Some(SkipCause::ChildPanicked),
            ChildOutcome::Signal(_) => Some(SkipCause::ChildCrashed),
            ChildOutcome::Backpressured { .. } => Some(SkipCause::RecorderBehind),
            ChildOutcome::Stalled { .. } => Some(SkipCause::ChildTimeout),
            ChildOutcome::GoneStatusUnknown => None,
        }
    }

    /// The ring manifest indices this child left NO record for — exactly the ones the
    /// parent must report.
    ///
    /// Derived from the accounting word rather than from `node_idx` in the breadcrumb,
    /// which names the node the child ENTERED and therefore over-reports by one for a
    /// child killed just after finishing it (see [`ChildBreadcrumb`]).
    pub fn uncovered(&self) -> &[u32] {
        let from = (self.nodes_accounted as usize).min(self.node_idx.len());
        &self.node_idx[from..]
    }
}

/// What a negative `waitpid` return means to the teardown wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TeardownWait {
    /// A signal arrived. NOTHING is known — keep polling within the ceiling.
    Interrupted,
    /// No such child: it is gone, and the wait got the outcome it wanted.
    Gone,
    /// An errno this wait cannot make progress on. Report it; do not infer.
    Unusable,
}

/// PURE: classify a negative `waitpid` return by errno.
///
/// Extracted because the ARM is the whole decision and the interleaves that reach the
/// non-`Gone` ones are effectively unreachable from a test (see the note below), so an
/// inline `match` would be a rule nothing could check.
///
/// Treating EVERY negative return alike — break out with the
/// child neither reaped nor known dead — is wrong in two different directions:
///
/// * `EINTR` means nothing is known YET. Breaking there abandons a child that may
///   still be running, after the reaper thread has been joined and can never collect
///   it. (Reachability: Linux documents EINTR as occurring only when
///   `WNOHANG` is NOT set, and this wait sets it; macOS does not exclude it. So this
///   arm is a correctness rule for a case Linux says cannot arise, kept because the
///   cost is one comparison and the alternative is an unreaped child.)
/// * `ECHILD` means the child IS gone — the outcome the wait wanted. Treating it as a
///   failure would emit a loud "did not become reapable within the teardown ceiling",
///   which is an affirmatively WRONG claim that sends an operator hunting an orphan
///   that does not exist. (Also narrow in practice: the reaper thread's own final pass
///   normally reaps first and clears the slot, so `terminate_in_flight_child` finds
///   nothing to do. This arm covers a child reaped by something else in the window
///   between that pass and this kill.)
///
/// Everything else is neither: reported, never inferred.
fn classify_teardown_wait(errno: Option<i32>) -> TeardownWait {
    match errno {
        Some(libc::EINTR) => TeardownWait::Interrupted,
        Some(libc::ECHILD) => TeardownWait::Gone,
        _ => TeardownWait::Unusable,
    }
}

/// A child in flight, and the fork set it is covering.
#[derive(Debug)]
struct InFlight {
    reaper: ChildReaper,
    slot: usize,
    step: u64,
    node_idx: Vec<u32>,
}

/// The state the reaper thread and the node thread share.
///
/// ONE mutex over `(in flight, finished)` rather than two, because the transition
/// between them must be atomic to the node thread: it decides "may I touch the ring?"
/// from the first and "what must I report?" from the second, and observing an empty
/// mailbox beside a cleared in-flight slot would let it fork again without ever
/// resyncing the producer the previous child advanced.
#[derive(Debug, Default)]
struct ReaperMailbox {
    in_flight: Option<InFlight>,
    finished: Vec<FinishedChild>,
}

/// The parent-side reaper THREAD — one per armed runtime.
///
/// It owns everything that must not run on the node thread: the targeted `waitpid`, the
/// progress watchdog's kill, the arm word's claim release, the
/// stale-claim sweep, and the memory samples the boundary reads out of an atomic.
///
/// # It must be STOPPED and JOINED before the breadcrumb is unmapped
///
/// The thread reads the breadcrumb through an `Arc`, so the mapping outlives it by
/// construction — but the arm word does not: a thread still sweeping claims after the
/// runtime that armed it has gone is writing into a table its owner may have
/// `shm_unlink`ed. [`stop`](Self::stop) is therefore called from `Drop` as well as from
/// the explicit disarm, and it JOINS rather than merely signalling.
#[derive(Debug)]
pub struct CaptureReaper {
    shared: Arc<ReaperShared>,
    handle: Option<std::thread::JoinHandle<()>>,
}

#[derive(Debug)]
struct ReaperShared {
    crumb: Arc<MappedBreadcrumb>,
    arm: Arc<MappedStateArm>,
    mailbox: Mutex<ReaperMailbox>,
    stop: AtomicBool,
    /// The last sample, or 0 for "not sampled / unavailable" — `Option` cannot ride an
    /// atomic, and 0 available bytes is not a state a running process observes.
    mem_available: AtomicU64,
    /// This process's resident anonymous memory at the last sample.
    anon_rss: AtomicU64,
    /// Every stale claim this reaper has reclaimed (Principle #3: the stale-claim sweep is
    /// otherwise only visible as cadences that stopped stopping).
    stale_claims_reclaimed: AtomicU64,
}

impl CaptureReaper {
    /// Start the reaper for an armed runtime.
    pub fn start(crumb: Arc<MappedBreadcrumb>, arm: Arc<MappedStateArm>) -> Self {
        let shared = Arc::new(ReaperShared {
            crumb,
            arm,
            mailbox: Mutex::new(ReaperMailbox::default()),
            stop: AtomicBool::new(false),
            mem_available: AtomicU64::new(0),
            anon_rss: AtomicU64::new(0),
            stale_claims_reclaimed: AtomicU64::new(0),
        });
        // Sampled ONCE synchronously so the very first cadence has real numbers to gate
        // on. Without it the first fork of every run is gated on zeros, which reads as
        // "no memory available" and declines exactly the anchor an operator watches for
        // when they attach a recorder.
        shared.sample_memory();
        let worker = Arc::clone(&shared);
        let handle = std::thread::Builder::new()
            .name("cerulion-state-reaper".to_string())
            .spawn(move || worker.run())
            .ok();
        if handle.is_none() {
            tracing::warn!(
                "the checkpoint reaper thread could not be spawned; the fork \
                 carrier is disabled for this run (nodes that overflow the inline arena \
                 will be reported as skipped rather than captured)"
            );
        }
        Self { shared, handle }
    }

    /// Whether a capture child of THIS process is still running.
    ///
    /// While true the node thread must not touch the state ring at all.
    pub fn child_in_flight(&self) -> bool {
        self.shared
            .mailbox
            .lock()
            .map(|m| m.in_flight.is_some())
            .unwrap_or(false)
    }

    /// Whether the reaper thread is actually running.
    ///
    /// `false` means the spawn failed, and the caller must not fork: nothing would ever
    /// reap the child, so its claim would hold every worker's cadence until the arm
    /// word's stale sweep noticed — which is also this thread's job.
    pub fn is_running(&self) -> bool {
        self.handle.is_some()
    }

    /// Hand a freshly forked child over to the reaper.
    pub fn note_fork(&self, pid: i32, slot: usize, step: u64, node_idx: Vec<u32>) {
        if let Ok(mut m) = self.shared.mailbox.lock() {
            m.in_flight = Some(InFlight {
                reaper: ChildReaper::new(pid),
                slot,
                step,
                node_idx,
            });
        }
    }

    /// Take every reaped child the node thread has not yet acted on.
    ///
    /// Draining and clearing in one lock acquisition is what makes "the mailbox is
    /// non-empty" imply "no child is in flight" for the caller.
    pub fn take_finished(&self) -> Vec<FinishedChild> {
        match self.shared.mailbox.lock() {
            Ok(mut m) => std::mem::take(&mut m.finished),
            Err(_) => Vec::new(),
        }
    }

    /// The last memory sample, or `None` where the platform cannot say.
    pub fn mem_available(&self) -> Option<u64> {
        match self.shared.mem_available.load(Ordering::Relaxed) {
            0 => None,
            n => Some(n),
        }
    }

    /// The memory-gate projection: what a child forked now is assumed to cost.
    pub fn projected_bytes(&self) -> u64 {
        self.shared.anon_rss.load(Ordering::Relaxed)
    }

    /// Stop the thread, JOIN it, and TERMINATE any child still in flight. Idempotent.
    ///
    /// # Why the child must not be left running
    ///
    /// The run loop's final pass is `WNOHANG`: a child still encoding is simply not
    /// reaped, and once the thread is joined NOTHING will ever poll it again. Its
    /// claim slot was taken with THIS process's pid, and a claim is released only on
    /// the reap path — so an orphaned child leaks one of `STATE_CLAIM_SLOTS`
    /// permanently. The stale-claim sweep cannot recover it either: the sweep
    /// reclaims a slot whose OWNER pid is dead, and the owner here is the very
    /// process that is tearing down and carrying on. The arm word's own docs name
    /// where that ends — the table is FINITE, so anything unreclaimable there is a
    /// permanent leak that ends in machine-wide checkpoint death.
    ///
    /// So: stop, join (after which this thread owns the mailbox with no race), then
    /// SIGKILL, reap, and release. The kill is sound by the same argument the
    /// watchdog's is — this is the carrier's own child, and the anchor it was
    /// building is abandoned with the runtime either way.
    pub fn stop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(handle) = self.handle.take() {
            // A join that fails means the thread PANICKED, which is already reported by
            // the default hook; there is nothing this caller can do about it and
            // propagating it would turn a diagnostic thread's death into a runtime
            // teardown failure.
            let _ = handle.join();
        }
        self.shared.terminate_in_flight_child();
    }
}

impl Drop for CaptureReaper {
    fn drop(&mut self) {
        self.stop();
    }
}

impl ReaperShared {
    fn run(self: Arc<Self>) {
        let mut last = std::time::Instant::now();
        let mut pass: u32 = 0;
        while !self.stop.load(Ordering::Acquire) {
            std::thread::sleep(REAPER_POLL_INTERVAL);
            let now = std::time::Instant::now();
            let elapsed_ns = now.duration_since(last).as_nanos() as u64;
            last = now;

            self.poll_child(elapsed_ns);

            pass = pass.wrapping_add(1);
            if pass.is_multiple_of(REAPER_SLOW_PASS_EVERY) {
                self.sample_memory();
                self.sweep_stale_claims();
            }
        }
        // One last pass on the way out, so a child that finished between the final
        // sleep and the stop flag is still reaped and its claim released rather than
        // left for the next run's stale sweep.
        self.poll_child(0);
    }

    /// One `waitpid` pass, plus the watchdog. Returns having either done nothing or
    /// moved a child from `in_flight` to `finished`.
    fn poll_child(&self, elapsed_ns: u64) {
        let Ok(mut m) = self.mailbox.lock() else {
            return;
        };
        let Some(in_flight) = m.in_flight.as_mut() else {
            return;
        };
        let Some(outcome) = in_flight.reaper.poll(&self.crumb, elapsed_ns) else {
            return;
        };
        // The child is reaped, so the breadcrumb is final and can be read without a
        // race. Read BEFORE the slot is released: a peer that sees the slot free may
        // fork immediately, and its `rearm` would zero the page under this read.
        let nodes_accounted = self.crumb.nodes_accounted();
        let phase = self.crumb.phase();
        let watermark = in_flight.reaper.progress_watermark();
        let slot = in_flight.slot;
        let finished = FinishedChild {
            outcome,
            step: in_flight.step,
            node_idx: std::mem::take(&mut in_flight.node_idx),
            nodes_accounted,
            phase,
            watermark,
        };
        m.in_flight = None;
        m.finished.push(finished);
        // Released with THIS process's pid, not the child's: `release` names the party
        // running the teardown, which is what the arm word's sweep tests the liveness
        // of. Naming the (now dead) child would make a live teardown look abandoned.
        // SAFETY: `getpid` takes no arguments and cannot fail.
        let releaser = unsafe { libc::getpid() };
        self.arm.release(slot, releaser);
        // Dropped LAST, after the release, so the lock still orders the mailbox
        // transition against a peer's next claim.
        drop(m);
    }

    /// Teardown: kill and reap a child the joined thread left running, and give its
    /// claim back. Called ONLY from [`CaptureReaper::stop`], after the join — so the
    /// mailbox has no other user and the poll loop cannot race this.
    ///
    /// The wait is BOUNDED. A SIGKILLed child becomes reapable as soon as the kernel
    /// has torn it down, which is immediate in every shape this reaches; the ceiling
    /// is there so a teardown can never become a hang, and the claim is given back on
    /// BOTH paths. Holding it back on the slow path would trade a bounded wait for
    /// the permanent leak this function exists to prevent.
    fn terminate_in_flight_child(&self) {
        /// How long teardown waits for a SIGKILLed child to become reapable.
        const REAP_CEILING: std::time::Duration = std::time::Duration::from_secs(2);
        const REAP_POLL: std::time::Duration = std::time::Duration::from_millis(1);

        let Ok(mut m) = self.mailbox.lock() else {
            return;
        };
        let Some(in_flight) = m.in_flight.take() else {
            return;
        };
        let pid = in_flight.reaper.pid();
        let slot = in_flight.slot;
        tracing::warn!(
            child_pid = pid,
            step = in_flight.step,
            nodes = in_flight.node_idx.len(),
            "a capture child was still encoding when its carrier was torn down \
             — terminating it and releasing its claim. Its nodes have no anchor at this \
             step, and the recording reports them as nodes without one"
        );
        // SAFETY: signalling this carrier's own child.
        unsafe { libc::kill(pid, libc::SIGKILL) };
        let start = std::time::Instant::now();
        let mut reaped = false;
        while start.elapsed() < REAP_CEILING {
            let mut status: libc::c_int = 0;
            // SAFETY: a targeted, non-blocking wait on this carrier's own child.
            let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
            if r == pid {
                reaped = true;
                break;
            }
            if r < 0 {
                match classify_teardown_wait(std::io::Error::last_os_error().raw_os_error()) {
                    TeardownWait::Interrupted => continue,
                    TeardownWait::Gone => {
                        reaped = true;
                        break;
                    }
                    TeardownWait::Unusable => {
                        tracing::error!(
                            child_pid = pid,
                            errno = ?std::io::Error::last_os_error().raw_os_error(),
                            "waitpid on a capture child failed with an errno this \
                             teardown cannot act on — it is neither reaped nor known dead"
                        );
                        break;
                    }
                }
            }
            std::thread::sleep(REAP_POLL);
        }
        if !reaped {
            tracing::error!(
                child_pid = pid,
                "a capture child did not become reapable within the teardown \
                 ceiling after SIGKILL — releasing its claim anyway, because holding it \
                 back would leak one of a finite number of slots for the life of the arm"
            );
        }
        // SAFETY: `getpid` takes no arguments and cannot fail.
        let releaser = unsafe { libc::getpid() };
        self.arm.release(slot, releaser);
        drop(m);
    }

    fn sample_memory(&self) {
        self.mem_available
            .store(sample_mem_available().unwrap_or(0), Ordering::Relaxed);
        self.anon_rss
            .store(sample_anon_rss().unwrap_or(0), Ordering::Relaxed);
    }

    /// THE STALE-CLAIM SWEEP's only production driver.
    ///
    /// Under the shipping `--peer-loss continue` a worker dying between its claim and
    /// its reap leaves the count non-zero forever, and every SURVIVOR skips every
    /// future cadence as `StillEncoding` — a silent, run-long loss of the black box on
    /// precisely the degraded robot an incident recorder exists for. The rule lives in
    /// the arm word and is oracle-tested there; this is the syscall that feeds it.
    fn sweep_stale_claims(&self) {
        let sweep = self.arm.sweep_stale_claims(claimant_is_alive);
        if sweep.is_empty() {
            return;
        }
        // BOTH arms: a published claim whose owner died, and one whose owner died
        // INSIDE the claim body. Counting only the first would under-report exactly the
        // rarer, worse case (a worker killed in a window a handful of instructions
        // wide), which is the one an operator most needs to see repeated.
        self.stale_claims_reclaimed.fetch_add(
            u64::from(sweep.cleared) + u64::from(sweep.interrupted_reclaimed),
            Ordering::Relaxed,
        );
        tracing::warn!(
            cleared = sweep.cleared,
            interrupted_reclaimed = sweep.interrupted_reclaimed,
            freed_bytes = sweep.freed_bytes,
            live = sweep.live,
            "reclaimed checkpoint claims whose owning \
             process is gone. Without this every surviving worker would skip every \
             future cadence as still-encoding, and the run's black box would stop \
             silently"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_memory_gate_declines_only_when_the_floor_would_be_breached() {
        // Hand oracle on both sides of the threshold. The arithmetic is saturating on
        // purpose: an over-committed total must read as ZERO headroom, because the one
        // slip that matters here would admit every fork on exactly the machine that is
        // already out of memory.
        const FLOOR: u64 = 512;
        // Comfortable headroom.
        assert_eq!(
            memory_verdict(100, Some(2_000), FLOOR),
            MemoryVerdict::Proceed
        );
        // Exactly AT the floor is admitted — the floor is what must be LEFT.
        assert_eq!(
            memory_verdict(1_488, Some(2_000), FLOOR),
            MemoryVerdict::Proceed
        );
        // One byte under it is not.
        assert_eq!(
            memory_verdict(1_489, Some(2_000), FLOOR),
            MemoryVerdict::Decline
        );
        // Reservations already exceeding availability saturate to zero headroom rather
        // than wrapping to a very large one.
        assert_eq!(
            memory_verdict(u64::MAX, Some(2_000), FLOOR),
            MemoryVerdict::Decline
        );
        assert_eq!(memory_verdict(0, Some(0), FLOOR), MemoryVerdict::Decline);
    }

    #[test]
    fn an_unknown_memory_reading_proceeds_rather_than_refusing_forever() {
        // Fail-OPEN, and it is a decision: a platform that cannot report available
        // memory would otherwise never anchor at all — a cost-derived refusal by
        // another name, and the exact outcome the watchdog's liveness rule avoids.
        // The verdict stays DISTINCT from `Proceed` so a run whose gate is inert can
        // say so rather than looking like one that passed a check.
        let v = memory_verdict(u64::MAX, None, CAPTURE_MEM_FLOOR_BYTES);
        assert_eq!(v, MemoryVerdict::Unknown);
        assert!(v.may_fork(), "unknown must not become a permanent refusal");
        assert!(MemoryVerdict::Proceed.may_fork());
        assert!(!MemoryVerdict::Decline.may_fork());
    }

    /// The sample is PRIVATE ANONYMOUS bytes, and the
    /// parse that produces it is pinned against hand-written `/proc/self/status`
    /// bodies.
    ///
    /// PURE, so it runs on every platform this repo builds for — including
    /// macOS, which has no `/proc` at all and where the live arm below cannot run.
    /// That matters more than usual here: the defect this pins is a helper whose
    /// NAME says anonymous and whose BODY reads total RSS, which no other test
    /// would notice, because none looks at the number's provenance.
    #[test]
    fn the_rss_parse_reads_private_anonymous_kib_and_nothing_else() {
        // A real body, in the real order, with the near-miss neighbours that make
        // this a DISCRIMINATION test rather than a presence test: `RssFile` and
        // `RssShmem` are exactly the pages that must NOT be counted, and `VmRSS` is
        // their total — the number a total-RSS read would return.
        let body = "\
Name:\tcerulion\n\
VmRSS:\t 1048576 kB\n\
RssAnon:\t   65536 kB\n\
RssFile:\t  131072 kB\n\
RssShmem:\t  851968 kB\n";
        assert_eq!(
            parse_rss_anon_kib(body),
            Some(65_536),
            "the ANON line, not VmRSS and not a sibling"
        );

        // A kernel that does not report it (pre-4.5) — the caller's documented
        // fallback arm.
        assert_eq!(parse_rss_anon_kib("Name:\tx\nVmRSS:\t 100 kB\n"), None);
        assert_eq!(parse_rss_anon_kib(""), None);

        // A UNIT change must DEGRADE rather than be misread: reading kB as B
        // under-reports by 1024x, which at this gate admits a fork nobody can pay
        // for. A prefix-only parse would accept every one of these.
        for hostile in [
            "RssAnon:\t   65536 MB\n",
            "RssAnon:\t   65536\n",
            "RssAnon:\tnotanumber kB\n",
            "RssAnon:\n",
        ] {
            assert_eq!(parse_rss_anon_kib(hostile), None, "{hostile:?}");
        }
        // A LABEL that merely starts the same is not this field.
        assert_eq!(parse_rss_anon_kib("RssAnonymous:\t 1 kB\n"), None);
    }

    /// STRUCTURAL: the Linux arm ASKS for `RssAnon:`
    /// first, and only falls back to `statm`.
    ///
    /// # Why this exists beside the other two arms
    ///
    /// The pure oracle pins what `parse_rss_anon_kib` RETURNS; the live arm pins
    /// what `sample_anon_rss` MEASURES. Neither sees the wiring BETWEEN them, and
    /// the live arm — the only thing that does — is Linux-only, so on the machine
    /// this was authored on a variant that simply skips the `RssAnon` read and falls
    /// straight through to the total-RSS fallback restores the whole defect with
    /// the suite green.
    ///
    /// So this reads the module's own source and requires the ORDER. It is the
    /// `every_hand_written_cdylib_init_applies_the_iox2_log_level` pattern: where a
    /// property cannot be executed on the platform you are on, walk the code for it
    /// rather than leaving it to a CI run nobody reads.
    #[test]
    fn the_linux_sample_asks_for_rss_anon_before_falling_back_to_total_rss() {
        let src = include_str!("fork.rs");
        // Comments are STRIPPED, because this file's own docs discuss `statm` and
        // `RssAnon` at length — a presence test over the raw text would be
        // satisfied by the prose that explains the rule.
        let code: String = src
            .lines()
            .map(|l| match l.find("//") {
                Some(i) => &l[..i],
                None => l,
            })
            .collect::<Vec<_>>()
            .join("\n");
        let start = code
            .find("pub fn sample_anon_rss()")
            .expect("the sampler is in this file");
        let body = &code[start..];
        let end = body.find("\npub fn ").unwrap_or(body.len());
        let body = &body[..end];

        // ANTI-TAUTOLOGY: the slice really is the sampler, and stripping left its
        // code intact — otherwise every assertion below is vacuous.
        assert!(
            body.contains("/proc/self/statm") && body.contains("resident_size"),
            "the extracted body is not `sample_anon_rss`:\n{body}"
        );

        let anon_at = body.find("parse_rss_anon_kib").expect(
            "the Linux arm must READ `RssAnon:` — without this call the sampler is \
             back to total RSS, which prices MAP_SHARED pages a fork never copies",
        );
        let statm_at = body
            .find("/proc/self/statm")
            .expect("the fallback is still there");
        assert!(
            anon_at < statm_at,
            "`RssAnon:` must be asked for BEFORE the total-RSS fallback, or the \
             fallback answers every time and the private-anon read is dead code"
        );
    }

    /// LIVE: a large `MAP_SHARED` mapping must NOT move
    /// the sample.
    ///
    /// The defect in its own terms. `MAP_SHARED` pages are ≈ free at fork
    /// (`vma_needs_copy`), so a robot whose iceoryx pools are resident must not be
    /// priced for them — under the old total-RSS read a 128 MiB pool added 128 MiB
    /// of apparent "state" and could push a healthy robot past the arm-time ceiling.
    ///
    /// LINUX-ONLY, for the reason documented on `sample_anon_rss`: on macOS
    /// `resident_size` counts these pages and the claim is simply FALSE there, so
    /// asserting it would be asserting a bug. **NOT RUN on the authoring machine
    /// (a Mac)** — the pure arm above is what covers this logic everywhere.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_large_shared_mapping_is_not_priced_as_private_state() {
        const SPAN: usize = 128 * 1024 * 1024;
        let before = sample_anon_rss().expect("linux reads its own RSS");

        // SAFETY: an anonymous MAP_SHARED reservation of our own; unmapped below.
        let addr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                SPAN,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        assert_ne!(addr, libc::MAP_FAILED, "the fixture mapping must exist");

        // FAULT IT IN — an unfaulted mapping is resident nowhere and would make this
        // vacuous whatever the helper read.
        // SAFETY: writing inside the mapping we just made.
        unsafe {
            let p = addr as *mut u8;
            let page = libc::sysconf(libc::_SC_PAGESIZE).max(4096) as usize;
            let mut off = 0;
            while off < SPAN {
                p.add(off).write_volatile(1);
                off += page;
            }
        }
        let after = sample_anon_rss().expect("still readable");
        // SAFETY: unmap exactly what we mapped.
        unsafe { libc::munmap(addr, SPAN) };

        // The mapping is resident and SHARED, so it belongs to neither side of this
        // number. A generous slack absorbs the test's own incidental allocations
        // while staying far below the 128 MiB a total-RSS read would have added.
        let moved = after.saturating_sub(before);
        assert!(
            moved < (SPAN as u64) / 4,
            "a faulted-in {SPAN}-byte MAP_SHARED region moved the private-anon sample by \
             {moved} bytes — the sample is counting shared pages, which fork does not copy"
        );
    }

    #[test]
    fn the_two_samples_answer_for_this_process_and_this_host() {
        // Both are platform code with no other coverage, and a sampler that silently
        // returned `None` would leave the gate permanently `Unknown` — i.e. inert —
        // without anything failing. So the arm requires a real answer on the platforms
        // this repo builds for, and requires it to be SANE rather than merely present:
        // a running process has a non-zero RSS, and a host that can run this test has
        // some memory available.
        let rss = sample_anon_rss();
        let avail = sample_mem_available();
        if cfg!(any(target_os = "linux", target_vendor = "apple")) {
            let rss = rss.expect("this platform can read its own RSS");
            assert!(
                rss > 0,
                "a running process has resident memory; 0 means the read silently failed"
            );
            let avail = avail.expect("this platform can read available memory");
            assert!(
                avail > 0,
                "a host running this test has some memory available; 0 would make the \
                 gate read Unknown forever"
            );
        }
    }

    #[test]
    fn the_uncovered_set_starts_where_the_child_stopped_accounting() {
        // THE post-mortem rule. Positions the child got a record into the ring for are
        // NOT reported: a reader holding a part and a SKIP for one node at one step has
        // a contradiction it has no rule for resolving.
        let child = FinishedChild {
            outcome: ChildOutcome::Panicked {
                node_idx: 1,
                field_idx: 0,
            },
            step: 40,
            node_idx: vec![7, 3, 9, 4],
            nodes_accounted: 2,
            phase: ChildPhase::Encoding,
            watermark: 12,
        };
        assert_eq!(
            child.uncovered(),
            &[9, 4],
            "the first two positions are in the ring; the rest are the parent's to report"
        );
        assert_eq!(child.skip_cause(), Some(SkipCause::ChildPanicked));

        // A child that covered everything reports nothing, whatever its exit code.
        let all = FinishedChild {
            nodes_accounted: 4,
            ..child.clone()
        };
        assert!(all.uncovered().is_empty());

        // An accounting word BEYOND the fork set cannot index past the end. Unreachable
        // through the encoder (which bumps once per node), and a slice panic in the
        // parent's post-mortem would take down the graph over a corrupted page.
        let over = FinishedChild {
            nodes_accounted: 99,
            ..child.clone()
        };
        assert!(over.uncovered().is_empty());
    }

    #[test]
    fn every_outcome_maps_to_the_cause_whose_remedy_is_the_right_one() {
        // The capture failure table. Each arm exists because the remedies differ, so a
        // collapse here sends an operator to the wrong half of the system — which is
        // exactly what the recorder-vs-encoder split stops for the `Backpressured` row.
        let base = FinishedChild {
            outcome: ChildOutcome::Completed,
            step: 1,
            node_idx: vec![0],
            nodes_accounted: 0,
            phase: ChildPhase::Encoding,
            watermark: 0,
        };
        let with = |outcome| FinishedChild {
            outcome,
            ..base.clone()
        };

        assert_eq!(with(ChildOutcome::Completed).skip_cause(), None);
        assert_eq!(
            with(ChildOutcome::CaptureFailed).skip_cause(),
            Some(SkipCause::CaptureFailed)
        );
        assert_eq!(
            with(ChildOutcome::Panicked {
                node_idx: 0,
                field_idx: 0
            })
            .skip_cause(),
            Some(SkipCause::ChildPanicked)
        );
        assert_eq!(
            with(ChildOutcome::Signal(libc::SIGSEGV)).skip_cause(),
            Some(SkipCause::ChildCrashed)
        );
        assert_eq!(
            with(ChildOutcome::Backpressured {
                node_idx: 0,
                progress: 0,
                stalled_for_ns: 0
            })
            .skip_cause(),
            Some(SkipCause::RecorderBehind),
            "a dead recorder must not be reported as a broken encoder"
        );
        assert_eq!(
            with(ChildOutcome::Stalled {
                node_idx: 0,
                phase: ChildPhase::Encoding,
                progress: 0,
                stalled_for_ns: 0
            })
            .skip_cause(),
            Some(SkipCause::ChildTimeout)
        );
        assert_eq!(
            with(ChildOutcome::GoneStatusUnknown).skip_cause(),
            None,
            "ECHILD means this process cannot know whether the child finished; minting \
             a cause there would be inventing a verdict"
        );
    }

    /// The teardown wait's errno rule, as a hand oracle.
    ///
    /// Every arm decides something DIFFERENT about a child the reaper thread can no
    /// longer collect, so collapsing any two of them is a real bug: `Interrupted` must
    /// keep waiting (nothing is known), `Gone` must stop and count the child as
    /// accounted for, and anything else must be reported rather than guessed at.
    #[test]
    fn the_teardown_wait_classifies_each_errno_by_what_it_can_conclude() {
        // Hand oracle, one row per conclusion.
        let cases: &[(Option<i32>, TeardownWait)] = &[
            // A signal arrived: nothing is known YET, so the wait must continue.
            (Some(libc::EINTR), TeardownWait::Interrupted),
            // No such child: it IS gone — the outcome the wait wanted, not a failure.
            (Some(libc::ECHILD), TeardownWait::Gone),
            // Neither: report it.
            (Some(libc::EINVAL), TeardownWait::Unusable),
            (Some(libc::EPERM), TeardownWait::Unusable),
            (Some(0), TeardownWait::Unusable),
            // An errno the platform did not give us at all is still not a licence to
            // conclude the child is gone.
            (None, TeardownWait::Unusable),
        ];
        for (errno, want) in cases {
            assert_eq!(classify_teardown_wait(*errno), *want, "errno {errno:?}");
        }
        // The two that are NOT interchangeable, stated as an inequality so a future
        // collapse of the match fails here with the reason attached.
        assert_ne!(
            classify_teardown_wait(Some(libc::EINTR)),
            classify_teardown_wait(Some(libc::ECHILD)),
            "an interrupted wait knows NOTHING; ECHILD knows the child is gone — \
             treating them alike either abandons a live child or fabricates a \
             teardown failure"
        );
    }
}

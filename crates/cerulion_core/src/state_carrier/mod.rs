// SPDX-License-Identifier: AGPL-3.0-only
//! The checkpoint CARRIER — `take_anchor`'s bounded inline attempt,
//! the `fork(2)` child that encodes everything the arena could not hold, and the
//! parent-side machinery that makes a disposable child safe to have.
//!
//! # Why there are two carriers, and why neither reads a clock
//!
//! At every Nth logical step boundary — the one instant when no tick is running, rayon
//! has joined and no output is loaned — the executor walks the process's nodes
//! in declaration order and encodes each one into that boundary's single 64 KiB arena.
//! Two compile-time-and-bytes filters decide the carrier: a node is attempted inline
//! only if `CerulionState::INLINE_SAFE` (the const saying its capture runs only
//! framework-generated code over data that cannot block it) and it stays inline
//! only while its encode FITS. Everything else joins a fork set, and the process calls
//! `fork(2)` **once**, so the MMU-frozen child can encode at leisure.
//!
//! Both carriers call the same generated encoder and emit identical bytes, so which
//! nodes went inline and which forked is invisible in the bag. What varies with state
//! size is anchor SPACING and the first post-fork tick's page-fault cost — never
//! anchor EXISTENCE.
//!
//! # What is in this module
//!
//! | Submodule | Owns |
//! |---|---|
//! | [`anchor`] | `take_anchor`'s bounded inline walk — the two carrier filters, neither a clock |
//! | [`breadcrumb`] | the one page a child may write — the progress word the watchdog reads |
//! | [`watchdog`] | the liveness rule, and the recorder-vs-encoder split |
//! | [`hook`] | the parent-installed, pid-branching panic hook |
//! | [`dontfork`] | the mappings a child must not inherit |
//! | [`child`] | the child's own code, and the ONLY file the source-walk gate polices |
//! | [`reaper`] | the targeted `waitpid`, the outcome vocabulary, and the stale-claim sweep's `kill(pid,0)` |
//! | [`fork`] | the `fork(2)` call site, the child's encode path, and the reaper THREAD |
//!
//! # The measured basis (2026-08-08, Jetson / L4T 5.15.148-tegra)
//!
//! The fork carrier rests on measurements, not on an argument:
//!
//! - **CoW smear**, dense 256 MiB slab: the first tick after `fork` costs **91.6-92.8
//!   ms (11.4-11.6x the 8 ms baseline)**, then p99 returns to baseline — the
//!   front-loading is confirmed on the target. Derived **2.6 us/fault @ 4 KiB**, inside
//!   the expected 1.5-3 us range. Shortening the child's life buys nothing; **cadence
//!   is the lever**.
//! - **`fork` itself**, on the shipping `THP=always` configuration: **0.70-0.85
//!   ms/GiB** (a 2 GiB process forks in 1.4 ms).
//! - **`vma_needs_copy` verified on the real L4T kernel**: 1 GiB of touched
//!   `MAP_SHARED` mappings changes fork cost by ~nothing — iceoryx2 pools are free at
//!   fork.
//! - **Barrier propagation**: a bystander worker's worst rendezvous wait (178.7 ms)
//!   tracks the smearing worker's worst tick (178.2 ms) exactly — the blast radius IS
//!   the peer's worst tick, so the max-vs-sum anti-staggering model holds as
//!   measured.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`. Non-Unix has no `fork` and uses
//! the bounded inline attempt for everything, which is a platform performance
//! statement and never a node refusal.

pub mod anchor;
pub mod breadcrumb;
pub mod child;
pub mod dontfork;
pub mod fork;
pub mod hook;
pub mod reaper;
pub mod watchdog;

pub use anchor::{probe_quiescent, walk_inline, ForkReason, InlineCaptureTarget, InlineWalkStats};
pub use breadcrumb::{ChildBreadcrumb, ChildPhase, MappedBreadcrumb, BREADCRUMB_BYTES};
pub use child::{
    apply_child_discipline, child_main, push_with_backpressure_phase, resolve_fd_ceiling,
    ChildSetup, FdCeiling, FdCeilingSource, KeepFds, NodeCaptureOutcome, NodeEncoder,
};
pub use dontfork::{
    exclude_at_birth, exclude_from_fork, sweep_birth_exclusions_at_arm, ExclusionOutcome,
    ForkExcludedMapping, ForkExclusionSweep,
};
pub use fork::{
    fork_capture, memory_verdict, sample_anon_rss, sample_mem_available, BumpingSink,
    CaptureReaper, FinishedChild, ForkCaptureTarget, ForkOutcome, ForkSetEncoder, MemoryVerdict,
    CAPTURE_MEM_FLOOR_BYTES, REAPER_POLL_INTERVAL,
};
pub use hook::{
    clear_fork_panic_breadcrumb, install_fork_panic_hook, is_capture_child, set_child_stderr_fd,
    set_fork_panic_breadcrumb, CHILD_EXIT_CAPTURE_FAILED, CHILD_EXIT_OK, CHILD_EXIT_PANIC,
};
pub use reaper::{claimant_is_alive, ChildOutcome, ChildReaper};
pub use watchdog::{ProgressReading, StallVerdict, StallWatch, STATE_STALL_TIMEOUT_NS};

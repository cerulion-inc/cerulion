// SPDX-License-Identifier: AGPL-3.0-only
//! Per-topic SHM cache-line "doorbell": a process-shared page the producer RINGS
//! after each publish and the consumer WAITS on, so the live loop wakes the instant
//! a producer rings and not only on the timer. Where the CPU carries a monitor-wait
//! primitive ([`crate::monitor_wait`]) the consumer arms it on the ring counter's
//! line; on macOS the page carries a kernel wake word beside that counter and the
//! consumer blocks on it instead.
//!
//! # Why
//!
//! [`crate::monitor_wait`] parks the inter-step idle in a SHALLOW optimized CPU
//! state (no deep cpuidle C-state, hence no cold-wake), but on its own only the
//! TSC/event-stream timer wakes it — so it re-polls the iceoryx2 listener every
//! `recheck` (~100µs). To wake on DATA the instant it arrives, every
//! data-trigger topic gets one cache-line-aligned page that producer and consumer
//! both map: the producer `ring()`s on each publish, and the consumer waits on that
//! exact line, with `UMONITOR`/`WFE` where the CPU has them and with a kernel block
//! on the page's wake word on macOS, so a ring wakes the park with no timer
//! round-trip.
//!
//! # Mechanism (linux)
//!
//! `shm_open(O_CREAT|O_RDWR, <name>)` + `ftruncate(64)` + `mmap(MAP_SHARED)` → a
//! pointer to one `AtomicU64`. Two processes opening the SAME name map the SAME
//! physical page (POSIX named SHM), so a store in the publisher is observed in
//! the consumer. 64 bytes = one cache line, so the doorbell never false-shares
//! with an adjacent topic. The seq is a RELATIVE counter: the consumer snapshots
//! it on attach and reads deltas, so a stale absolute value from a reused object
//! is harmless.
//!
//! # Naming (namespaced — kills cross-tenant collisions)
//!
//! `<ns>` is a caller-supplied namespace (a graph name or `$USER`) and it
//! partitions the name space, so two tenants never collide on a topic-hash even
//! if their FNV-64 hashes were to clash. The SHAPE is per-OS, because macOS caps
//! a POSIX SHM name at 31 characters: `/cer_db_<ns>_<fnv1a64(topic):016x>` on
//! Linux, where the raw `<ns>` in the `/dev/shm` filename names the deployment,
//! and `/cer_db_<fnv1a64(ns 0x1f topic):016x>` elsewhere, which folds both
//! components into one fixed-width token. The pure derivation lives in
//! `doorbell_shm_name`, hermetically testable on any OS, and both shapes are
//! pinned there on every OS.
//!
//! # Ownership / lifecycle (producer-owned RAII)
//!
//! The PRODUCER calls [`Doorbell::open_owned`]: it `O_CREAT`s the page, OWNS the
//! name (`owns_name == true`), rings it, and on `Drop` `shm_unlink`s the name
//! (best-effort) AND `munmap`s its mapping. A CONSUMER calls
//! [`Doorbell::open_unowned`]: it ALSO opens with `O_CREAT` (so it works even
//! when the producer is not up yet), but does NOT own the name — on `Drop` it
//! `munmap`s only, never unlinking. [`DoorbellRegistry`] holds one unowned
//! doorbell per data-trigger topic.
//!
//! # Crash / restart robustness
//!
//! A producer crash leaves an orphan `cer_db_*` POSIX SHM object (bounded: 64 B
//! per topic; `/dev/shm/cer_db_*` on Linux, and macOS exposes no such path).
//! This is self-healing: the next producer's `O_CREAT` REUSES the
//! orphan, and because the counter is RELATIVE (consumers snapshot on attach and
//! read deltas) a stale absolute value is harmless. If a producer restarts and
//! re-creates a FRESH object (new inode) the consumer's old mapping points at
//! the orphan; [`DoorbellRegistry::reopen`] is the re-map seam for that case,
//! which the live loop does not call. Full restart-race
//! correctness is the runtime's timer-recheck backstop; this
//! module only guarantees it does not make that worse.
//!
//! On macOS a POSIX SHM object has no filesystem path, so an orphan there is
//! neither listable nor removable by hand the way a `/dev/shm` entry is. It is
//! REUSED in place by the next `O_CREAT` under the same name, and its 64 bytes
//! are freed only by an owner's `shm_unlink` or a reboot.
//!
//! A CONSUMER-ONLY topic (an absolute external `source:` / cross-process
//! producer with NO in-process [`Doorbell::open_owned`] producer to
//! `shm_unlink` the object) INTENTIONALLY leaves the same kind of orphan after
//! exit: [`Doorbell::open_unowned`] `O_CREAT`s the `cer_db_*` object but
//! never owns it, so on `Drop` it only `munmap`s. This orphan is BOUNDED (64 B)
//! and SELF-HEALING — the next `O_CREAT` (consumer or producer) reuses it, and
//! the RELATIVE counter makes any stale value harmless — exactly the same
//! property as a producer-crash orphan, so it requires no extra cleanup.
//!
//! # Determinism firewall (NON-NEGOTIABLE)
//!
//! The doorbell seq is a WAKE SIGNAL ONLY — it changes only WHEN the live loop
//! wakes, NEVER what fires. The actual message is still read by
//! `step()`/`drain_level` from the iceoryx2 SHM queue; the doorbell value is
//! never consumed as data. See the firewall comment in
//! [`crate::graph::GraphRuntime`] and [`crate::monitor_wait`].
//!
//! # Targets
//!
//! - `linux`: real POSIX named SHM (`shm_open`/`mmap`/`shm_unlink` via `libc`),
//!   rung by one `Release` atomic increment, heard by the CPU monitor-wait
//!   primitive (`UMONITOR`/`WFE`) armed on the same line where the CPU carries
//!   one, and otherwise by the park's loop-top poll within one recheck.
//! - `macos`: real POSIX named SHM too, plus a KERNEL WAKE WORD. There is no CPU
//!   monitor-wait primitive on this target, so a plain store is heard by nobody
//!   and a parked consumer would only re-poll at its pacing chunk. The page
//!   therefore carries a 4-byte wake epoch beside the ring counter: the producer
//!   bumps it and, only while a consumer holds the page's `parked` gate, issues
//!   `os_sync_wake_by_address_all`; the consumer blocks on it with
//!   `os_sync_wait_on_address_with_timeout`, bounded by the same park slice the
//!   pacing nap would have used. Same mechanism, same shared backend and same
//!   one-latch degradation as the barrier's step-start wake word and the credit
//!   word's producer wake. See [`wake_word_block_primitive_available`].
//! - everything else: a no-op stub over a boxed scratch word so the code
//!   COMPILES. `ring()`/`seq()` are no-ops; `addr()` returns a stable scratch
//!   address (so [`crate::monitor_wait`]'s no-op fallback has a valid pointer to
//!   ignore).
//!
//! # Why the open is FIRST-WINS, and never unlink-first
//!
//! Either side may arrive first and both must end up on the SAME physical page,
//! so neither side may unlink a name it did not just create. That is why this
//! module keeps its own open sequence rather than calling the crate's
//! unlink-first `create_exclusive` mechanic: unlinking would hand a late producer
//! a FRESH object while the consumer kept its mapping of the orphan, and a ring
//! on the new page would be heard by nobody.
//!
//! # Who a ring serves
//!
//! A kernel wake reaches a consumer only while that consumer is blocked, and a
//! consumer can be blocked while a publish happens only when the two are in
//! different processes: inside one process the publish runs in a step the parking
//! thread itself drives. So a producer arms a doorbell only for an output topic a
//! sibling process was planned to read
//! (`cerulion_cli_engine::multiprocess::WorkerPlan::sibling_consumed_topics`), and
//! a consumer opens a page only when one of its declared trigger topics has a
//! writer outside the process ([`crate::graph`]'s `rung_topics`).
//!
//! The one consumer class that loses a wake to this is the `rmw_cerulion` wait
//! set, the only other opener of a consumer-side doorbell in the tree: for a
//! topic no sibling group reads it blocks on a line nothing advances and falls
//! back to its own listener descriptor wait, which is what it does on a release
//! with no doorbell at all. It does not poll: the descriptor wait is a kernel
//! block on an iceoryx2 event. An `rmw_cerulion` PUBLISHER arms its own bell
//! unconditionally at `rmw_create_publisher`, independent of these gates, so an
//! rmw to rmw hop on Linux still wakes on the ring. Tools that read a topic open
//! no doorbell and so lose nothing.

use std::io;
use std::sync::atomic::AtomicU64;
// The park slice the guard's kernel block is bounded by. macOS only, because
// only there does this module own a wait.
#[cfg(target_os = "macos")]
use std::time::Duration;

/// Derive the POSIX SHM object name for `(ns, topic)`.
///
/// Per-OS shape, for the reason the credit word's name carries the same split:
/// - **linux**, [`doorbell_shm_name_verbose`]: `/cer_db_<ns>_<fnv1a64(topic):016x>`.
///   The raw `<ns>` in the `/dev/shm` filename is a deliberate debuggability aid
///   (an `ls /dev/shm` names the deployment).
/// - **non-linux (macos)**, [`doorbell_shm_name_compact`]:
///   `/cer_db_<fnv1a64(ns 0x1f topic):016x>`, 24 chars. macOS caps POSIX SHM
///   names at 31 chars (`PSHMNAMLEN`, the leading slash included), and the
///   verbose form is `25 + len(ns)` chars, so it overruns that cap at a
///   seven-character `$USER`. BOTH components are therefore hashed into one
///   fixed-width token.
///
/// Either shape preserves the tenant partition (a different `ns` gives a
/// different name; the compact form separates `ns` from `topic` with a 0x1F unit
/// separator so `("ab","c")` and `("a","bc")` can never alias). Pure (no I/O), so
/// both are hermetically testable on every OS. Stable across processes, so the
/// producer and consumer of the same `(ns, topic)` derive the same name → map
/// the same page. This is also the registry's dedup key.
///
/// Gated to the targets that MAP a page plus `test`: the no-op stub derives no
/// name at all, so compiling this there is dead code the workspace lint denies.
#[cfg(any(target_os = "linux", target_os = "macos", test))]
fn doorbell_shm_name(ns: &str, topic: &str) -> String {
    #[cfg(target_os = "linux")]
    {
        doorbell_shm_name_verbose(ns, topic)
    }
    #[cfg(not(target_os = "linux"))]
    {
        doorbell_shm_name_compact(ns, topic)
    }
}

/// Test seam: does the named shared memory object for `(ns, topic)` EXIST, without
/// creating one?
///
/// `shm_open` without `O_CREAT`, so a miss leaves the namespace as it found it.
/// The mode argument is passed because the Linux binding declares it as a fixed
/// parameter (`libc`'s Apple declaration is variadic, so omitting it compiles
/// there and fails on Linux); the kernel ignores it without `O_CREAT`.
///
/// This is the oracle for "this graph mapped no doorbell page": a registry object
/// count cannot tell an absent page from a page some other process created, and
/// this can. It answers for ONE name, not for the namespace, so a multi-topic
/// graph needs one call per topic.
///
/// A `false` means ENOENT and nothing else: any other errno PANICS, because an
/// `EACCES` or an `EMFILE` reported as "absent" would make a negative assertion
/// pass on a run that did create the page.
///
/// `false` on a target with no real page, where nothing is ever created.
#[cfg(any(test, feature = "test-helpers"))]
pub fn shm_object_exists_for_test(ns: &str, topic: &str) -> bool {
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    {
        let name = doorbell_shm_name(ns, topic);
        let c = std::ffi::CString::new(name.as_str()).expect("a derived name holds no NUL");
        // SAFETY: `c` is a NUL-terminated C string that outlives the call; the
        // flags carry no `O_CREAT`, so the call creates nothing and the only
        // outcomes are a descriptor or an errno. The mode is required by the
        // Linux binding's fixed arity and ignored by the kernel without `O_CREAT`.
        let fd = unsafe { libc::shm_open(c.as_ptr(), libc::O_RDONLY, 0) };
        if fd >= 0 {
            // SAFETY: `fd` is a descriptor this call just obtained.
            unsafe { libc::close(fd) };
            return true;
        }
        let errno = io::Error::last_os_error().raw_os_error().unwrap_or(0);
        assert_eq!(
            errno,
            libc::ENOENT,
            "the existence probe for {name:?} failed with errno {errno}, which is \
             not ENOENT, so a false here would report a page as absent that this \
             call simply could not open"
        );
        false
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        let _ = (ns, topic);
        false
    }
}

/// The linux name shape: `ns` verbatim (deployment-visible in `/dev/shm`),
/// `topic` hashed. See [`doorbell_shm_name`].
///
/// The hash is the crate's shared FNV-1a-64 rather than a second copy of the
/// constants; it is the same function the inline copy computed, so every name
/// this has ever produced is byte-identical (pinned by
/// `the_linux_name_shape_is_byte_stable`).
#[cfg(any(target_os = "linux", test))]
fn doorbell_shm_name_verbose(ns: &str, topic: &str) -> String {
    let h = crate::shm_map::fnv1a64(topic.as_bytes());
    format!("/cer_db_{ns}_{h:016x}")
}

/// The macOS POSIX SHM name cap, `PSHMNAMLEN`, the leading slash included. Not
/// exposed by `libc`, so it is spelled here once and the compact shape is
/// measured against it rather than against a number typed into a test.
#[cfg(any(target_os = "macos", test))]
const PSHM_NAME_MAX: usize = 31;

/// The macOS-safe name shape: fixed width, under [`PSHM_NAME_MAX`] for ANY
/// `(ns, topic)`. The 0x1F unit separator keeps the split unambiguous. See
/// [`doorbell_shm_name`].
///
/// Gated to the targets that DERIVE a name plus `test`: a target with neither a
/// real page nor a wake word maps nothing and never asks for one, so compiling
/// it there is dead code the workspace lint denies.
#[cfg(any(target_os = "macos", test))]
fn doorbell_shm_name_compact(ns: &str, topic: &str) -> String {
    // hot-path-alloc-ok: name derivation runs only at create/open (cold path).
    let mut key = Vec::with_capacity(ns.len() + 1 + topic.len());
    key.extend_from_slice(ns.as_bytes());
    key.push(0x1f);
    key.extend_from_slice(topic.as_bytes());
    let h = crate::shm_map::fnv1a64(&key);
    let name = format!("/cer_db_{h:016x}");
    debug_assert!(
        name.len() <= PSHM_NAME_MAX,
        "the compact shape is fixed-width and must fit the {PSHM_NAME_MAX}-char \
         cap for every input: {name}"
    );
    name
}

/// Deduplicate `topics` preserving the order given: the registry maps one
/// doorbell per UNIQUE topic, and `primary` must remain the first entry of the
/// list it is handed.
///
/// Pure (no I/O / no SHM), so the order/dedup contract is testable without
/// mapping anything.
fn dedup_topics(topics: &[String]) -> Vec<String> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::with_capacity(topics.len());
    for t in topics {
        if seen.insert(t.as_str()) {
            out.push(t.clone());
        }
    }
    out
}

/// The default SHM doorbell namespace for this process: `$USER`, or the literal
/// `"cerulion"` when `USER` is unset/empty (containers, systemd units). Both a
/// producer ([`Doorbell::open_owned`]) and a consumer ([`DoorbellRegistry`]) in
/// the same process derive the SAME value, so they agree on the object name
/// `doorbell_shm_name` derives and map the same page. `$USER`-based (not
/// graph-name-based) so it is also stable across a single user's processes —
/// forward-compatible with the p4 cross-process doorbell. A different `$USER`
/// gives a different name under either shape, which kills cross-tenant
/// collisions: the Linux shape carries it as a literal segment, the compact
/// shape folds it into the hashed key behind a unit separator.
pub fn default_namespace() -> String {
    namespace_from(std::env::var("USER").ok().as_deref())
}

/// Pure core of [`default_namespace`] (no env read) — `user` when present and
/// non-empty, else the `"cerulion"` fallback. Factored out so the fallback is
/// hermetically testable without mutating the process environment.
fn namespace_from(user: Option<&str>) -> String {
    user.filter(|s| !s.is_empty())
        .map(str::to_string)
        .unwrap_or_else(|| "cerulion".to_string())
}

#[cfg(target_os = "linux")]
mod imp {
    use super::doorbell_shm_name;
    use std::ffi::CString;
    use std::io;
    use std::os::raw::c_void;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// 64 bytes — one cache line — so a topic's doorbell never false-shares with
    /// an adjacent topic's. Only the leading `AtomicU64` is used.
    const DOORBELL_BYTES: usize = 64;

    /// A process-shared SHM doorbell — one cache-line-aligned `AtomicU64`.
    ///
    /// Construct via [`Doorbell::open_owned`] (producer — owns + `shm_unlink`s
    /// the name on drop) or [`Doorbell::open_unowned`] (consumer — maps only).
    #[must_use = "the doorbell is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local for the desired scope"]
    pub struct Doorbell {
        /// Pointer to the mapped `AtomicU64` (offset 0 of the `MAP_SHARED` page).
        ptr: *mut AtomicU64,
        /// The POSIX SHM object name — retained so an OWNED doorbell can
        /// `shm_unlink` it on drop.
        name: CString,
        /// `true` for a producer-created doorbell that owns the name and must
        /// `shm_unlink` it on drop; `false` for a consumer mapping (munmap only).
        owns_name: bool,
    }

    // SAFETY: the doorbell is a single `AtomicU64` in a shared page. All access
    // goes through atomic load/fetch_add, so it is sound to send/share the
    // handle across threads (the OS guarantees the page is coherent across the
    // mapping; atomics give the intra-process ordering). `name`/`owns_name` are
    // plain Send+Sync data.
    unsafe impl Send for Doorbell {}
    unsafe impl Sync for Doorbell {}

    impl Doorbell {
        /// Open (or create) the doorbell for `(ns, topic)` as the OWNER (the
        /// producer). `O_CREAT`s the page and takes ownership of the name, so
        /// drop `shm_unlink`s it.
        pub fn open_owned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, true)
        }

        /// Open (or create) the doorbell for `(ns, topic)` as a CONSUMER. Also
        /// `O_CREAT`s (works before the producer is up), but does NOT own the
        /// name — drop `munmap`s only, never unlinking.
        pub fn open_unowned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, false)
        }

        /// Shared open path. Producer (`owns_name`) and consumer of the same
        /// `(ns, topic)` map the SAME physical page.
        fn open(ns: &str, topic: &str, owns_name: bool) -> io::Result<Self> {
            let name = CString::new(doorbell_shm_name(ns, topic))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            // SAFETY: FFI to POSIX named SHM. `name` is a valid C string; mode
            // 0o600 restricts the object to the owner.
            let fd = unsafe {
                libc::shm_open(
                    name.as_ptr(),
                    libc::O_CREAT | libc::O_RDWR,
                    0o600 as libc::mode_t,
                )
            };
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            // SAFETY: size the (possibly freshly-created) object to one cache
            // line. Idempotent if another process already created+sized it.
            if unsafe { libc::ftruncate(fd, DOORBELL_BYTES as libc::off_t) } < 0 {
                let err = io::Error::last_os_error();
                // SAFETY: fd is the descriptor we just opened.
                unsafe { libc::close(fd) };
                return Err(err);
            }
            // SAFETY: map the shared page read/write. `MAP_SHARED` is what makes
            // a producer store visible to the consumer's mapping.
            let addr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    DOORBELL_BYTES,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    fd,
                    0,
                )
            };
            // The fd can be closed once mapped — the mapping keeps the object
            // alive.
            // SAFETY: fd is the descriptor we just opened (and have now mapped).
            unsafe { libc::close(fd) };
            if addr == libc::MAP_FAILED {
                return Err(io::Error::last_os_error());
            }
            Ok(Self {
                ptr: addr as *mut AtomicU64,
                name,
                owns_name,
            })
        }

        /// Ring the doorbell — a `Release` store (`fetch_add(1)`) that wakes a
        /// consumer parked with `UMONITOR`/`WFE` on this line.
        pub fn ring(&self) {
            // SAFETY: `ptr` is a valid, 8-byte-aligned mapping of an `AtomicU64`.
            let a = unsafe { &*self.ptr };
            a.fetch_add(1, Ordering::Release);
        }

        /// Current ring count (`Acquire` load). The consumer snapshots this
        /// before parking and re-checks it after, to detect a ring. RELATIVE —
        /// compare against a snapshot, never an absolute baseline.
        pub fn seq(&self) -> u64 {
            // SAFETY: as `ring`.
            let a = unsafe { &*self.ptr };
            a.load(Ordering::Acquire)
        }

        /// The watched address for `UMONITOR`/`WFE` (the `AtomicU64`'s location).
        pub fn addr(&self) -> *const AtomicU64 {
            self.ptr as *const AtomicU64
        }
    }

    impl Drop for Doorbell {
        fn drop(&mut self) {
            // SAFETY: unmap the page we mapped in `open`.
            unsafe {
                libc::munmap(self.ptr as *mut c_void, DOORBELL_BYTES);
            }
            if self.owns_name {
                // SAFETY: best-effort unlink of the name THIS doorbell created
                // and owns. Ignoring the result is intentional — a concurrent
                // unlink (or already-gone name) is benign. Existing consumer
                // mappings keep the inode alive until they unmap.
                unsafe {
                    libc::shm_unlink(self.name.as_ptr());
                }
            }
        }
    }
}

#[cfg(target_os = "macos")]
mod imp {
    use super::doorbell_shm_name;
    use super::OpenAttemptKind as OpenAttempt;
    // The os_sync operand size the barrier and the credit word already wait and
    // wake on: 4 bytes. Imported rather than re-declared, because an operand
    // size the kernel refuses latches the whole os_sync family off for the
    // process, so the three wake words share one width or they degrade apart.
    use crate::barrier::OS_SYNC_WORD_SIZE;
    use crate::os_sync::{
        os_sync_backend, os_sync_errno_is_benign, os_sync_errno_is_unrecoverable,
    };
    use std::ffi::CString;
    use std::io;
    use std::os::raw::c_void;
    use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
    use std::time::Duration;

    /// 64 bytes, one cache line, exactly as the Linux page: a topic's doorbell
    /// never false-shares with an adjacent topic's. The leading
    /// [`DoorbellShared`] uses the first 16.
    const DOORBELL_BYTES: usize = 64;

    /// The page size the open path measures an object against, so a test's
    /// fixture is DERIVED from the bound rather than typed beside it.
    #[cfg(all(test, target_os = "macos"))]
    pub(super) fn page_bytes() -> usize {
        DOORBELL_BYTES
    }

    /// The mapped doorbell page's head.
    ///
    /// `seq` is the RELATIVE ring counter, byte-for-byte the contract the Linux
    /// page carries (consumers snapshot it and read deltas). `wake_seq` and
    /// `parked` are the kernel wake word, laid out exactly as the barrier's and
    /// the credit word's: the ringer bumps the epoch and issues the wake syscall
    /// only when somebody holds a parked bit, so a ring with nobody blocked stays
    /// atomics-only.
    #[repr(C)]
    struct DoorbellShared {
        /// Ring count. `Release`-bumped by the producer on every publish;
        /// `Acquire`-read by the consumer's poll-all delta.
        seq: AtomicU64,
        /// The kernel wait/wake compare word. Bumped alongside `seq`, read as the
        /// parker's snapshot, never consumed as data.
        wake_seq: AtomicU32,
        /// How many consumers are kernel-blocked on `wake_seq` right now. Read
        /// by the ringer as a SYSCALL GATE: zero means nobody is waiting and the
        /// ring stays atomics-only.
        ///
        /// A COUNT, not the rank bitmask the barrier and the credit word use. A
        /// doorbell page belongs to ONE topic and the park blocks the ONE
        /// live-loop thread of a consuming process, so there is no rank to index
        /// and no ceiling to run out of; several processes consuming one topic
        /// simply add up, and the wake is wake-all, so the ringer never needs to
        /// know which of them is waiting.
        ///
        /// A consumer SIGKILLed inside its block never runs its guard's `Drop`
        /// and leaks its claim. The cost is latency only: every later ring pays
        /// one wake syscall for a waiter that is not there. Correctness is
        /// untouched, because the claim only ever gates a wake. The barrier's
        /// equivalent stale bit is swept by the supervisor, which knows the dead
        /// rank; there is no rank to name here, so there is no sweep. The claim
        /// clears only when a producer creates a FRESH page, which needs the
        /// previous owner's `Drop` to have unlinked the name first: an orphan
        /// reused after a crash is attached with its stale claim intact, and on
        /// a consumer-only topic, which no owner ever unlinks, the claims of
        /// successive crashed consumers add up for the life of the machine.
        parked: AtomicU32,
    }

    const _: () = assert!(
        core::mem::size_of::<DoorbellShared>() <= DOORBELL_BYTES,
        "DoorbellShared must fit the cache line the page maps"
    );
    const _: () = assert!(
        core::mem::offset_of!(DoorbellShared, seq) == 0,
        "seq must sit at offset 0 so addr() keeps the Linux page's contract"
    );
    const _: () = assert!(
        core::mem::offset_of!(DoorbellShared, wake_seq) == 8,
        "wake_seq must sit at offset 8 (4-byte aligned) as the kernel wait/wake address"
    );
    const _: () = assert!(
        core::mem::offset_of!(DoorbellShared, parked) == 12,
        "parked must sit at offset 12, in the same cache line as the epoch it gates"
    );

    /// A process-shared SHM doorbell with a kernel wake word.
    ///
    /// Construct via [`Doorbell::open_owned`] (producer, unlinks the name on
    /// drop) or [`Doorbell::open_unowned`] (consumer, unmaps only).
    #[must_use = "the doorbell is unmapped (and, if owned, shm_unlink'd) on drop - bind it to a named local for the desired scope"]
    pub struct Doorbell {
        /// Base of the `MAP_SHARED` page, read as a [`DoorbellShared`].
        ptr: *mut DoorbellShared,
        /// The POSIX SHM object name, retained so an OWNED doorbell can unlink it.
        name: CString,
        /// The topic this doorbell belongs to, retained as the SITE KEY every
        /// flood-suppressed event on this page carries. The object name is no
        /// substitute: the macOS shape is a bare hash of namespace and topic, so a
        /// log line naming it tells an operator nothing they can look up.
        topic: String,
        /// `true` for a producer-created doorbell that owns the name.
        owns_name: bool,
    }

    // SAFETY: every access goes through the atomics in the shared page. The OS
    // keeps the page coherent across mappings; the atomics give the intra-process
    // ordering. `name`/`owns_name` are plain Send+Sync data.
    unsafe impl Send for Doorbell {}
    unsafe impl Sync for Doorbell {}

    impl Doorbell {
        /// Open (or create) the doorbell for `(ns, topic)` as the OWNER (the
        /// producer): drop unlinks the name.
        pub fn open_owned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, true)
        }

        /// Open (or create) the doorbell for `(ns, topic)` as a CONSUMER: drop
        /// unmaps only, never unlinking.
        pub fn open_unowned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, false)
        }

        /// Shared open path: FIRST-WINS, never unlink-first.
        ///
        /// Either side may arrive first and both must end up on the SAME physical
        /// page, so this deliberately does not use the crate's unlink-first
        /// `create_exclusive` mechanic: unlinking would hand a late producer a
        /// FRESH object while the consumer kept its mapping of the orphan, and a
        /// ring on the new page would be heard by nobody.
        ///
        /// macOS `EINVAL`s a re-`ftruncate` of a POSIX SHM object (Linux does
        /// not), so the size is asked for on BOTH branches and the rc is read off
        /// the object's size rather than off the call: whichever side gets there
        /// first wins, and the loser's `EINVAL` is the outcome this wants.
        ///
        /// ONE first-wins race is recoverable and is retried rather than reported:
        /// the name is claimed when this call sees `EEXIST` and unlinked by its
        /// owner's `Drop` before the attach, so the attach reports `ENOENT`. It is
        /// ordinary at a graph's startup and otherwise costs the whole run its
        /// data-wake path, because the registry returns the first error for every
        /// topic and a publisher that fails here never arms its bell again.
        ///
        /// An object left UNSIZED by a creator killed between its two syscalls is
        /// not a race at all: whichever side gets there next sizes it in place.
        /// See the sizing step in [`Doorbell::open_once`], which is why no opener
        /// ever has to decide whether to delete a name a live peer may hold.
        fn open(ns: &str, topic: &str, owns_name: bool) -> io::Result<Self> {
            let name = CString::new(doorbell_shm_name(ns, topic))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            super::retry_lost_name_race(|| Self::open_once(&name, topic, owns_name))
        }

        /// One pass of the first-wins open sequence. See [`Doorbell::open`] for the
        /// one recoverable shape it reports.
        fn open_once(name: &CString, topic: &str, owns_name: bool) -> Result<Self, OpenAttempt> {
            let mut created = false;
            // SAFETY: FFI to POSIX named SHM. `name` is a valid C string; mode
            // 0o600 restricts the object to the owner. `mode_t` is `u16` on macOS
            // and integer promotion forbids passing it to a variadic fn, so the
            // mode is widened to `c_uint` (the same widening `shm_map` documents).
            let mut fd = unsafe {
                libc::shm_open(
                    name.as_ptr(),
                    libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                    0o600 as libc::c_uint,
                )
            };
            if fd >= 0 {
                created = true;
            } else {
                let err = io::Error::last_os_error();
                if err.raw_os_error() != Some(libc::EEXIST) {
                    return Err(OpenAttempt::Fatal(err));
                }
                // Somebody else owns the name: attach to THEIR page.
                // SAFETY: FFI open of an existing named SHM object.
                fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDWR, 0) };
                if fd < 0 {
                    let err = io::Error::last_os_error();
                    if err.raw_os_error() == Some(libc::ENOENT) {
                        // The owner unlinked between our EEXIST and this open:
                        // the name is free again, so the next attempt creates it.
                        return Err(OpenAttempt::RaceLostName);
                    }
                    return Err(OpenAttempt::Fatal(err));
                }
            }

            // Size the object, whether this call created it or attached to it.
            //
            // MEASURED on macOS: a POSIX SHM object accepts its FIRST `ftruncate`
            // from ANY descriptor, and refuses every later one with `EINVAL`. So
            // both sides can ask, exactly one wins, and both end on the same page.
            // That is what makes a creator killed between its `shm_open` and its
            // `ftruncate` harmless: nothing else would ever size the object it
            // left, and the next opener sizes it in place rather than having to
            // decide whether to delete a name a live peer may still be holding.
            // A wait-and-see loop was the alternative and it cannot work, because
            // `fstat` on a POSIX SHM object reports every timestamp as zero on this
            // target, so a dead orphan and a descheduled creator are
            // indistinguishable.
            //
            // `EINVAL` here means somebody sized it first, which is the outcome
            // this wants, so the verdict below reads the SIZE rather than the rc.
            // The errno is kept all the same: on the branch that CREATED the object
            // there is no peer, so a failure here is this process's own and its
            // errno is the only thing an operator can act on.
            // SAFETY: FFI ftruncate on the descriptor this call holds.
            let size_rc = unsafe { libc::ftruncate(fd, DOORBELL_BYTES as libc::off_t) };
            let size_errno = if size_rc < 0 {
                io::Error::last_os_error().raw_os_error().unwrap_or(0)
            } else {
                0
            };
            // SAFETY: `st` is zeroed first so a failed `fstat` leaves no
            // uninitialised read; `fstat` fills it on success.
            let mut st: libc::stat = unsafe { std::mem::zeroed() };
            // SAFETY: FFI fstat on the descriptor this call holds.
            if unsafe { libc::fstat(fd, &mut st) } < 0 {
                let err = io::Error::last_os_error();
                // SAFETY: `fd` is the descriptor we opened.
                unsafe { libc::close(fd) };
                // NOT unlinked, even on the branch that created the name: the sizing
                // step has run, so a peer that met `EEXIST` may already have sized and
                // mapped it, and removing the name would send the next opener to a
                // fresh page while that peer rings this one. An object nobody maps is
                // the bounded orphan this module documents as reused in place.
                return Err(OpenAttempt::Fatal(err));
            }
            if st.st_size < DOORBELL_BYTES as libc::off_t {
                // Neither side could size it. Mapping a short object is refused
                // with an errno that names nothing, so the refusal is made here
                // where it can say what it found; the caller warns and the park
                // keeps its recheck timer.
                // SAFETY: `fd` is the descriptor we opened.
                unsafe { libc::close(fd) };
                // Left in place for the reason above: an unsized object is sized by
                // the next opener, so it wedges nothing.
                let why = if created {
                    format!(
                        "this process created it and its own ftruncate failed with errno \
                         {size_errno}"
                    )
                } else {
                    "neither this process nor the peer that created it could size it".to_string()
                };
                return Err(OpenAttempt::Fatal(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!(
                        "the object under doorbell name {name:?} is {} bytes, short of the \
                         one cache line the page needs: {why}",
                        st.st_size
                    ),
                )));
            }

            // SAFETY: map the shared page read/write. `MAP_SHARED` is what makes a
            // producer store visible in the consumer's mapping.
            let addr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    DOORBELL_BYTES,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    fd,
                    0,
                )
            };
            // Capture the mmap errno BEFORE the close: errno after a SUCCESSFUL
            // call is unspecified, so a close landing in between could leave the
            // reported failure reading "Success".
            let mmap_err = if addr == libc::MAP_FAILED {
                Some(io::Error::last_os_error())
            } else {
                None
            };
            // The descriptor can go once mapped; the mapping keeps the object
            // alive.
            // SAFETY: `fd` is the descriptor we opened (and have now mapped).
            unsafe { libc::close(fd) };
            if let Some(err) = mmap_err {
                // Left in place for the reason above: the object is sized by now, so
                // a peer may hold it, and the name is the only thing that keeps the
                // two of us on one page.
                return Err(OpenAttempt::Fatal(err));
            }
            Ok(Self {
                ptr: addr as *mut DoorbellShared,
                name: name.clone(),
                topic: topic.to_string(),
                owns_name,
            })
        }

        /// The mapped page.
        #[inline]
        fn shared(&self) -> &DoorbellShared {
            // SAFETY: `ptr` is a valid, page-aligned `MAP_SHARED` mapping of
            // `DOORBELL_BYTES`, alive for this handle's lifetime.
            unsafe { &*self.ptr }
        }

        /// Ring the doorbell: bump the ring counter, bump the wake epoch, and
        /// kernel-wake a consumer that is blocked on it right now.
        ///
        /// # Cost on the publish path
        ///
        /// One `Release` increment of the ring counter, then one cached backend
        /// read. Where the os_sync family resolved: a second `Release` increment
        /// into the same cache line, a `SeqCst` fence, and a `Relaxed` load of
        /// `parked`. Where it did not, the ring ends at the first increment,
        /// since no peer on the host can block on the word.
        /// On the arm where the wake syscall itself fails
        /// with something unexpected, add an errno read, a mutex acquisition and
        /// one log event; the errnos a healthy run produces return before any of
        /// that. The fence is a store-buffer drain (`dmb ish` on this
        /// target) and it is the
        /// RINGER's half of the store-buffer litmus pair: ringer = {bump the
        /// epoch; load `parked`}, parker = {claim `parked`; re-derive the ring
        /// delta}, each with a `SeqCst` fence between its store and its load.
        /// Without both fences the model permits the both-see-stale outcome,
        /// where the ringer skips the syscall AND the parker blocked on a stale
        /// predicate. The
        /// UNCONDITIONAL epoch bump keeps even that bounded (the parker's kernel
        /// compare value already differs, so the worst case is one park slice),
        /// but the fence makes it correct rather than merely likely. The syscall
        /// itself is gated on `parked`, so a topic nobody is blocked on pays no
        /// syscall at all.
        pub fn ring(&self) {
            let s = self.shared();
            s.seq.fetch_add(1, Ordering::Release);
            if os_sync_backend().is_none() {
                // No resolved backend anywhere in this process, so no peer on this
                // host can block on the word either: the epoch bump, the fence and
                // the gate read would buy nothing. The RING counter still advances,
                // because the consumer's poll-all reads it. Gated on backend
                // PRESENCE only, never on this process's latch or kill switch: those
                // are ours, and a peer's ability to block is its own.
                return;
            }
            s.wake_seq.fetch_add(1, Ordering::Release);
            core::sync::atomic::fence(Ordering::SeqCst);
            if s.parked.load(Ordering::Relaxed) != 0 {
                self.wake_word_wake();
            }
        }

        /// The kernel wake syscall on the wake word: wake ALL blocked consumers
        /// (each re-derives its own ring delta; a spurious wake is a bounded no-op
        /// re-park).
        ///
        /// Gated on backend PRESENCE, deliberately not on this process's own
        /// activity gate. The parker is BY DEFINITION in another process (that is
        /// what the shared page is for) and its ability to be woken is decided by
        /// ITS latch and ITS kill switch, not ours. A wake issued while we are
        /// locally latched costs one syscall; a wake SKIPPED costs the peer a
        /// full park slice on every publish. The same asymmetry the credit word
        /// prices the same way.
        ///
        /// The rc is CLASSIFIED rather than discarded. A host whose wake is
        /// refused while its wait works leaves every consumer blocked for the
        /// whole cap and woken by nothing, and the producer is the only process
        /// that can see the errno, so the ones that say something are logged and
        /// the ones the gate's own race produces are not.
        ///
        /// MEASURED with a probe rather than read off the header, which lists it
        /// for the `_any` variant alone: `ENOENT` from `os_sync_wake_by_address_all`
        /// means no waiter was found, which is the ORDINARY outcome of a
        /// `parked`-gated wake rather than a fault. A claim is set before its
        /// thread reaches the kernel, it is held across a block the parker skips,
        /// and a consumer killed inside its block leaves it set for good, so a
        /// healthy run reaches the kernel with nobody there many times a second.
        /// Logging it would put a line on the publish path of a working run. It is
        /// the ONLY errno treated that way; everything else, the header's transient
        /// `ENOMEM` and `EFAULT` included, reaches the flood latch, because a
        /// transient fault that persists is exactly what an operator needs told.
        ///
        /// An unrecoverable errno does NOT latch the shared os_sync family from
        /// here. The family latch means "this kernel primitive is unusable", and
        /// the WAIT side is where that is established; the wake side's `EINVAL` is
        /// documented as flags, size, address, or kernel state inconsistent at
        /// THIS address, so latching from here would disable the barrier's
        /// step-start wake, the credit park and the park nap over one page.
        #[inline]
        fn wake_word_wake(&self) {
            let Some(backend) = os_sync_backend() else {
                return;
            };
            // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq` in the
            // mapping this handle owns; `os_sync_wake_by_address_all` reads no
            // user memory beyond keying on address + size. `backend.wake` is
            // the dlsym-resolved, signature-checked fn pointer.
            let rc = unsafe {
                (backend.wake)(
                    self.wake_word_addr(),
                    OS_SYNC_WORD_SIZE,
                    libc::OS_SYNC_WAKE_BY_ADDRESS_SHARED,
                )
            };
            super::note_wake_syscall();
            if rc >= 0 {
                // A wake that reached a waiter closes an OPEN regime, so the latch
                // can report what it suppressed instead of staying open for the
                // life of the process. Gated on a relaxed load, never on the lock:
                // this is the arm every publish to a parked topic takes, and taking
                // a process-global mutex here would serialize a producer's
                // publishes across every topic it owns.
                if super::wake_regime_is_open() {
                    super::note_wake_recovered();
                }
                return;
            }
            let errno = io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if crate::os_sync::os_sync_wake_errno_is_expected(errno) {
                super::note_wake_found_nobody();
                return;
            }
            use crate::transport::failure_regime_latch::RegimeDecision;
            match super::note_wake_errno() {
                RegimeDecision::Loud => tracing::warn!(
                    topic = %self.topic,
                    errno,
                    "doorbell wake word os_sync_wake_by_address failed; a consumer blocked on \
                     this topic waits out its park slice instead of being woken by this publish"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    topic = %self.topic,
                    errno,
                    suppressed,
                    "doorbell wake word os_sync_wake_by_address failed; a consumer blocked on \
                     this topic waits out its park slice instead of being woken by this publish"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                    topic = %self.topic,
                    errno,
                    total_failures = total,
                    suppressed,
                    "doorbell wake word os_sync_wake_by_address failed; a consumer blocked on \
                     this topic waits out its park slice instead of being woken by this publish"
                ),
            }
        }

        /// The wake word's kernel wait/wake ADDRESS: the 4-byte `wake_seq`. ONE
        /// recipe shared by the wake side and the wait side.
        #[inline]
        fn wake_word_addr(&self) -> *mut c_void {
            &self.shared().wake_seq as *const AtomicU32 as *mut c_void
        }

        /// Current ring count (`Acquire`). RELATIVE: compare against a snapshot,
        /// never an absolute baseline.
        pub fn seq(&self) -> u64 {
            self.shared().seq.load(Ordering::Acquire)
        }

        /// Test seam: the page's `parked` claim count, read through THIS
        /// mapping. Exists so the RAII contract can be asserted through a
        /// SECOND mapping of the same page (which also proves the gate is
        /// shared state, not a process-local flag). Never read in production.
        #[cfg(test)]
        pub fn parked_for_test(&self) -> u32 {
            self.shared().parked.load(Ordering::Acquire)
        }

        /// The watched address for a CPU monitor-wait primitive (the ring
        /// counter's location). No such primitive exists on this target; the
        /// address is kept valid because the caller's contract says it is.
        pub fn addr(&self) -> *const AtomicU64 {
            &self.shared().seq as *const AtomicU64
        }

        /// The parker's kernel compare value, taken BEFORE its final ring-delta
        /// re-derive: any bump landing after the snapshot fails the compare, so a
        /// ring can never be lost inside the snapshot-to-block window.
        ///
        /// `pub(super)`, reachable only through [`ParkedDoorbellGuard`]: a
        /// snapshot taken before the claim is set does not pair with the
        /// store-buffer fence, and the guard is what makes the order
        /// unwritable-wrong.
        pub(super) fn wake_seq_snapshot(&self) -> u32 {
            self.shared().wake_seq.load(Ordering::Acquire)
        }

        /// Mark this doorbell as kernel-blocked-on RIGHT NOW (the syscall gate the
        /// ringer reads). `pub(super)`: the only caller is
        /// [`ParkedDoorbellGuard`], so the claim is released on every exit path
        /// including an unwind, and an unpaired call cannot be written.
        ///
        /// The `SeqCst` fence after the claim is taken is the parker's half of the
        /// store-buffer litmus pair described on [`Doorbell::ring`].
        pub(super) fn park_enter(&self) {
            self.shared().parked.fetch_add(1, Ordering::AcqRel);
            core::sync::atomic::fence(Ordering::SeqCst);
        }

        /// Clear this process's parked claim (park exit). `pub(super)`: paired
        /// with `park_enter` by [`ParkedDoorbellGuard`] alone.
        ///
        /// SATURATING, not `fetch_sub`. The claim lives in a page every peer
        /// maps, so a decrement below zero wraps to `u32::MAX` and the ringer's
        /// gate then reads "somebody is parked" forever, or reads zero after the
        /// next claim wraps it and a real waiter is never woken. A wrap is
        /// unreachable through the guard, which is why it is also a
        /// `debug_assert`; the saturation is what keeps a corrupted page from
        /// turning into a missed wake on a peer that did nothing wrong.
        pub(super) fn park_exit(&self) {
            let parked = &self.shared().parked;
            let mut cur = parked.load(Ordering::Acquire);
            loop {
                debug_assert!(cur != 0, "park_exit without a matching park_enter");
                let Some(next) = super::parked_decrement(cur) else {
                    // Unreachable through the guard, so reaching it means the
                    // shared count was written by something else. Saying nothing
                    // would leave a peer's missed wake with no evidence at all in
                    // a release build, where the assertion above is compiled out.
                    super::note_parked_gate_underflow();
                    return;
                };
                match parked.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                    Ok(_) => return,
                    Err(seen) => cur = seen,
                }
            }
        }

        /// Kernel-block on the wake word until a ring bumps the epoch past
        /// `snapshot` or `cap` expires.
        ///
        /// [`Parked`](crate::monitor_wait::AddrParkOutcome::Parked) when a real
        /// block ran, so the caller skips its pacing nap and counts a completed
        /// wait; [`RingPending`](crate::monitor_wait::AddrParkOutcome::RingPending)
        /// when the epoch had already moved off `snapshot`, so the caller skips
        /// the block AND the nap and counts NOTHING;
        /// [`Unavailable`](crate::monitor_wait::AddrParkOutcome::Unavailable)
        /// when nothing blocked at all, so the caller naps.
        ///
        /// The three-way vocabulary is the same one the hardware address park
        /// returns, and for the same reason: a `bool` cannot separate "skipped"
        /// from "waited", and the slice telemetry keys on that difference. The
        /// epoch re-read just before the syscall is the analogue of that park's
        /// arm-time recheck; without it, the kernel's own value compare returns a
        /// NON-NEGATIVE rc that is indistinguishable from a wake, and a
        /// zero-length call is counted as a completed park slice.
        ///
        /// No arm sleeps here. An unexpected errno returns `Unavailable` and the
        /// caller paces with the nap it already owns, bounded by the same window,
        /// counted as the nap it is.
        ///
        /// Record-only (Principle 7): this changes only WHEN the park returns. The
        /// caller's own ring-delta re-derive after every wake stays the sole
        /// correctness, and the message itself is still read from the iceoryx2 SHM
        /// queue by the step.
        pub(super) fn park_wait_ring(
            &self,
            snapshot: u32,
            cap: Duration,
        ) -> crate::monitor_wait::AddrParkOutcome {
            use crate::monitor_wait::AddrParkOutcome;
            if cap.is_zero() {
                // Nothing to block for; a zero-timeout kernel call risks EINVAL
                // for no benefit.
                return AddrParkOutcome::Unavailable;
            }
            if !super::doorbell_os_sync_tier_active() {
                return AddrParkOutcome::Unavailable;
            }
            let Some(backend) = os_sync_backend() else {
                return AddrParkOutcome::Unavailable;
            };
            if self.shared().wake_seq.load(Ordering::Acquire) != snapshot {
                // A ring landed between the caller's snapshot and here: the
                // kernel would return at once with a rc no wake can be told from.
                return AddrParkOutcome::RingPending;
            }
            // Relative ns; `cap` is a bounded park slice, far inside u64.
            let timeout_ns = cap.as_nanos() as u64;
            // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq` in this
            // handle's mapping; `backend.wait` is the dlsym-resolved,
            // signature-checked `os_sync_wait_on_address_with_timeout`. SHARED
            // keys on the physical page (cross-process); a value mismatch returns
            // a non-negative rc immediately, reading no memory beyond addr+size.
            let rc = unsafe {
                (backend.wait)(
                    self.wake_word_addr(),
                    u64::from(snapshot),
                    OS_SYNC_WORD_SIZE,
                    libc::OS_SYNC_WAIT_ON_ADDRESS_SHARED,
                    libc::OS_CLOCK_MACH_ABSOLUTE_TIME,
                    timeout_ns,
                )
            };
            if rc >= 0 {
                super::note_wait_recovered();
                return AddrParkOutcome::Parked;
            }
            let errno = io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if os_sync_errno_is_unrecoverable(errno) {
                // One latch for the whole os_sync family: an EINVAL/ENOTSUP
                // means the primitive is unusable, not one call shape.
                if crate::os_sync::latch_os_sync_disabled() {
                    tracing::warn!(
                        topic = %self.topic,
                        errno,
                        "doorbell wake word os_sync_wait_on_address returned an \
                         unrecoverable errno (EINVAL/ENOTSUP); disabling the os_sync tier \
                         process-wide, so the data-wake park falls back to sleep-recheck \
                         pacing"
                    );
                }
                return AddrParkOutcome::Unavailable;
            }
            if os_sync_errno_is_benign(errno) {
                // ETIMEDOUT or EINTR: the slice really ran, so a regime opened by an
                // earlier errno has recovered.
                super::note_wait_recovered();
                return AddrParkOutcome::Parked;
            }
            use crate::transport::failure_regime_latch::RegimeDecision;
            match super::note_wait_errno() {
                RegimeDecision::Loud => tracing::warn!(
                    topic = %self.topic,
                    errno,
                    "doorbell wake word os_sync_wait_on_address failed; this park takes its \
                     bounded recheck nap instead of the kernel block for as long as the errno \
                     persists"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    topic = %self.topic,
                    errno,
                    suppressed,
                    "doorbell wake word os_sync_wait_on_address failed; this park takes its \
                     bounded recheck nap instead of the kernel block for as long as the errno \
                     persists"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                    topic = %self.topic,
                    errno,
                    total_failures = total,
                    suppressed,
                    "doorbell wake word os_sync_wait_on_address failed; this park takes its \
                     bounded recheck nap instead of the kernel block for as long as the errno \
                     persists"
                ),
            }
            AddrParkOutcome::Unavailable
        }
    }

    impl Drop for Doorbell {
        fn drop(&mut self) {
            // SAFETY: unmap the page we mapped in `open`.
            unsafe {
                libc::munmap(self.ptr as *mut c_void, DOORBELL_BYTES);
            }
            if self.owns_name {
                // SAFETY: best-effort unlink of the name THIS doorbell created
                // and owns. Ignoring the result is intentional: a concurrent
                // unlink (or an already-gone name) is benign, and unlink removes
                // only the NAME - existing mappings, including a peer PARKED on
                // the page, stay memory-backed until the last unmap.
                unsafe {
                    libc::shm_unlink(self.name.as_ptr());
                }
            }
        }
    }
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
mod imp {
    use std::io;
    use std::sync::atomic::AtomicU64;

    /// No-op doorbell stub so the code COMPILES on a target with neither a CPU
    /// monitor-wait primitive nor a kernel wake word.
    /// `ring()`/`seq()` are no-ops; `addr()` returns a stable scratch address
    /// (the boxed word) so [`crate::monitor_wait`]'s no-op fallback has a valid
    /// pointer to ignore. Each handle owns a DISTINCT scratch word, so per-topic
    /// addresses still differ (the registry order/dedup invariants hold here).
    #[must_use = "the doorbell is unmapped (and, if owned, shm_unlink'd) on drop — bind it to a named local for the desired scope"]
    pub struct Doorbell {
        scratch: Box<AtomicU64>,
    }

    impl Doorbell {
        /// Producer constructor — a no-op on this target.
        pub fn open_owned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, true)
        }

        /// Consumer constructor — a no-op on this target.
        pub fn open_unowned(ns: &str, topic: &str) -> io::Result<Self> {
            Self::open(ns, topic, false)
        }

        fn open(_ns: &str, _topic: &str, _owns_name: bool) -> io::Result<Self> {
            Ok(Self {
                scratch: Box::new(AtomicU64::new(0)),
            })
        }

        /// No-op on this target.
        pub fn ring(&self) {}

        /// Always `0` on this target.
        pub fn seq(&self) -> u64 {
            0
        }

        /// A stable scratch address (the boxed word) — distinct per handle.
        pub fn addr(&self) -> *const AtomicU64 {
            &*self.scratch as *const AtomicU64
        }
    }
}

pub use imp::Doorbell;

/// Kill switch for the doorbell's macOS kernel wake word: `=0` disables the
/// block and the wake, leaving the park on its bounded sleep-recheck pacing.
///
/// Its OWN switch, not the barrier's, the credit word's or the park nap's, for
/// the reason those three are separate from each other: the env names are
/// consumer-facing surface, and one switch silently disabling an unrelated tier
/// is the misleading-name class this repo rejects. The credit plane's coupling to
/// the barrier's switch shipped once, which is why this one carries an
/// independence test of its own. All of them still ride ONE backend and ONE
/// unrecoverable-errno latch, so the FACT is read from one place while the
/// DECISION stays per consumer.
pub const DOORBELL_OS_SYNC_ENV: &str = "CERULION_DOORBELL_OS_SYNC";

/// Resolve [`DOORBELL_OS_SYNC_ENV`] to "disabled?", once per process.
#[cfg(target_os = "macos")]
fn doorbell_os_sync_kill_switch() -> bool {
    static DISABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *DISABLED.get_or_init(|| {
        resolve_doorbell_os_sync_disabled(std::env::var(DOORBELL_OS_SYNC_ENV).ok().as_deref())
    })
}

/// The pure half of [`doorbell_os_sync_kill_switch`], split out so the loud
/// garbage warn is pinnable without env or `OnceLock` games.
#[cfg(any(target_os = "macos", test))]
fn resolve_doorbell_os_sync_disabled(raw: Option<&str>) -> bool {
    let (disabled, was_garbage) = crate::kill_switch::parse_kill_switch(raw);
    if was_garbage {
        tracing::warn!(
            env = DOORBELL_OS_SYNC_ENV,
            got = %raw.unwrap_or(""),
            "CERULION_DOORBELL_OS_SYNC is set but not `0` (disable) or `1`/unset (enable); \
             keeping the doorbell wake word ON (`0` is the explicit kill switch)"
        );
    }
    disabled
}

/// Is the doorbell's macOS os_sync tier active: the shared BACKEND fact
/// (resolved and not latched) AND this plane's own kill switch?
#[cfg(target_os = "macos")]
fn doorbell_os_sync_tier_active() -> bool {
    wake_word_active_from(
        crate::os_sync::os_sync_backend_usable(),
        doorbell_os_sync_kill_switch(),
    )
}

/// The pure combination rule of [`doorbell_os_sync_tier_active`], over an
/// INJECTED pair, so the truth table is pinned on every OS rather than against
/// whatever this host happens to answer.
///
/// Both sides are load-bearing in opposite directions: dropping the backend
/// term blocks on an unresolved symbol, dropping the kill-switch term ships an
/// operator switch that does nothing.
#[cfg(any(target_os = "macos", test))]
fn wake_word_active_from(backend_usable: bool, killed: bool) -> bool {
    backend_usable && !killed
}

/// Can a consumer KERNEL-BLOCK on a doorbell's wake word on this host?
///
/// `true` only on macOS with the os_sync family resolved, unlatched and not
/// killed by [`DOORBELL_OS_SYNC_ENV`]. On Linux this is a compile-time `false`
/// and the whole data-wake park rung is compiled out: a Linux consumer wakes on
/// the doorbell through the CPU monitor-wait primitive (`UMONITOR`/`WFE`) armed
/// on the same line where the CPU carries one, and otherwise at the park's next
/// recheck poll; the hardware park is what the kernel block must not displace.
/// On every other target there is no primitive at all.
pub fn wake_word_block_primitive_available() -> bool {
    #[cfg(target_os = "macos")]
    {
        doorbell_os_sync_tier_active()
    }
    #[cfg(not(target_os = "macos"))]
    {
        false
    }
}

/// The wake side's flood regime. One latch per condition, which is the latch
/// module's own rule: a failing wait and a failing wake have different remedies,
/// and a shared regime would report whichever failed second at `debug` while the
/// operator's only loud line named the other.
#[cfg(target_os = "macos")]
fn wake_latch(
) -> &'static std::sync::Mutex<crate::transport::failure_regime_latch::FailureRegimeLatch> {
    use crate::transport::failure_regime_latch::FailureRegimeLatch;
    static LATCH: std::sync::Mutex<FailureRegimeLatch> =
        std::sync::Mutex::new(FailureRegimeLatch::new());
    &LATCH
}

/// Close the wake regime after a wake that reached a waiter, and log what it
/// suppressed rather than leaving the regime open for the life of the process.
///
/// Called only when [`wake_regime_is_open`] says there is a regime to close, so
/// the lock stays off the healthy publish path.
#[cfg(target_os = "macos")]
fn note_wake_recovered() {
    use crate::transport::failure_regime_latch::lock_regime_latch;
    let suppressed = lock_regime_latch(wake_latch()).on_success();
    WAKE_REGIME_OPEN.store(false, std::sync::atomic::Ordering::Relaxed);
    if let Some(suppressed) = suppressed {
        tracing::warn!(
            suppressed,
            "doorbell wake word os_sync_wake_by_address is succeeding again"
        );
    }
}

/// The wait side's own regime; see [`wake_latch`] for why the two are not
/// shared.
#[cfg(target_os = "macos")]
fn wait_latch(
) -> &'static std::sync::Mutex<crate::transport::failure_regime_latch::FailureRegimeLatch> {
    use crate::transport::failure_regime_latch::FailureRegimeLatch;
    static LATCH: std::sync::Mutex<FailureRegimeLatch> =
        std::sync::Mutex::new(FailureRegimeLatch::new());
    &LATCH
}

/// Whether the wait regime is open. Read on the park path, which can afford the
/// lock, but kept symmetric with the wake side so both regimes close the same way.
#[cfg(target_os = "macos")]
static WAIT_REGIME_OPEN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Record one failing wake-word WAIT and say how the caller should log it: a
/// loud head per regime, downgraded repeats, and a loud re-announcement each
/// decade of the running total.
///
/// The crate's shared flood latch, never a hand-rolled errno compare, so this
/// site cannot drift from the discipline every other flood site in the crate
/// keeps. The DECISION comes back rather than a formatted line, because the
/// message is the caller's.
#[cfg(target_os = "macos")]
fn note_wait_errno() -> crate::transport::failure_regime_latch::RegimeDecision {
    use crate::transport::failure_regime_latch::lock_regime_latch;
    WAIT_REGIME_OPEN.store(true, std::sync::atomic::Ordering::Relaxed);
    lock_regime_latch(wait_latch()).on_failure()
}

/// Close the wait regime after a wait that really ran, and log what it suppressed.
///
/// Without it the first unexpected errno is loud and every LATER regime resolves to
/// a downgraded repeat, so a park that degrades a second time says nothing at the
/// level an operator reads.
#[cfg(target_os = "macos")]
fn note_wait_recovered() {
    use crate::transport::failure_regime_latch::lock_regime_latch;
    if !WAIT_REGIME_OPEN.load(std::sync::atomic::Ordering::Relaxed) {
        return;
    }
    let suppressed = lock_regime_latch(wait_latch()).on_success();
    WAIT_REGIME_OPEN.store(false, std::sync::atomic::Ordering::Relaxed);
    if let Some(suppressed) = suppressed {
        tracing::warn!(
            suppressed,
            "doorbell wake word os_sync_wait_on_address is succeeding again"
        );
    }
}

/// How one pass of the first-wins open sequence ended. The ONE non-fatal shape is
/// the name race [`retry_lost_name_race`] retries; everything else is the caller's
/// error.
///
/// At module level so the retry loop can be driven over an injected attempt on
/// every OS: the race is another process unlinking a name between this one's
/// `EEXIST` and its attach, which no test can arrange against the real syscalls.
#[cfg(any(target_os = "macos", test))]
enum OpenAttemptKind {
    /// Report this to the caller.
    Fatal(io::Error),
    /// The name was claimed at `shm_open(O_EXCL)` and gone by the attach: its owner
    /// unlinked in between, so the name is free to create.
    RaceLostName,
}

/// Two retries for the name race, plus the attempt that meets it: drive `attempt`
/// until it succeeds, reports a fatal error, or runs out of tries.
///
/// Bounded so a pathological peer reports rather than spins; the only non-fatal
/// shape is the lost name race, so the bound is also the count the exhaustion
/// message carries.
///
/// Takes the attempt as a closure so both arms are reachable without a peer: the
/// race is driven by another process unlinking a name between this one's `EEXIST`
/// and its attach, which no test can arrange, and the exhaustion arm needs it to
/// happen three times running.
#[cfg(any(target_os = "macos", test))]
fn retry_lost_name_race<T>(
    mut attempt: impl FnMut() -> Result<T, OpenAttemptKind>,
) -> io::Result<T> {
    const OPEN_ATTEMPTS: u32 = 3;
    for _ in 0..OPEN_ATTEMPTS {
        match attempt() {
            Ok(v) => return Ok(v),
            Err(OpenAttemptKind::Fatal(e)) => return Err(e),
            Err(OpenAttemptKind::RaceLostName) => continue,
        }
    }
    Err(io::Error::new(
        io::ErrorKind::WouldBlock,
        format!(
            "doorbell open lost the first-wins name race on all {OPEN_ATTEMPTS} attempts, so a \
             peer is creating and unlinking this name in a loop"
        ),
    ))
}

/// What a park exit does with the claim count it observed: one fewer, or `None`
/// when the count was already zero and a decrement would wrap it.
///
/// Pure and split out because the arm it decides is unreachable through the guard,
/// so the `debug_assert` beside it can never fire in a test build, and the warn it
/// guards is the only evidence a release build would leave.
#[cfg(any(target_os = "macos", test))]
fn parked_decrement(cur: u32) -> Option<u32> {
    cur.checked_sub(1)
}

/// Say ONCE that a doorbell page's `parked` count was decremented below zero.
///
/// Unreachable through [`ParkedDoorbellGuard`], so reaching it means the count in
/// a page every peer maps was written by something else. The consequence lands on
/// a PEER (its ringer's gate reads the wrong value and a real waiter is never
/// woken), so the process that sees it is the only one that can report it.
#[cfg(target_os = "macos")]
fn note_parked_gate_underflow() {
    static ANNOUNCED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !ANNOUNCED.swap(true, std::sync::atomic::Ordering::Relaxed) {
        tracing::warn!(
            "a doorbell page's parked count was already zero at park exit: the count is shared \
             with every peer that maps the topic, so a ringer may now read it wrong and leave a \
             blocked consumer unwoken for its whole park slice"
        );
    }
}

/// Whether the wake regime is open, as a relaxed load the publish path can afford.
///
/// The recovery call takes the flood latch's mutex, and a producer publishing to a
/// parked topic reaches it on EVERY publish, so the lock cannot be the thing that
/// decides there is nothing to recover from. A stale read costs at most one late
/// recovery line.
#[cfg(target_os = "macos")]
static WAKE_REGIME_OPEN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Is a wake regime open? Read on the publish path, so it is one relaxed load.
#[cfg(target_os = "macos")]
fn wake_regime_is_open() -> bool {
    WAKE_REGIME_OPEN.load(std::sync::atomic::Ordering::Relaxed)
}

/// Record one failing wake-word WAKE, in the wake side's own regime.
#[cfg(target_os = "macos")]
fn note_wake_errno() -> crate::transport::failure_regime_latch::RegimeDecision {
    use crate::transport::failure_regime_latch::lock_regime_latch;
    WAKE_REGIME_OPEN.store(true, std::sync::atomic::Ordering::Relaxed);
    lock_regime_latch(wake_latch()).on_failure()
}

/// The doorbell page size, for a test that builds a fixture object at and below
/// the bound the open path enforces.
#[cfg(all(test, target_os = "macos"))]
fn doorbell_page_bytes_for_test() -> usize {
    imp::page_bytes()
}

/// Wake syscalls this process issued on a doorbell wake word. Its pair,
/// [`WAKES_WITH_NO_WAITER`], counts how many found nobody; both are read together
/// through [`wake_syscall_counts`].
///
/// Shipped, not test-only. A consumer killed inside its block leaves its `parked`
/// claim set for good, and on a consumer-only topic the claims of successive
/// crashed consumers add up, so every later publish pays a syscall that returns at
/// once. That is accepted and bounded, but it is a permanent publish-path cost with
/// no log line by design, so the numbers are the only way an operator can answer
/// "my macOS publish path got slower after a worker died" without a rebuild. A
/// relaxed counter on an arm that already paid for a syscall.
#[cfg(target_os = "macos")]
static WAKE_SYSCALLS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Wake syscalls that found no waiter. See [`WAKE_SYSCALLS`].
#[cfg(target_os = "macos")]
static WAKES_WITH_NO_WAITER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Record one issued wake syscall.
#[cfg(target_os = "macos")]
fn note_wake_syscall() {
    WAKE_SYSCALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

/// Record one wake syscall that found no waiter.
#[cfg(target_os = "macos")]
fn note_wake_found_nobody() {
    WAKES_WITH_NO_WAITER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

/// Wake syscalls issued so far in this process, and how many found no waiter. A
/// second number far behind the first is a leaked claim on some topic.
#[cfg(target_os = "macos")]
pub fn wake_syscall_counts() -> (u64, u64) {
    (
        WAKE_SYSCALLS.load(std::sync::atomic::Ordering::Relaxed),
        WAKES_WITH_NO_WAITER.load(std::sync::atomic::Ordering::Relaxed),
    )
}

/// Say ONCE that the data-wake park stopped kernel-blocking for a reason this
/// plane did not choose.
///
/// The shared os_sync family latches off for the whole process the first time
/// ANY of its consumers meets an unrecoverable errno, so the barrier's or the
/// credit word's failure silences this one, and that peer's warn names its own
/// plane. Without this line the run's only statement about the wake word stays
/// the one it printed at startup, while every park paces. An operator who set
/// the kill switch asked for exactly this and is not told at all.
///
/// Called by the PARK, from the branch where its own gate declined on the
/// wake-word term with everything else in place: the gate is re-evaluated every
/// iteration, so by the time a wait could report it the wait is no longer
/// reached.
#[cfg(target_os = "macos")]
pub(crate) fn note_tier_inactive() {
    if doorbell_os_sync_kill_switch() {
        return;
    }
    static ANNOUNCED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !ANNOUNCED.swap(true, std::sync::atomic::Ordering::Relaxed) {
        tracing::warn!(
            env = DOORBELL_OS_SYNC_ENV,
            "the doorbell data-wake block is OFF although it was not disabled here: the shared \
             os_sync family is unresolved or has latched after an unrecoverable errno, so this \
             park is pacing on its recheck timer for the rest of the run"
        );
    }
}

/// RAII claim on a doorbell's `parked` gate: set on `enter`, cleared on every exit
/// path THIS PROCESS runs, an unwind included, so no ringer pays for a waiter this
/// process stopped being of its own accord. A process killed without running `Drop`
/// leaves its claim set; the `parked` field states what that costs and why
/// correctness is untouched.
///
/// It is also the only route to the block from OUTSIDE this module: the claim,
/// the epoch read and the wait are all `pub(super)` on the handle, so a caller
/// elsewhere in the crate cannot block without the claim, and cannot hand the
/// block a compare value the claim did not precede. A caller that blocked without
/// the claim would never be woken by a ringer (the gate reads zero, so no wake
/// syscall is issued) and would wait out its whole cap in silence. Inside this
/// module the order is a convention the guard makes obvious, not one the compiler
/// checks.
#[cfg(target_os = "macos")]
#[must_use = "the parked claim is released on drop - bind the guard for the block's lifetime"]
pub struct ParkedDoorbellGuard<'a> {
    bell: &'a Doorbell,
    /// The kernel compare value, read AFTER the claim and its fence. Held here
    /// rather than passed in: a caller handing over a stale value gets an
    /// immediate kernel value-mismatch, which returns a non-negative rc that
    /// reads as a completed block, so the park would skip its nap and spin the
    /// window at full CPU.
    snapshot: u32,
}

#[cfg(target_os = "macos")]
impl<'a> ParkedDoorbellGuard<'a> {
    /// Claim the gate for the duration of one kernel block, and read the compare
    /// value the block will use.
    pub fn enter(bell: &'a Doorbell) -> Self {
        bell.park_enter();
        let snapshot = bell.wake_seq_snapshot();
        Self { bell, snapshot }
    }

    /// The compare value this guard read at [`enter`](Self::enter). Read-only:
    /// the block uses the stored value, never one a caller hands over.
    pub fn snapshot(&self) -> u32 {
        self.snapshot
    }

    /// Kernel-block on the wake word until a ring bumps the epoch past the value
    /// read at [`enter`](Self::enter), or `cap` expires. See
    /// `Doorbell::park_wait_ring` (private) for the three outcomes.
    pub fn wait(&self, cap: Duration) -> crate::monitor_wait::AddrParkOutcome {
        self.bell.park_wait_ring(self.snapshot, cap)
    }
}

#[cfg(target_os = "macos")]
impl Drop for ParkedDoorbellGuard<'_> {
    fn drop(&mut self) {
        self.bell.park_exit();
    }
}

/// A registry of unowned [`Doorbell`]s over a set of (consumer-side)
/// data-trigger topics.
///
/// Opens and dedups exactly one doorbell per UNIQUE topic (the order it is
/// handed, preserved), RAII-owning them all (every one unowned: production producers
/// own + `shm_unlink` their own lines via [`Doorbell::open_owned`]). The runtime
/// uses [`snapshot_all`](DoorbellRegistry::snapshot_all) for the record-only
/// poll-all, and [`addr`](DoorbellRegistry::addr) with
/// [`slot_of`](DoorbellRegistry::slot_of) and [`bell`](DoorbellRegistry::bell) on
/// the one topic it chose to arm. [`reopen`](DoorbellRegistry::reopen) is the
/// re-map seam for a producer that re-created its object; see the
/// `doorbell_registry` field doc on `GraphRuntime` for why the live loop does
/// not call it.
#[must_use = "the registry unmaps all its doorbells on drop — bind it to a named local for the desired scope"]
pub struct DoorbellRegistry {
    /// Namespace passed at construction — retained so [`reopen`] can re-derive
    /// SHM names.
    ///
    /// [`reopen`]: DoorbellRegistry::reopen
    ns: String,
    /// Deduped topics in the order handed to `open`; parallel to `doorbells`.
    topics: Vec<String>,
    /// One unowned doorbell per topic, parallel to `topics`.
    doorbells: Vec<Doorbell>,
}

impl DoorbellRegistry {
    /// Open one unowned doorbell per UNIQUE topic in `topics` (the given
    /// order preserved) under namespace `ns`.
    ///
    /// Returns the first open error on a target with a real SHM page (linux,
    /// macos); on the no-op stub every open succeeds.
    pub fn open(ns: &str, topics: &[String]) -> io::Result<Self> {
        let topics = dedup_topics(topics);
        let mut doorbells = Vec::with_capacity(topics.len());
        for topic in &topics {
            // The registry is all or nothing, so the first failure costs the run its
            // whole data-wake path. The topic goes into the error, because the
            // caller's warn has the graph and nothing else, and on macOS the object
            // cannot be listed by hand to find out which one it was.
            doorbells.push(Doorbell::open_unowned(ns, topic).map_err(|e| {
                io::Error::new(
                    e.kind(),
                    format!("doorbell open failed for topic {topic:?}: {e}"),
                )
            })?);
        }
        Ok(Self {
            ns: ns.to_string(),
            topics,
            doorbells,
        })
    }

    /// Number of (deduped) doorbells held.
    pub fn len(&self) -> usize {
        self.doorbells.len()
    }

    /// `true` when the registry holds no doorbells.
    pub fn is_empty(&self) -> bool {
        self.doorbells.is_empty()
    }

    /// The deduped topics, in the order handed to [`open`](Self::open) (parallel
    /// to the doorbells).
    pub fn topics(&self) -> &[String] {
        &self.topics
    }

    /// Snapshot the current ring count of every doorbell, in topic order — the
    /// record-only poll-all the runtime reads after a monitor-wait wake.
    pub fn snapshot_all(&self) -> Vec<u64> {
        self.doorbells.iter().map(Doorbell::seq).collect()
    }

    /// `true` iff ANY doorbell's ring count differs from the parallel `baseline`
    /// snapshot (a prior [`snapshot_all`](Self::snapshot_all)). The
    /// non-allocating poll-all the runtime's park loop uses to detect a ring on
    /// a NON-primary topic, and the re-derive the macOS wake-word block runs
    /// between its claim and its kernel wait. Where the CPU carries a monitor-wait
    /// primitive the primary line is additionally hardware-armed; every other
    /// topic is caught only by this ≤100µs-recheck delta scan.
    /// RELATIVE counters ⇒ only the delta from
    /// `baseline` matters, so a stale absolute value is harmless. A `baseline`
    /// shorter than the doorbell set conservatively reports any extra as advanced.
    pub fn any_advanced_since(&self, baseline: &[u64]) -> bool {
        // The baseline must come from THIS registry's `snapshot_all()`, so it can
        // never be LONGER than the doorbell set — a longer baseline is an
        // unambiguous wrong-registry bug. A debug/test-only pin; a SHORTER baseline
        // stays a documented, supported input (the conservative `is_none_or` branch
        // below reports any extra topic as advanced — exercised by the any-OS unit
        // test, the only guard against a no-op edit on a no-primitive machine), so `<=` not `==`.
        debug_assert!(
            baseline.len() <= self.doorbells.len(),
            "any_advanced_since baseline must be a snapshot_all() of THIS registry \
             (got len {} > {} doorbells)",
            baseline.len(),
            self.doorbells.len()
        );
        self.doorbells
            .iter()
            .enumerate()
            .any(|(i, db)| baseline.get(i).is_none_or(|&b| db.seq() != b))
    }

    /// The FIRST topic as handed to [`open`](Self::open), and its doorbell line.
    /// `None` when the registry is empty.
    ///
    /// The POSITIONAL accessor. The runtime arms the hardware monitor through
    /// [`addr`](Self::addr) on the topic it chose, not through this.
    ///
    /// The registry preserves the order it is given and knows nothing about who
    /// publishes a topic; the graph runtime hands it an order whose first entry
    /// is a topic a publisher outside that process can write
    /// (`graph::runtime::rung_topics`), so "first" here is not "first declared in
    /// the graph file".
    #[deprecated(
        since = "1.0.0",
        note = "take the line by topic: addr, bell and slot_of. A position taken here and an index taken from a baseline can name different topics."
    )]
    pub fn primary_addr(&self) -> Option<*const AtomicU64> {
        self.doorbells.first().map(Doorbell::addr)
    }

    /// The FIRST topic as handed to [`open`](Self::open) and its DOORBELL. `None`
    /// when the registry is empty.
    ///
    /// The POSITIONAL accessor. The macOS park takes its handle through
    /// [`bell`](Self::bell) on the topic it chose, not through this.
    ///
    /// The handle it hands back can `ring()`, and every handle in this registry is
    /// a CONSUMER's: a ring from here would be a data wake with no data behind it,
    /// which the park would attribute to the doorbell and act on. The park reads
    /// it to block, never to ring.
    #[deprecated(
        since = "1.0.0",
        note = "take the handle by topic: bell. A position taken here and an index taken from a baseline can name different topics."
    )]
    pub fn primary(&self) -> Option<&Doorbell> {
        self.doorbells.first()
    }

    /// The FIRST topic NAME as handed to [`open`](Self::open). `None` when the
    /// registry is empty.
    ///
    /// The POSITIONAL accessor. The `run_live` wait-policy line reports the topic
    /// the runtime chose from its own record, not from here.
    #[deprecated(
        since = "1.0.0",
        note = "a caller that armed a line already knows its topic; slot_of gives that topic index in a baseline."
    )]
    pub fn primary_topic(&self) -> Option<&str> {
        self.topics.first().map(String::as_str)
    }

    /// The doorbell line for a specific `topic`, or `None` if not registered.
    pub fn addr(&self, topic: &str) -> Option<*const AtomicU64> {
        self.index_of(topic).map(|i| self.doorbells[i].addr())
    }

    /// Re-map the doorbell for `topic` (a fresh `open_unowned`), dropping the old
    /// mapping. The seam the runtime calls on a producer-reconnect
    /// `LivelinessEvent`, when the producer may have re-created a fresh SHM
    /// object (new inode) the prior mapping no longer points at.
    ///
    /// Errors if `topic` is not registered, or if the re-open fails.
    pub fn reopen(&mut self, topic: &str) -> io::Result<()> {
        match self.index_of(topic) {
            Some(i) => {
                // Open the fresh mapping BEFORE dropping the old one so a failed
                // re-open leaves the prior (still-valid) mapping in place.
                let fresh = Doorbell::open_unowned(&self.ns, topic)?;
                self.doorbells[i] = fresh;
                Ok(())
            }
            None => Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("doorbell registry has no topic {topic:?}"),
            )),
        }
    }

    /// The doorbell handle for `topic`, or `None` if not registered.
    ///
    /// The macOS wake-word park takes its handle here rather than positionally,
    /// so the handle it blocks on and the address the hardware plane arms come
    /// from the same topic.
    pub fn bell(&self, topic: &str) -> Option<&Doorbell> {
        self.index_of(topic).map(|i| &self.doorbells[i])
    }

    /// The slot `topic` occupies in this registry's order: the index of its ring
    /// counter in [`snapshot_all`](Self::snapshot_all), and so in any baseline
    /// taken from it.
    ///
    /// A caller arming a kernel wake takes the address and this slot from the one
    /// topic, which is what keeps the armed line and the value it is compared
    /// against from being derived two ways.
    pub fn slot_of(&self, topic: &str) -> Option<usize> {
        self.index_of(topic)
    }

    /// Index of `topic` in the deduped topic list, or `None`.
    fn index_of(&self, topic: &str) -> Option<usize> {
        self.topics.iter().position(|t| t.as_str() == topic)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unique namespace per test run (pid-scoped) so concurrent test binaries
    /// and re-runs never collide on a POSIX SHM object.
    fn test_ns(tag: &str) -> String {
        format!("doorbell_{}_{tag}", std::process::id())
    }

    /// On a target with a real SHM page, `shm_unlink` every `(ns, topic)` an
    /// unowned-registry test created (unowned doorbells never unlink
    /// themselves). No-op where the doorbell is the stub.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    fn cleanup(ns: &str, topics: &[&str]) {
        for topic in topics {
            if let Ok(name) = std::ffi::CString::new(doorbell_shm_name(ns, topic)) {
                // SAFETY: best-effort unlink of a name this test derived.
                unsafe {
                    libc::shm_unlink(name.as_ptr());
                }
            }
        }
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    fn cleanup(_ns: &str, _topics: &[&str]) {}

    // ---- PURE name derivation (all OS) ----

    #[test]
    fn name_is_deterministic_and_carries_the_family_prefix() {
        let n1 = doorbell_shm_name("g", "a");
        let n2 = doorbell_shm_name("g", "a");
        assert_eq!(n1, n2, "same (ns,topic) → identical name (the dedup key)");
        assert!(
            n1.starts_with("/cer_db_"),
            "every shape carries the `/cer_db_` family prefix: {n1}"
        );
        #[cfg(target_os = "linux")]
        assert!(
            n1.starts_with("/cer_db_g_"),
            "on linux the ns appears as a literal prefix: {n1}"
        );
    }

    /// The LINUX name shape, against a hand oracle, and the reason it is pinned
    /// on every OS rather than behind a `cfg`: the derivation moved from an
    /// inline copy of the FNV constants to the crate's shared `fnv1a64`, and
    /// these three literals are the evidence that the swap changed no name any
    /// deployment has ever mapped.
    #[test]
    fn the_linux_name_shape_is_byte_stable() {
        // FNV-1a-64 hand-computed oracle (independent Python reference):
        //   fnv1a64("a")     = 0xaf63dc4c8601ec8c
        //   fnv1a64("b")     = 0xaf63df4c8601f1a5
        //   fnv1a64("topic") = 0x520c8b7d6934ac64
        assert_eq!(
            doorbell_shm_name_verbose("g", "a"),
            "/cer_db_g_af63dc4c8601ec8c"
        );
        assert_eq!(
            doorbell_shm_name_verbose("g", "b"),
            "/cer_db_g_af63df4c8601f1a5"
        );
        assert_eq!(
            doorbell_shm_name_verbose("g", "topic"),
            "/cer_db_g_520c8b7d6934ac64"
        );
    }

    /// The COMPACT shape, against a hand oracle. The BYTES are what has to stay
    /// stable, because a producer and a consumer built at different times must
    /// derive the same name and map the same page; a length assertion alone is
    /// a property `format!` guarantees for every input and would survive any
    /// change to the hashed key.
    ///
    /// The three literals come from the FNV-1a-64 values pinned independently in
    /// `shm_map`'s own oracle over the same unit-separated keys.
    #[test]
    fn the_compact_name_shape_is_byte_stable() {
        // Hand oracle, an independent Python FNV-1a-64 over the unit-separated
        // key. The first two are the values `shm_map`'s own oracle pins over the
        // same keys; the third is hand-computed here, since the bare separator is
        // not among them:
        //   fnv1a64(b"g\x1fa") = 0xd4c07218fa8dad0e
        //   fnv1a64(b"g\x1fb") = 0xd4c07118fa8dab5b
        //   fnv1a64(b"\x1f")   = 0xaf63d24c8601db8e
        assert_eq!(
            doorbell_shm_name_compact("g", "a"),
            "/cer_db_d4c07218fa8dad0e"
        );
        assert_eq!(
            doorbell_shm_name_compact("g", "b"),
            "/cer_db_d4c07118fa8dab5b"
        );
        assert_eq!(
            doorbell_shm_name_compact("", ""),
            "/cer_db_af63d24c8601db8e"
        );
    }

    /// The COMPACT shape fits the macOS name cap for ANY `(ns, topic)`. The cap
    /// is the reason the shape exists, so it is asserted against the constant
    /// the module states rather than a number typed here, and on the longest
    /// input a deployment can produce rather than a short one.
    #[test]
    fn the_compact_name_fits_the_macos_name_cap() {
        for (ns, topic) in [
            ("g", "a"),
            ("", ""),
            (
                "cerdep_a_very_long_graph_name_with_a_nonce_0123456789abcdef",
                "/an/absolute/topic/name/that/is/also/quite/long/indeed",
            ),
        ] {
            let n = doorbell_shm_name_compact(ns, topic);
            assert!(
                n.len() <= PSHM_NAME_MAX,
                "must fit the {PSHM_NAME_MAX}-char cap: {n} is {} chars",
                n.len()
            );
            assert!(n.starts_with("/cer_db_"), "family prefix: {n}");
        }
    }

    /// Both sides of the cap the compact shape exists to clear, with the boundary
    /// DERIVED from the constant rather than from the number in the module's prose,
    /// so the arithmetic that justifies the shape cannot drift from it.
    #[test]
    fn the_verbose_shape_overruns_the_macos_cap_one_character_past_the_longest_fit() {
        // "/cer_db_" (8) + ns + "_" (1) + 16 hex = 25 + len(ns).
        const FIXED: usize = 25;
        let longest_fit = "u".repeat(PSHM_NAME_MAX - FIXED);
        let one_more = "u".repeat(PSHM_NAME_MAX - FIXED + 1);
        assert_eq!(
            doorbell_shm_name_verbose(&longest_fit, "t").len(),
            PSHM_NAME_MAX,
            "the longest fitting namespace lands exactly on the cap"
        );
        assert!(
            doorbell_shm_name_verbose(&one_more, "t").len() > PSHM_NAME_MAX,
            "one character more overruns it, which is the whole reason the compact \
             shape hashes both components"
        );
        assert!(
            doorbell_shm_name_compact(&one_more, "t").len() <= PSHM_NAME_MAX,
            "and the compact shape clears the cap for that same namespace"
        );
    }

    /// The 0x1F unit separator is what makes the two-component hash
    /// unambiguous: without it `("ab","c")` and `("a","bc")` would hash the same
    /// key and two different tenants would share one page. Pinned as the two
    /// hand-oracle values rather than as an inequality, so a change that made
    /// both names wrong in the same way could not pass.
    #[test]
    fn the_compact_name_cannot_alias_across_the_ns_topic_split() {
        //   fnv1a64(b"ab\x1fc") = 0xfd86a083ef3eee44
        //   fnv1a64(b"a\x1fbc") = 0xe8bd15823051e636
        assert_eq!(
            doorbell_shm_name_compact("ab", "c"),
            "/cer_db_fd86a083ef3eee44"
        );
        assert_eq!(
            doorbell_shm_name_compact("a", "bc"),
            "/cer_db_e8bd15823051e636"
        );
    }

    #[test]
    fn name_different_topic_different_name() {
        assert_ne!(doorbell_shm_name("g", "a"), doorbell_shm_name("g", "b"));
    }

    #[test]
    fn name_different_ns_different_name() {
        // The whole point of namespacing: same topic, different tenant → no
        // collision.
        assert_ne!(doorbell_shm_name("g1", "a"), doorbell_shm_name("g2", "a"));
    }

    #[test]
    fn namespace_from_uses_present_user() {
        assert_eq!(namespace_from(Some("alice")), "alice");
    }

    #[test]
    fn namespace_from_falls_back_on_empty_or_unset() {
        assert_eq!(namespace_from(Some("")), "cerulion");
        assert_eq!(namespace_from(None), "cerulion");
    }

    #[test]
    fn default_namespace_is_nonempty_and_stable() {
        let a = default_namespace();
        let b = default_namespace();
        assert!(!a.is_empty());
        assert_eq!(a, b, "stable within a process");
    }

    // ---- PURE dedup / order (all OS) ----

    #[test]
    fn dedup_preserves_the_given_order_and_removes_dups() {
        let topics = ["a", "b", "a"].map(String::from);
        assert_eq!(
            dedup_topics(&topics),
            vec!["a".to_string(), "b".to_string()]
        );
    }

    #[test]
    fn dedup_keeps_first_occurrence_position() {
        // "b" first appears before its duplicate at the end → order is by FIRST
        // occurrence, not last.
        let topics = ["b", "a", "b", "c"].map(String::from);
        assert_eq!(
            dedup_topics(&topics),
            vec!["b".to_string(), "a".to_string(), "c".to_string()]
        );
    }

    #[test]
    fn dedup_empty_is_empty() {
        assert!(dedup_topics(&[]).is_empty());
    }

    // ---- registry dedup / order (all OS: real SHM where a page exists, stub elsewhere) ----

    #[test]
    #[allow(deprecated)] // the positional family is what this arm pins
    fn registry_dedups_and_primary_is_the_first_topic_given() {
        let ns = test_ns("reg_dedup");
        let topics = ["a", "b", "a"].map(String::from);
        let reg = DoorbellRegistry::open(&ns, &topics).expect("registry open");

        assert_eq!(reg.len(), 2, "['a','b','a'] dedups to 2 doorbells");
        assert!(!reg.is_empty());
        assert_eq!(
            reg.topics(),
            &["a".to_string(), "b".to_string()],
            "the order handed to open is preserved"
        );
        // primary == the first given topic ("a")'s line, cross-checked
        // against the name-based lookup, so this is NOT a self-compare.
        assert_eq!(reg.primary_addr(), reg.addr("a"));
        // The primary topic NAME (surfaced in the run_live wait-policy
        // line) is the same first given topic.
        assert_eq!(reg.primary_topic(), Some("a"));
        assert_ne!(
            reg.addr("a"),
            reg.addr("b"),
            "distinct topics → distinct lines"
        );
        assert_eq!(reg.snapshot_all().len(), 2);
        assert!(reg.addr("not-registered").is_none());

        drop(reg);
        cleanup(&ns, &["a", "b"]);
    }

    #[test]
    #[allow(deprecated)] // the positional family is what this arm pins
    fn registry_empty_has_no_primary() {
        let reg = DoorbellRegistry::open(&test_ns("reg_empty"), &[]).expect("open empty");
        assert!(reg.is_empty());
        assert_eq!(reg.len(), 0);
        assert!(reg.primary_addr().is_none());
        // No primary topic NAME either (the wait-policy line renders
        // "none").
        assert!(reg.primary_topic().is_none());
        assert!(reg.snapshot_all().is_empty());
        assert!(reg.topics().is_empty());
    }

    #[test]
    fn registry_reopen_known_topic_ok_unknown_errs() {
        let ns = test_ns("reg_reopen");
        let mut reg = DoorbellRegistry::open(&ns, &["a".to_string()]).expect("open");
        assert!(reg.reopen("missing").is_err(), "unknown topic → Err");
        assert!(reg.reopen("a").is_ok(), "registered topic re-maps");
        // Re-open leaves the topic registered (still resolvable).
        assert!(reg.addr("a").is_some());

        drop(reg);
        cleanup(&ns, &["a"]);
    }

    // ---- any_advanced_since (the load-bearing poll-all delta scan) ----

    #[test]
    fn any_advanced_since_shorter_baseline_and_equal_snapshot() {
        // Coverage for the conservative SHORTER-baseline branch + the equal-
        // snapshot no-advance branch — both run on any OS (the stub's seq()≡0 is
        // fine here; where a real page is mapped these are the no-ring
        // cases). Without this, a no-op
        // `any_advanced_since` impl would pass every other test.
        let ns = test_ns("any_adv_shorter");
        let topics = ["a", "b"].map(String::from);
        let reg = DoorbellRegistry::open(&ns, &topics).expect("registry open");

        // A baseline SHORTER than the doorbell set conservatively reports
        // advanced: `baseline.get(i)` is `None` → `is_none_or` → true.
        assert!(
            reg.any_advanced_since(&[]),
            "a baseline shorter than the doorbell set must conservatively report advanced"
        );

        // An EQUAL snapshot → no advance. On a target that maps a real page
        // this is the no-ring case; on the stub seq()≡0 holds it trivially.
        assert!(
            !reg.any_advanced_since(&reg.snapshot_all()),
            "an equal snapshot must report NO advance"
        );

        drop(reg);
        cleanup(&ns, &["a", "b"]);
    }

    /// The ONLY place the ring→advance path is exercised end-to-end, so it runs
    /// on every target that maps a real page and is skipped where the ring is the
    /// stub (seq()≡0 there, and there is no path to exercise). An OWNED producer doorbell
    /// and an `open_unowned` registry on the SAME `(ns, topic)` map the same page,
    /// so a producer `ring()` is observed by the consumer registry's poll-all.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    fn any_advanced_since_detects_a_ring() {
        let ns = test_ns("any_adv_ring");
        let topic = "scan";

        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let reg = DoorbellRegistry::open(&ns, &[topic.to_string()]).expect("registry open");

        // Snapshot the baseline BEFORE any ring: no advance yet.
        let baseline = reg.snapshot_all();
        assert!(
            !reg.any_advanced_since(&baseline),
            "no ring since the baseline → no advance"
        );

        // The producer rings its OWNED line (same physical page as the registry's
        // unowned mapping); the registry's poll-all now sees the delta.
        owner.ring();
        assert!(
            reg.any_advanced_since(&baseline),
            "a producer ring on the same page must be observed by the registry poll-all"
        );

        drop(reg);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    // ---- lifecycle (real SHM): create → ring → observe → drop-unlinks ----

    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    fn owner_rings_consumer_observes_and_owner_drop_unlinks() {
        use std::ffi::CString;

        let ns = test_ns("lifecycle");
        let topic = "scan";

        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        // The counter is RELATIVE — snapshot a base so a (rare same-pid) orphan
        // is harmless.
        let base = owner.seq();
        for _ in 0..3 {
            owner.ring();
        }
        assert_eq!(owner.seq(), base + 3, "owner observes its own rings");

        // Consumer maps the SAME page (O_CREAT idempotent) and sees the count.
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");
        assert_eq!(
            consumer.seq(),
            base + 3,
            "consumer observes the owner's rings (same physical page)"
        );
        owner.ring();
        owner.ring();
        assert_eq!(
            consumer.seq(),
            base + 5,
            "consumer observes subsequent rings"
        );
        // Distinct mappings → distinct virtual addresses (same physical page).
        assert_ne!(owner.addr(), consumer.addr());

        // The name exists while the owner is alive.
        let cname = CString::new(doorbell_shm_name(&ns, topic)).expect("cstring");
        // SAFETY: open-only probe of the name; mode is ignored without O_CREAT.
        let fd = unsafe { libc::shm_open(cname.as_ptr(), libc::O_RDONLY, 0) };
        assert!(fd >= 0, "shm object exists while owner is alive");
        // SAFETY: close the probe descriptor.
        unsafe {
            libc::close(fd);
        }

        // Dropping the OWNER shm_unlinks the name.
        drop(owner);
        // SAFETY: open-only probe — must now fail (name unlinked).
        let fd_after = unsafe { libc::shm_open(cname.as_ptr(), libc::O_RDONLY, 0) };
        assert!(
            fd_after < 0,
            "owner Drop shm_unlink'd the name (open-only now fails)"
        );

        // The consumer mapping is still valid memory (unlink ≠ unmap); it
        // munmaps WITHOUT unlinking on drop — no leak, the name is already gone.
        drop(consumer);
    }

    // ---- stub target: open/ring/seq/drop are no-ops that never panic ----

    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    #[test]
    fn stub_is_a_noop_and_does_not_panic() {
        let owner = Doorbell::open_owned("ns", "t").expect("stub open_owned");
        owner.ring();
        assert_eq!(owner.seq(), 0, "stub seq is a no-op 0");

        let consumer = Doorbell::open_unowned("ns", "t").expect("stub open_unowned");
        assert_eq!(consumer.seq(), 0);
        assert!(!owner.addr().is_null(), "stub addr is a valid scratch word");
        assert_ne!(
            owner.addr(),
            consumer.addr(),
            "each stub handle owns a distinct scratch word"
        );

        drop(owner);
        drop(consumer);
    }

    // ---- the kill-switch grammar (pure, every OS) ----

    /// `0` disables, `1`/unset keeps the word on, and anything else is GARBAGE:
    /// kept on, with a loud warn. The grammar is the crate's shared one, so this
    /// pins the wiring rather than a second parser.
    #[test]
    #[tracing_test::traced_test]
    fn the_kill_switch_grammar_disables_only_on_zero() {
        assert!(resolve_doorbell_os_sync_disabled(Some("0")));
        assert!(!resolve_doorbell_os_sync_disabled(Some("1")));
        assert!(!resolve_doorbell_os_sync_disabled(None));
        assert!(
            !logs_contain("CERULION_DOORBELL_OS_SYNC is set but not"),
            "a value the grammar accepts must not warn"
        );
        assert!(
            !resolve_doorbell_os_sync_disabled(Some("yes")),
            "garbage keeps the wake word ON - `0` is the explicit kill switch"
        );
        // The split exists so this warn is pinnable without env or OnceLock
        // games, and it must name THIS variable: a diagnostic pointing at a
        // sibling switch sends the operator to a knob that changes nothing here.
        assert!(
            logs_contain("CERULION_DOORBELL_OS_SYNC is set but not"),
            "garbage must warn, naming this variable"
        );
    }

    /// Off macOS there is no kernel wake word at all, so the availability gate
    /// is a compile-time `false` and the park rung is compiled out.
    #[cfg(not(target_os = "macos"))]
    #[test]
    fn the_wake_word_is_unavailable_off_macos() {
        assert!(!wake_word_block_primitive_available());
    }

    // ---- the macOS kernel wake word (real SHM, real syscalls) ----

    /// The availability rule's truth table, over an INJECTED pair rather than
    /// this host's answer: the tier is active only when the shared backend is
    /// usable AND this plane's kill switch is off. Runs on every OS.
    #[test]
    fn the_wake_word_is_active_only_with_a_backend_and_no_kill() {
        for (backend_usable, killed, expected) in [
            (true, false, true),
            (true, true, false),
            (false, false, false),
            (false, true, false),
        ] {
            assert_eq!(
                wake_word_active_from(backend_usable, killed),
                expected,
                "backend_usable={backend_usable} killed={killed} must give \
                 {expected}: dropping the backend term would block on an \
                 unresolved symbol, dropping the kill-switch term would ship an \
                 operator switch that does nothing"
            );
        }
    }

    /// The HEADLINE: a producer's ring WAKES a consumer kernel-blocked on the
    /// wake word, and wakes it far inside the cap.
    ///
    /// The ringer waits on the page's OWN `parked` gate rather than on a sleep:
    /// the ring has to land while the consumer holds the claim, which is what
    /// makes the wake syscall fire, and a sleep only arranges that with high
    /// probability. Spinning on the gate is both the deterministic handshake and
    /// the anti-vacuity evidence, so no wall-clock floor is asserted.
    ///
    /// The cap is seconds and the ceiling is a fraction of it: contention can
    /// only push the observed wall UP, and the failure this is built to catch
    /// (no wake at all) costs the WHOLE cap. A tight upper bound would be the
    /// class a loaded runner inverts, so none is asserted.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_ring_wakes_a_kernel_blocked_consumer_well_inside_the_cap() {
        use crate::monitor_wait::AddrParkOutcome;
        const CAP: Duration = Duration::from_secs(5);
        // Generous: the wake must merely beat the cap, not a tight wall.
        const CEILING: Duration = Duration::from_secs(2);
        // Bounded so a gate that never appears fails the test rather than
        // hanging it.
        const GATE_WAIT: Duration = Duration::from_secs(10);

        let ns = test_ns("wake_ring");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        let ringer = {
            let ns = ns.clone();
            std::thread::spawn(move || {
                let db = Doorbell::open_unowned(&ns, topic).expect("ringer open_unowned");
                let start = std::time::Instant::now();
                while db.parked_for_test() == 0 {
                    assert!(
                        start.elapsed() < GATE_WAIT,
                        "the consumer never claimed the parked gate, so the ring \
                         under test could not have been the thing that woke it"
                    );
                    std::thread::yield_now();
                }
                db.ring();
            })
        };

        let guard = ParkedDoorbellGuard::enter(&consumer);
        let t0 = std::time::Instant::now();
        let outcome = guard.wait(CAP);
        let waited = t0.elapsed();
        ringer.join().expect("ringer thread panicked");

        // Both arms are judged. Where the tier is available a real block must
        // have run and the ring must have ended it; where it is not (macOS
        // before 14.4, or CERULION_DOORBELL_OS_SYNC=0 in the invoking shell) the
        // call must report that NOTHING blocked, so the caller naps instead of
        // counting a wait it never got.
        if wake_word_block_primitive_available() {
            assert_eq!(
                outcome,
                AddrParkOutcome::Parked,
                "the tier is available and the ring landed after the snapshot was \
                 taken, so a real block ran and a real wake ended it"
            );
            assert!(
                waited < CEILING,
                "a ring must WAKE the kernel block, not let it time out - waited {waited:?} \
                 against a {CAP:?} cap"
            );
        } else {
            assert_eq!(
                outcome,
                AddrParkOutcome::Unavailable,
                "with no os_sync tier the wait must report that no block ran, so \
                 the caller naps instead of counting a wait it never got"
            );
        }

        drop(guard);
        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// A ring with NOBODY parked issues no wake syscall: the `parked` gate is
    /// what keeps a publish atomics-only on a topic no one is blocked on, which
    /// is the cost claim on `Doorbell::ring`. Counted at the syscall itself, so
    /// removing the gate fails this rather than merely changing a comment.
    #[cfg(target_os = "macos")]
    #[test]
    #[serial_test::serial]
    fn a_ring_with_nobody_parked_issues_no_wake_syscall() {
        let ns = test_ns("wake_gate");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        let before = wake_syscall_counts().0;
        owner.ring();
        assert_eq!(
            wake_syscall_counts().0,
            before,
            "a ring with nobody parked must not reach the kernel at all"
        );

        // The positive control: with the gate held the same ring DOES issue one
        // wherever the backend resolved, so the assertion above is about the gate
        // and not about a counter that can never move. Where it did not resolve
        // there is no syscall to count, which is judged rather than skipped.
        let guard = ParkedDoorbellGuard::enter(&consumer);
        owner.ring();
        if crate::os_sync::os_sync_backend().is_some() {
            assert!(
                wake_syscall_counts().0 > before,
                "a ring with a consumer parked must issue the wake syscall"
            );
        } else {
            assert_eq!(
                wake_syscall_counts().0,
                before,
                "with no resolved backend there is no wake syscall to issue, so \
                 the count cannot move in either direction"
            );
        }
        drop(guard);

        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// The lost-wake protocol: a ring landing AFTER the parker's snapshot and
    /// BEFORE its block must fail the kernel compare, so the block returns at
    /// once instead of sleeping through a wake nobody will repeat (Principle 6).
    #[cfg(target_os = "macos")]
    #[test]
    fn a_ring_after_the_snapshot_fails_the_compare_and_returns_at_once() {
        const CAP: Duration = Duration::from_secs(5);
        const CEILING: Duration = Duration::from_secs(2);

        let ns = test_ns("lost_wake");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        let guard = ParkedDoorbellGuard::enter(&consumer);
        // The ring lands after the guard read its compare value and before the
        // block. The guard is held, so a wake syscall IS issued; what this pins
        // is that the compare alone would have sufficed, because the call returns
        // without blocking.
        owner.ring();

        let t0 = std::time::Instant::now();
        let outcome = guard.wait(CAP);
        let waited = t0.elapsed();

        // Both arms are judged, as in the wake test above. RingPending, not
        // Parked: no wait ran, so the caller must not count a completed slice.
        if wake_word_block_primitive_available() {
            assert_eq!(
                outcome,
                crate::monitor_wait::AddrParkOutcome::RingPending,
                "a ring since the snapshot is a wake in flight, not a completed \
                 wait: counting it as a slice inflates the park telemetry by \
                 exactly the calls the feature makes most common"
            );
        } else {
            assert_eq!(
                outcome,
                crate::monitor_wait::AddrParkOutcome::Unavailable,
                "with no tier the call reports that nothing blocked"
            );
        }
        assert!(
            waited < CEILING,
            "a ring since the snapshot must return immediately - waited {waited:?} \
             against a {CAP:?} cap"
        );

        drop(guard);
        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// A zero cap blocks for nothing rather than risking an EINVAL, and reports
    /// that no block ran so the caller takes its nap.
    ///
    /// The nonzero companion is the control: without it the zero-cap assertion
    /// also passes on a host with no tier, where the call returns the same
    /// verdict for a different reason, and deleting the `cap.is_zero()` guard
    /// would be invisible there.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_zero_cap_runs_no_block() {
        use crate::monitor_wait::AddrParkOutcome;
        let ns = test_ns("zero_cap");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let guard = ParkedDoorbellGuard::enter(&owner);
        assert_eq!(
            guard.wait(Duration::ZERO),
            AddrParkOutcome::Unavailable,
            "a zero cap has nothing to block for"
        );
        assert_eq!(
            guard.wait(Duration::from_micros(1)),
            if wake_word_block_primitive_available() {
                AddrParkOutcome::Parked
            } else {
                AddrParkOutcome::Unavailable
            },
            "one microsecond over zero is a real wait wherever the tier exists, \
             so the zero-cap verdict above is about the cap"
        );
        drop(guard);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// The parked gate is the ringer's SYSCALL gate, so it must be set for the
    /// block's lifetime and released on every exit path, the RAII contract.
    /// Read through the page the OTHER handle maps, which also proves the gate
    /// is cross-mapping shared state rather than a process-local flag.
    #[cfg(target_os = "macos")]
    #[test]
    fn the_parked_gate_is_claimed_for_the_block_and_released_on_drop() {
        let ns = test_ns("parked_gate");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        assert_eq!(owner.parked_for_test(), 0, "nobody parked yet");
        {
            let _guard = ParkedDoorbellGuard::enter(&consumer);
            assert_eq!(
                owner.parked_for_test(),
                1,
                "the claim must be visible through the OTHER mapping of the page"
            );
        }
        assert_eq!(owner.parked_for_test(), 0, "the guard releases on drop");

        // Several consumers of one topic add up, which is why the gate is a
        // count and not a bit: the ringer must keep waking while ANY of them is
        // blocked.
        {
            let _outer = ParkedDoorbellGuard::enter(&consumer);
            let inner = ParkedDoorbellGuard::enter(&consumer);
            assert_eq!(owner.parked_for_test(), 2, "two claims add up");
            drop(inner);
            assert_eq!(owner.parked_for_test(), 1, "one release leaves the other");
        }
        assert_eq!(owner.parked_for_test(), 0, "both releases land");

        // An UNWIND is the exit path a `return` cannot stand in for: a panic
        // inside the block leaves the claim set unless Drop runs, and the ringer
        // then pays a wake syscall on every publish for a waiter that is gone.
        let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = ParkedDoorbellGuard::enter(&consumer);
            panic!("unwind with the claim held");
        }));
        assert!(unwound.is_err(), "the panic must really have unwound");
        assert_eq!(
            owner.parked_for_test(),
            0,
            "the claim must be released on an unwind, not only on a normal exit"
        );

        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// A ring bumps BOTH words in the page: the relative ring counter the
    /// poll-all reads, and the wake epoch the kernel compares. Dropping either
    /// bump breaks a different half of the design, so both are pinned.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_ring_bumps_both_the_ring_counter_and_the_wake_epoch() {
        let ns = test_ns("two_words");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        let seq0 = consumer.seq();
        let guard = ParkedDoorbellGuard::enter(&consumer);
        let epoch0 = guard.snapshot();
        drop(guard);
        owner.ring();
        // EXACTLY one, not merely different: a double bump would pass an
        // inequality and would make the consumer's ring delta count publishes
        // that never happened.
        assert_eq!(
            consumer.seq(),
            seq0 + 1,
            "one ring advances the ring counter by exactly one"
        );
        let guard = ParkedDoorbellGuard::enter(&consumer);
        assert_eq!(
            guard.snapshot(),
            epoch0.wrapping_add(1),
            "one ring advances the wake epoch by exactly one, or a parked peer's \
             kernel compare would succeed and it would sleep through the ring"
        );
        drop(guard);

        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// The registry's PRIMARY is the FIRST-declared topic's doorbell, which is
    /// the line the kernel block watches. Proven by ringing each topic in turn
    /// and reading the primary's own counter, so a `last()` would fail: the
    /// `primary_addr`-against-`addr` comparison in the dedup test cannot catch
    /// that, because it compares two lookups in the same registry.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    #[allow(deprecated)] // the positional family is what this arm pins
    fn the_registry_primary_is_the_first_given_topics_bell() {
        let ns = test_ns("primary_bell");
        let topics = ["first", "second"].map(String::from);
        let reg = DoorbellRegistry::open(&ns, &topics).expect("registry open");
        let first = Doorbell::open_owned(&ns, "first").expect("owner first");
        let second = Doorbell::open_owned(&ns, "second").expect("owner second");

        let before = reg
            .primary()
            .expect("a two-topic registry has a primary")
            .seq();
        second.ring();
        assert_eq!(
            reg.primary().expect("primary").seq(),
            before,
            "a ring on the SECOND topic must not move the primary's counter"
        );
        first.ring();
        assert_eq!(
            reg.primary().expect("primary").seq(),
            before + 1,
            "the primary is the FIRST-declared topic's bell"
        );

        drop(reg);
        drop(first);
        drop(second);
        cleanup(&ns, &["first", "second"]);
    }

    /// Claim a doorbell name and leave the object UNSIZED, exactly as a creator
    /// killed between its `shm_open` and its `ftruncate` does. Returns the
    /// descriptor, which the caller holds so the object stays alive and readable.
    #[cfg(target_os = "macos")]
    fn unsized_fixture(ns: &str, topic: &str) -> (std::ffi::CString, libc::c_int) {
        let name =
            std::ffi::CString::new(doorbell_shm_name(ns, topic)).expect("name has no interior nul");
        // SAFETY: claim a pid-scoped name and do NOT size it.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "the fixture name must be free in this test's namespace"
        );
        // The premise: the kernel really reports this object as shorter than the
        // page, so the open path has something to judge.
        // SAFETY: `st` is zeroed first; `fstat` fills it on success.
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: FFI fstat on the descriptor just opened.
        assert_eq!(unsafe { libc::fstat(fd, &mut st) }, 0, "fstat the fixture");
        assert!(
            (st.st_size as usize) < doorbell_page_bytes_for_test(),
            "precondition: an unsized object must fstat below the page, or the \
             tests below judge nothing"
        );
        (name, fd)
    }

    /// An UNSIZED object under a doorbell name is SIZED IN PLACE, so the topic is
    /// not wedged and no name is deleted: the opener and the peer that left it end
    /// on the SAME page.
    ///
    /// This is the shape a creator killed between its `shm_open` and its
    /// `ftruncate` leaves behind, and before the sizing step nothing else ever
    /// re-sized it, so every later open of that topic failed for the life of the
    /// machine.
    ///
    /// MEASURED, and why the fixture is unsized rather than one byte short: macOS
    /// reports a POSIX SHM object's size ROUNDED UP to a page, so an `ftruncate` to
    /// `page_bytes() - 1` fstats as 16384 and the open attaches to it correctly.
    /// Zero is the only short size the open path can observe on this target, and it
    /// is also the only one a crash produces.
    #[cfg(target_os = "macos")]
    #[test]
    fn an_unsized_doorbell_object_is_sized_in_place() {
        let ns = test_ns("unsized");
        let topic = "scan";
        let (name, dead) = unsized_fixture(&ns, topic);

        let bell = Doorbell::open_unowned(&ns, topic)
            .expect("an unsized object must be sized in place, not refused");
        assert_eq!(bell.seq(), 0, "a freshly sized page reads zero");

        // The SAME object, not a replacement: the descriptor this test still holds
        // on the object the fixture created now reports the page size, and the name
        // still resolves to it. Deleting the name instead would put a peer that was
        // merely descheduled on a different page from every consumer, with nothing
        // reported anywhere.
        // SAFETY: `st` is zeroed first; `fstat` fills it on success.
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: FFI fstat on the descriptor this test opened.
        assert_eq!(
            unsafe { libc::fstat(dead, &mut st) },
            0,
            "fstat the fixture"
        );
        assert!(
            (st.st_size as usize) >= doorbell_page_bytes_for_test(),
            "the object the fixture created must itself have been sized, so the \
             peer that left it maps the same page"
        );
        // SAFETY: FFI open-only probe of the fixture name.
        let probe = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDONLY, 0) };
        assert!(probe >= 0, "the name must not have been unlinked");
        // SAFETY: close the probe descriptor.
        unsafe { libc::close(probe) };

        // The SAME page, proven by a write THROUGH the fixture's own descriptor
        // showing up in the mapping this open made. This is what says no name was
        // deleted and no second object created, and it is the only way to say it:
        // MEASURED, `fstat` reports `(st_dev, st_ino)` as `(0, 0)` for every POSIX
        // shared-memory object on this target, so an inode comparison between two
        // descriptors is vacuous here.
        // SAFETY: map the fixture's own descriptor, which the sizing above made
        // mappable, and write the ring counter's first word through it.
        let mapped = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                doorbell_page_bytes_for_test(),
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                dead,
                0,
            )
        };
        assert_ne!(
            mapped,
            libc::MAP_FAILED,
            "the object the fixture created must be mappable, which it is only \
             because the open sized it in place"
        );
        // SAFETY: `mapped` is a live MAP_SHARED mapping of at least 8 bytes, and
        // the doorbell's ring counter is an AtomicU64 at offset 0 of the same page.
        let ring_through_fixture = unsafe { &*(mapped as *const std::sync::atomic::AtomicU64) };
        ring_through_fixture.fetch_add(7, std::sync::atomic::Ordering::Release);
        assert_eq!(
            bell.seq(),
            7,
            "a store through the FIXTURE's own descriptor must show in the mapping \
             this open made, or the two are on different pages and a name was \
             replaced under a live peer"
        );
        // SAFETY: unmap the mapping this test made.
        unsafe { libc::munmap(mapped, doorbell_page_bytes_for_test()) };

        // SAFETY: close the descriptor this test opened.
        unsafe { libc::close(dead) };
        drop(bell);
        cleanup(&ns, &[topic]);
    }

    /// The two wake-word flood regimes are SEPARATE, and the wake one closes.
    ///
    /// A shared regime would give the loud head to whichever half failed first and
    /// downgrade the other's, so an operator chasing a failing WAKE would read a
    /// line about the WAIT and act on the wrong remedy. Driven directly, because
    /// both are module-private functions over pure latch state: no syscall, no env,
    /// no thread.
    #[cfg(target_os = "macos")]
    #[test]
    #[serial_test::serial]
    #[tracing_test::traced_test]
    fn the_wake_and_wait_regimes_are_separate_and_the_wake_one_closes() {
        use crate::transport::failure_regime_latch::RegimeDecision;
        // Each half opens its OWN regime, so each gets its own loud head.
        assert!(matches!(note_wake_errno(), RegimeDecision::Loud));
        assert!(matches!(note_wait_errno(), RegimeDecision::Loud));
        // A repeat on the wake side is downgraded, and the count it reports is the
        // wake side's alone.
        assert!(matches!(
            note_wake_errno(),
            RegimeDecision::Suppressed { suppressed: 1 }
        ));
        // The publish path's gate says there is something to close, which is what
        // keeps the lock off it while nothing is failing.
        assert!(wake_regime_is_open(), "a failing wake opens the regime");
        note_wake_recovered();
        assert!(
            !wake_regime_is_open(),
            "a recovery closes it, so the next publish takes one relaxed load"
        );
        assert!(
            logs_contain("os_sync_wake_by_address is succeeding again"),
            "the recovery reports what it suppressed"
        );
        // The WAIT regime is untouched by the wake side's recovery: its next repeat
        // is still a repeat.
        assert!(matches!(
            note_wait_errno(),
            RegimeDecision::Suppressed { suppressed: 1 }
        ));
        // And the wake side's next failure is LOUD again, or its regime never
        // really closed.
        assert!(matches!(note_wake_errno(), RegimeDecision::Loud));
        note_wake_recovered();
    }

    /// Both arms of the open path's retry loop, driven over an injected attempt.
    ///
    /// Neither is reachable through the real syscalls from a test: the race is
    /// another process unlinking a name between this one's `EEXIST` and its attach,
    /// and the exhaustion arm needs that to happen on every attempt. Runs on every
    /// OS, so the bound is pinned where the loop itself is not built.
    #[test]
    fn the_open_retry_loop_retries_a_lost_name_race_and_then_reports() {
        // A race followed by a success: the retry is what turns a peer's unlink into
        // an opened doorbell rather than a run without its data-wake path.
        let mut seen = 0;
        let got = retry_lost_name_race(|| {
            seen += 1;
            if seen == 1 {
                Err(OpenAttemptKind::RaceLostName)
            } else {
                Ok("opened")
            }
        })
        .expect("a lost race must be retried, not reported");
        assert_eq!(got, "opened");
        assert_eq!(
            seen, 2,
            "exactly one retry was needed and exactly one was made"
        );

        // A fatal error stops at once: retrying it would multiply a real failure by
        // the bound.
        let mut seen = 0;
        let err = retry_lost_name_race::<()>(|| {
            seen += 1;
            Err(OpenAttemptKind::Fatal(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "denied",
            )))
        })
        .expect_err("a fatal error is the caller's");
        assert_eq!(err.kind(), io::ErrorKind::PermissionDenied);
        assert_eq!(seen, 1, "a fatal error is never retried");

        // Losing every time is reported, bounded, and the message says what the loop
        // saw rather than guessing at a cause.
        let mut seen = 0;
        let err = retry_lost_name_race::<()>(|| {
            seen += 1;
            Err(OpenAttemptKind::RaceLostName)
        })
        .expect_err("an exhausted loop must report");
        assert_eq!(err.kind(), io::ErrorKind::WouldBlock);
        assert!(
            err.to_string().contains(&format!("all {seen} attempts")),
            "the message states the bound it actually spent: {err}"
        );
        assert_eq!(
            seen, 3,
            "the loop is bounded, so a hostile peer cannot spin it"
        );
    }

    /// The parked gate's underflow arm: the decision is pure, so both sides of it
    /// are pinned here rather than left to a `debug_assert` that is compiled out of
    /// every build where the warn is the only evidence.
    ///
    /// Runs on every OS, like the other pure decisions in this file. The function it
    /// calls compiles in every test build, so a macOS-only test would leave it with
    /// no user on Linux, and this crate denies dead code.
    #[test]
    fn a_parked_count_of_zero_at_exit_is_an_underflow() {
        assert_eq!(
            parked_decrement(0),
            None,
            "zero at exit means the shared count was written by something else, and \
             the peer whose wake it costs cannot see that"
        );
        assert_eq!(parked_decrement(1), Some(0));
        assert_eq!(parked_decrement(u32::MAX), Some(u32::MAX - 1));
    }

    /// An object at EXACTLY one cache line is ATTACHED, not replaced: the other
    /// side of the bound `an_unsized_doorbell_object_is_sized_in_place` pins, with
    /// the fixture derived from the same constant (and reported by the kernel as a
    /// whole page, per the note on that test).
    ///
    /// Attaching rather than replacing is the whole first-wins contract, so it is
    /// proven by a ring on a second handle showing through the page this open
    /// mapped, not merely by the open succeeding.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_doorbell_object_at_exactly_one_cache_line_opens() {
        let ns = test_ns("exact_obj");
        let topic = "scan";
        let name = std::ffi::CString::new(doorbell_shm_name(&ns, topic))
            .expect("name has no interior nul");
        // SAFETY: create and size a fixture object under a pid-scoped name.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "the fixture name must be free in this test's namespace"
        );
        // SAFETY: size the object we just created.
        assert_eq!(
            unsafe { libc::ftruncate(fd, doorbell_page_bytes_for_test() as libc::off_t) },
            0,
            "ftruncate the fixture"
        );
        // SAFETY: close the fixture descriptor; the name keeps the object alive.
        unsafe { libc::close(fd) };

        let bell = Doorbell::open_unowned(&ns, topic)
            .expect("an object at exactly one cache line must open");
        assert_eq!(bell.seq(), 0, "a fresh page reads zero");

        // ATTACHED, not replaced: a ring through a second handle on the same name
        // shows through this one. Had the open unlinked and re-created the object
        // at the limit, the two handles would sit on different pages and this
        // would read zero.
        let peer = Doorbell::open_owned(&ns, topic).expect("peer open_owned");
        peer.ring();
        assert_eq!(
            bell.seq(),
            1,
            "an object at exactly the bound must be ATTACHED, so a peer's ring \
             shows through the page this open mapped"
        );

        drop(peer);
        drop(bell);
        cleanup(&ns, &[topic]);
    }
}

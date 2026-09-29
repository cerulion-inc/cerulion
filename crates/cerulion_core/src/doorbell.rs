// SPDX-License-Identifier: AGPL-3.0-only
//! Per-topic SHM cache-line "doorbell" — a process-shared `AtomicU64` the
//! producer RINGS after each publish and the consumer MONITOR-WAITS on, so the
//! live loop's CPU monitor-wait ([`crate::monitor_wait`]) wakes the instant a
//! producer stores, not only on the timer.
//!
//! # Why
//!
//! [`crate::monitor_wait`] parks the inter-step idle in a SHALLOW optimized CPU
//! state (no deep cpuidle C-state, hence no cold-wake), but on its own only the
//! TSC/event-stream timer wakes it — so it re-polls the iceoryx2 listener every
//! `recheck` (~100µs). To wake on DATA the instant it arrives, every
//! data-trigger topic gets one cache-line-aligned `AtomicU64` in a `MAP_SHARED`
//! page that producer and consumer both map: the producer `ring()`s (a store)
//! on each publish and the consumer arms `UMONITOR`/`WFE` on that exact line, so
//! the store wakes the park with no timer round-trip.
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
//! the orphan; the runtime re-maps via [`DoorbellRegistry::reopen`] on a
//! producer-reconnect `LivelinessEvent`. Full restart-race
//! correctness is the runtime's timer-recheck backstop; this
//! module only guarantees it does not make that worse.
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
//!   rung by a plain store and heard by the CPU monitor-wait primitive
//!   (`UMONITOR`/`WFE`) armed on the same line.
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

use std::io;
use std::sync::atomic::AtomicU64;

/// Derive the POSIX SHM object name for `(ns, topic)`.
///
/// Per-OS shape, for the reason the credit word's name carries the same split:
/// - **linux**, [`doorbell_shm_name_verbose`]: `/cer_db_<ns>_<fnv1a64(topic):016x>`.
///   The raw `<ns>` in the `/dev/shm` filename is a deliberate debuggability aid
///   (an `ls /dev/shm` names the deployment).
/// - **non-linux (macos)**, [`doorbell_shm_name_compact`]:
///   `/cer_db_<fnv1a64(ns 0x1f topic):016x>`, 24 chars. macOS caps POSIX SHM
///   names at 31 chars (`PSHMNAMLEN`, the leading slash included) and a `$USER`
///   namespace alone already pushes the verbose form to the edge of it, so BOTH
///   components are hashed into one fixed-width token.
///
/// Either shape preserves the tenant partition (a different `ns` gives a
/// different name; the compact form separates `ns` from `topic` with a 0x1F unit
/// separator so `("ab","c")` and `("a","bc")` can never alias). Pure (no I/O), so
/// both are hermetically testable on every OS. Stable across processes, so the
/// producer and consumer of the same `(ns, topic)` derive the same name → map
/// the same page. This is also the registry's dedup key.
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

/// The macOS-safe name shape: 24 chars, under the macOS 31-char `PSHMNAMLEN`
/// cap for ANY `(ns, topic)`. The 0x1F unit separator keeps the split
/// unambiguous. See [`doorbell_shm_name`].
#[cfg(any(not(target_os = "linux"), test))]
fn doorbell_shm_name_compact(ns: &str, topic: &str) -> String {
    // hot-path-alloc-ok: name derivation runs only at create/open (cold path).
    let mut key = Vec::with_capacity(ns.len() + 1 + topic.len());
    key.extend_from_slice(ns.as_bytes());
    key.push(0x1f);
    key.extend_from_slice(topic.as_bytes());
    let h = crate::shm_map::fnv1a64(&key);
    format!("/cer_db_{h:016x}")
}

/// Deduplicate `topics` preserving FIRST-DECLARED order — the registry maps one
/// doorbell per UNIQUE topic, and `primary` must remain the first-declared one.
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
/// the same process derive the SAME value, so they agree on the
/// `/cer_db_<ns>_<hash>` object name and map the same page. `$USER`-based (not
/// graph-name-based) so it is also stable across a single user's processes —
/// forward-compatible with the p4 cross-process doorbell. The literal `<ns>`
/// prefix kills cross-tenant (different-user) collisions.
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
        /// and leaks its claim. The cost is bounded and is latency only: every
        /// later ring pays one wake syscall whose result is already ignored, for
        /// a waiter that is not there. Correctness is untouched, because the
        /// claim only ever gates a wake. (The barrier's equivalent stale bit is
        /// swept by the supervisor, which knows the dead rank; there is no
        /// equivalent sweep here because there is no rank to name. A re-created
        /// producer unlinks and re-creates the page, which clears it.)
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
        /// not), which is why the size is set only on the branch that really
        /// created it. A racing creator can be seen between its `shm_open` and its
        /// `ftruncate`, so an object that is still short is retried briefly and
        /// then reported: the caller warns and the park keeps its timer backstop.
        fn open(ns: &str, topic: &str, owns_name: bool) -> io::Result<Self> {
            let name = CString::new(doorbell_shm_name(ns, topic))
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            // How long to wait out a racing creator's `ftruncate`. The window is
            // two adjacent syscalls, so this is generous by orders of magnitude;
            // it is bounded so a truly broken object reports rather than spins.
            const SIZE_RETRIES: u32 = 100;
            const SIZE_RETRY_SLEEP: Duration = Duration::from_micros(200);

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
                // SAFETY: size the object we just created exclusively; `fd` is
                // the descriptor just opened.
                if unsafe { libc::ftruncate(fd, DOORBELL_BYTES as libc::off_t) } < 0 {
                    let err = io::Error::last_os_error();
                    // SAFETY: `fd` is the descriptor we opened.
                    unsafe { libc::close(fd) };
                    // A half-made object under a good name would wedge every
                    // later opener, so remove the name we just claimed.
                    // SAFETY: best-effort unlink of the name THIS call created.
                    unsafe { libc::shm_unlink(name.as_ptr()) };
                    return Err(err);
                }
            } else {
                let err = io::Error::last_os_error();
                if err.raw_os_error() != Some(libc::EEXIST) {
                    return Err(err);
                }
                // Somebody else owns the name: attach to THEIR page.
                // SAFETY: FFI open of an existing named SHM object.
                fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDWR, 0) };
                if fd < 0 {
                    return Err(io::Error::last_os_error());
                }
            }

            if !created {
                // Wait out the creator's `ftruncate`. Mapping a short object is
                // refused by macOS with an errno that names nothing, so the size is
                // checked here, where the error can say what it found.
                let mut ok = false;
                for _ in 0..SIZE_RETRIES {
                    // SAFETY: `st` is zeroed first so a failed `fstat` leaves no
                    // uninitialised read; `fstat` fills it on success.
                    let mut st: libc::stat = unsafe { std::mem::zeroed() };
                    // SAFETY: FFI fstat on the descriptor we hold.
                    if unsafe { libc::fstat(fd, &mut st) } < 0 {
                        let err = io::Error::last_os_error();
                        // SAFETY: `fd` is the descriptor we opened.
                        unsafe { libc::close(fd) };
                        return Err(err);
                    }
                    if st.st_size >= DOORBELL_BYTES as libc::off_t {
                        ok = true;
                        break;
                    }
                    std::thread::sleep(SIZE_RETRY_SLEEP);
                }
                if !ok {
                    // SAFETY: `fd` is the descriptor we opened.
                    unsafe { libc::close(fd) };
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "doorbell segment stayed shorter than one cache line (a creator died \
                         between shm_open and ftruncate, leaving a zero-length object under \
                         the name)",
                    ));
                }
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
                if created {
                    // SAFETY: best-effort unlink of the name THIS call created;
                    // a failed create must not orphan a named segment.
                    unsafe { libc::shm_unlink(name.as_ptr()) };
                }
                return Err(err);
            }
            Ok(Self {
                ptr: addr as *mut DoorbellShared,
                name,
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
        /// Two `Release` stores into one cache line, a `SeqCst` fence, and a
        /// `Relaxed` load. The fence is a store-buffer drain (`dmb ish` on this
        /// target, tens of cycles when the buffer is dirty) and it is the
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
        /// locally latched costs one syscall whose result is already ignored; a
        /// wake SKIPPED costs the peer a full park slice on every publish. The
        /// same asymmetry the credit word prices the same way.
        #[inline]
        fn wake_word_wake(&self) {
            if let Some(backend) = os_sync_backend() {
                // SAFETY: `wake_word_addr()` is the live 4-byte `wake_seq` in the
                // mapping this handle owns; `os_sync_wake_by_address_all` reads no
                // user memory beyond keying on address + size. `backend.wake` is
                // the dlsym-resolved, signature-checked fn pointer.
                unsafe {
                    (backend.wake)(
                        self.wake_word_addr(),
                        OS_SYNC_WORD_SIZE,
                        libc::OS_SYNC_WAKE_BY_ADDRESS_SHARED,
                    );
                }
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
        pub fn wake_seq_snapshot(&self) -> u32 {
            self.shared().wake_seq.load(Ordering::Acquire)
        }

        /// Mark this doorbell as kernel-blocked-on RIGHT NOW (the syscall gate the
        /// ringer reads). Callers use [`ParkedDoorbellGuard`] so the bit is
        /// cleared on every exit path.
        ///
        /// The `SeqCst` fence after the bit-set is the parker's half of the
        /// store-buffer litmus pair described on [`Doorbell::ring`].
        pub fn park_enter(&self) {
            self.shared().parked.fetch_add(1, Ordering::AcqRel);
            core::sync::atomic::fence(Ordering::SeqCst);
        }

        /// Clear this process's parked claim (park exit).
        pub fn park_exit(&self) {
            self.shared().parked.fetch_sub(1, Ordering::AcqRel);
        }

        /// Kernel-block on the wake word until a ring bumps the epoch past
        /// `snapshot` or `cap` expires. Returns `true` when a real block ran (the
        /// caller then skips its pacing nap), `false` when nothing blocked: a
        /// zero `cap`, an inactive tier, or an unresolved backend, each of which
        /// leaves the caller to nap.
        ///
        /// Record-only (Principle 7): this changes only WHEN the park returns. The
        /// caller's own ring-delta re-derive after every wake stays the sole
        /// correctness, and the message itself is still read from the iceoryx2 SHM
        /// queue by the step.
        pub fn park_wait_ring(&self, snapshot: u32, cap: Duration) -> bool {
            if cap.is_zero() {
                // Nothing to block for; a zero-timeout kernel call risks EINVAL
                // for no benefit.
                return false;
            }
            if !super::doorbell_os_sync_tier_active() {
                return false;
            }
            let Some(backend) = os_sync_backend() else {
                return false;
            };
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
            if rc < 0 {
                let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if os_sync_errno_is_unrecoverable(errno) {
                    // One latch for the whole os_sync family: an EINVAL/ENOTSUP
                    // means the primitive is unusable, not one call shape.
                    if crate::os_sync::latch_os_sync_disabled() {
                        tracing::warn!(
                            errno,
                            "doorbell wake word os_sync_wait_on_address returned an \
                             unrecoverable errno (EINVAL/ENOTSUP); disabling the os_sync tier \
                             process-wide - the data-wake park falls back to sleep-recheck \
                             pacing"
                        );
                    }
                    return false;
                }
                if !os_sync_errno_is_benign(errno) {
                    static LAST_WARNED_ERRNO: std::sync::atomic::AtomicI32 =
                        std::sync::atomic::AtomicI32::new(0);
                    if LAST_WARNED_ERRNO.swap(errno, Ordering::Relaxed) != errno {
                        tracing::warn!(
                            errno,
                            "doorbell wake word os_sync_wait_on_address returned an unexpected \
                             errno; the data-wake park degrades to bounded sleep pacing for \
                             this errno"
                        );
                    }
                    std::thread::sleep(Duration::from_micros(100).min(cap));
                }
            }
            true
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
/// Its OWN switch, not the barrier's or the park nap's, for the reason those two
/// are separate from each other: the env names are consumer-facing surface, and
/// one switch silently disabling an unrelated tier is the misleading-name class
/// this repo rejects. All of them still ride ONE backend and ONE
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
/// on the same line, which is a hardware park the kernel block must not displace.
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

/// RAII claim on a doorbell's `parked` gate: set on `enter`, cleared on EVERY
/// exit path including an unwind, so a ringer can never be left issuing a wake
/// syscall for a consumer that is no longer blocked.
#[cfg(target_os = "macos")]
#[must_use = "the parked claim is released on drop - bind the guard for the block's lifetime"]
pub struct ParkedDoorbellGuard<'a> {
    bell: &'a Doorbell,
}

#[cfg(target_os = "macos")]
impl<'a> ParkedDoorbellGuard<'a> {
    /// Claim the gate for the duration of one kernel block.
    pub fn enter(bell: &'a Doorbell) -> Self {
        bell.park_enter();
        Self { bell }
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
/// Opens and dedups exactly one doorbell per UNIQUE topic (first-declared order
/// preserved), RAII-owning them all (every one unowned — production producers
/// own + `shm_unlink` their own lines via [`Doorbell::open_owned`]). The runtime
/// uses [`snapshot_all`](DoorbellRegistry::snapshot_all) for the record-only
/// poll-all, [`primary_addr`](DoorbellRegistry::primary_addr) to arm the single
/// hardware monitor on the first-declared topic's line, and
/// [`reopen`](DoorbellRegistry::reopen) on a producer-reconnect `LivelinessEvent`.
#[must_use = "the registry unmaps all its doorbells on drop — bind it to a named local for the desired scope"]
pub struct DoorbellRegistry {
    /// Namespace passed at construction — retained so [`reopen`] can re-derive
    /// SHM names.
    ///
    /// [`reopen`]: DoorbellRegistry::reopen
    ns: String,
    /// Deduped topics in first-declared order; parallel to `doorbells`.
    topics: Vec<String>,
    /// One unowned doorbell per topic, parallel to `topics`.
    doorbells: Vec<Doorbell>,
}

impl DoorbellRegistry {
    /// Open one unowned doorbell per UNIQUE topic in `topics` (first-declared
    /// order preserved) under namespace `ns`.
    ///
    /// Returns the first open error on a target with a real SHM page (linux,
    /// macos); on the no-op stub every open succeeds.
    pub fn open(ns: &str, topics: &[String]) -> io::Result<Self> {
        let topics = dedup_topics(topics);
        let mut doorbells = Vec::with_capacity(topics.len());
        for topic in &topics {
            doorbells.push(Doorbell::open_unowned(ns, topic)?);
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

    /// The deduped topics, in first-declared order (parallel to the doorbells).
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
    /// a NON-primary topic (the primary line is hardware-armed; the rest wake on
    /// this ≤100µs-recheck delta scan). RELATIVE counters ⇒ only the delta from
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

    /// The first-declared topic's doorbell line — the single line the hardware
    /// monitor (`UMONITOR`/`WFE`) is armed on. `None` when the registry is empty.
    pub fn primary_addr(&self) -> Option<*const AtomicU64> {
        self.doorbells.first().map(Doorbell::addr)
    }

    /// The first-declared topic's DOORBELL: the handle whose wake word the
    /// macOS data-wake park blocks on (the same line
    /// [`primary_addr`](Self::primary_addr) hands the hardware monitor). `None`
    /// when the registry is empty.
    pub fn primary(&self) -> Option<&Doorbell> {
        self.doorbells.first()
    }

    /// The first-declared topic NAME — the one hardware-armed on
    /// [`primary_addr`](Self::primary_addr). Surfaced in the `run_live` wait-policy
    /// telemetry line so a worker's chosen doorbell primary is observable (the
    /// CLI monolith arm never runs in a worker). `None` when the registry is empty.
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

    /// Index of `topic` in the deduped topic list, or `None`.
    fn index_of(&self, topic: &str) -> Option<usize> {
        self.topics.iter().position(|t| t.as_str() == topic)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Only the macOS kernel-block arms take a timeout, so the import is gated
    // with them: unconditional, it is an unused import off macOS.
    #[cfg(target_os = "macos")]
    use std::time::Duration;

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
            "every shape carries the prefix-free family token: {n1}"
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

    /// The COMPACT shape is fixed-width and fits the macOS `PSHMNAMLEN` cap for
    /// ANY `(ns, topic)`, including the deployment namespaces that blow past
    /// the verbose form. The cap is the reason the shape exists, so it is
    /// asserted on the longest input a deployment can produce, not a short one.
    #[test]
    fn the_compact_name_is_fixed_width_and_fits_the_macos_cap() {
        for (ns, topic) in [
            ("g", "a"),
            ("", ""),
            (
                "cerdep_a_very_long_graph_name_with_a_nonce_0123456789abcdef",
                "/an/absolute/topic/name/that/is/also/quite/long/indeed",
            ),
        ] {
            let n = doorbell_shm_name_compact(ns, topic);
            assert_eq!(n.len(), 24, "compact name is fixed-width: {n}");
            assert!(n.len() <= 31, "must fit PSHMNAMLEN: {n}");
            assert!(n.starts_with("/cer_db_"), "family prefix: {n}");
        }
    }

    /// The 0x1F unit separator is what makes the two-component hash
    /// unambiguous: without it `("ab","c")` and `("a","bc")` would hash the same
    /// key and two different tenants would share one page.
    #[test]
    fn the_compact_name_cannot_alias_across_the_ns_topic_split() {
        assert_ne!(
            doorbell_shm_name_compact("ab", "c"),
            doorbell_shm_name_compact("a", "bc")
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
    fn dedup_preserves_first_declared_order_and_removes_dups() {
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

    // ---- registry dedup / order (all OS — stub off Linux, real SHM on Linux) ----

    #[test]
    fn registry_dedups_and_primary_is_first_declared() {
        let ns = test_ns("reg_dedup");
        let topics = ["a", "b", "a"].map(String::from);
        let reg = DoorbellRegistry::open(&ns, &topics).expect("registry open");

        assert_eq!(reg.len(), 2, "['a','b','a'] dedups to 2 doorbells");
        assert!(!reg.is_empty());
        assert_eq!(
            reg.topics(),
            &["a".to_string(), "b".to_string()],
            "first-declared order preserved"
        );
        // primary == the first-declared topic ("a")'s line — cross-checked
        // against the name-based lookup, so this is NOT a self-compare.
        assert_eq!(reg.primary_addr(), reg.addr("a"));
        // The primary topic NAME (surfaced in the run_live wait-policy
        // line) is the same first-declared topic.
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
        // fine here; on Linux these are the no-ring cases). Without this, a no-op
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
    fn the_kill_switch_grammar_disables_only_on_zero() {
        assert!(resolve_doorbell_os_sync_disabled(Some("0")));
        assert!(!resolve_doorbell_os_sync_disabled(Some("1")));
        assert!(!resolve_doorbell_os_sync_disabled(None));
        assert!(
            !resolve_doorbell_os_sync_disabled(Some("yes")),
            "garbage keeps the wake word ON - `0` is the explicit kill switch"
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
    /// The oracle is load-SAFE by construction. The cap is seconds and the ring
    /// lands at a small fraction of it, so the assertion is "returned well
    /// before the cap": contention can only push the observed wall UP, and the
    /// failure it is built to catch (no wake at all) costs the WHOLE cap. A
    /// tight upper bound would be the class a loaded runner inverts, so none is
    /// asserted.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_ring_wakes_a_kernel_blocked_consumer_well_inside_the_cap() {
        const CAP: Duration = Duration::from_secs(5);
        const RING_AT: Duration = Duration::from_millis(150);
        // Generous: the wake must merely beat the cap, not a tight wall.
        const CEILING: Duration = Duration::from_secs(2);

        let ns = test_ns("wake_ring");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        let consumer = Doorbell::open_unowned(&ns, topic).expect("consumer open_unowned");

        let ringer = {
            let ns = ns.clone();
            std::thread::spawn(move || {
                std::thread::sleep(RING_AT);
                let db = Doorbell::open_unowned(&ns, topic).expect("ringer open_unowned");
                db.ring();
            })
        };

        let _guard = ParkedDoorbellGuard::enter(&consumer);
        let snap = consumer.wake_seq_snapshot();
        let t0 = std::time::Instant::now();
        let blocked = consumer.park_wait_ring(snap, CAP);
        let waited = t0.elapsed();
        ringer.join().expect("ringer thread panicked");

        // Both arms are judged. Where the tier is available the block must run
        // and the ring must end it; where it is not (macOS before 14.4, or
        // CERULION_DOORBELL_OS_SYNC=0 in the invoking shell) the call must
        // report that NOTHING blocked, so the caller takes its nap instead of
        // believing a wait it never got.
        if wake_word_block_primitive_available() {
            assert!(
                blocked,
                "the os_sync tier is available, so a real block must run"
            );
            assert!(
                waited < CEILING,
                "a ring must WAKE the kernel block, not let it time out - waited {waited:?} \
                 against a {CAP:?} cap"
            );
            assert!(
                waited >= RING_AT,
                "anti-vacuity: the block must really have waited for the ring, not \
                 returned before it was sent - waited {waited:?}"
            );
        } else {
            assert!(
                !blocked,
                "with no os_sync tier the wait must report that no block ran, so \
                 the caller naps instead of counting a wait it never got"
            );
        }

        drop(_guard);
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

        let snap = consumer.wake_seq_snapshot();
        // The ring lands between the snapshot and the block, with NOBODY parked
        // so no wake syscall is issued and the compare is the only defence.
        owner.ring();

        let t0 = std::time::Instant::now();
        let blocked = consumer.park_wait_ring(snap, CAP);
        let waited = t0.elapsed();

        // Both arms are judged, as in the wake test above.
        assert_eq!(
            blocked,
            wake_word_block_primitive_available(),
            "the call reports a wait exactly when the tier is there to run one"
        );
        assert!(
            waited < CEILING,
            "a ring since the snapshot must fail the kernel compare and return \
             immediately - waited {waited:?} against a {CAP:?} cap"
        );

        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }

    /// A zero cap blocks for nothing rather than risking an EINVAL, and reports
    /// that no block ran so the caller takes its nap.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_zero_cap_runs_no_block() {
        let ns = test_ns("zero_cap");
        let topic = "scan";
        let owner = Doorbell::open_owned(&ns, topic).expect("owner open_owned");
        assert!(!owner.park_wait_ring(owner.wake_seq_snapshot(), Duration::ZERO));
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
        let epoch0 = consumer.wake_seq_snapshot();
        owner.ring();
        assert_ne!(consumer.seq(), seq0, "the ring counter must advance");
        assert_ne!(
            consumer.wake_seq_snapshot(),
            epoch0,
            "the wake epoch must advance, or a parked peer's kernel compare \
             would succeed and it would sleep through the ring"
        );

        drop(consumer);
        drop(owner);
        cleanup(&ns, &[topic]);
    }
}

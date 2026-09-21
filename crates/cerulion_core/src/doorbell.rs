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
//! The SHM object name is `/cer_db_<ns>_<fnv1a64(topic):016x>`, where `<ns>` is
//! a caller-supplied namespace (a graph name or `$USER`). The topic is FNV-1a-64
//! hashed (a fixed-length, filesystem-safe token); the literal `<ns>` prefix
//! partitions the name space so two tenants never collide on a topic-hash even
//! if their FNV-64 hashes were to clash. The pure derivation lives in
//! `doorbell_shm_name`, hermetically testable on any OS.
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
//! A producer crash leaves an orphan `/dev/shm/cer_db_*` object (bounded: 64 B
//! per topic). This is self-healing: the next producer's `O_CREAT` REUSES the
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
//! exit: [`Doorbell::open_unowned`] `O_CREAT`s the `/dev/shm/cer_db_*` object but
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
//! - `linux`: real POSIX named SHM (`shm_open`/`mmap`/`shm_unlink` via `libc`).
//! - everything else (incl. macOS): a no-op stub over a boxed scratch word so
//!   the code COMPILES on macOS. `ring()`/`seq()` are no-ops;
//!   `addr()` returns a stable scratch address (so
//!   [`crate::monitor_wait`]'s no-op fallback has a valid pointer to ignore).

use std::io;
use std::sync::atomic::AtomicU64;

/// Derive the POSIX SHM object name for `(ns, topic)`:
/// `/cer_db_<ns>_<fnv1a64(topic):016x>`.
///
/// Pure (no I/O), so it is hermetically testable on every OS. The `<ns>` prefix
/// is literal (partitions tenants); the topic is FNV-1a-64 hashed into a
/// fixed-width, filesystem-safe token. Stable across processes, so the producer
/// and consumer of the same `(ns, topic)` derive the same name → map the same
/// page. This is also the registry's dedup key.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn doorbell_shm_name(ns: &str, topic: &str) -> String {
    // FNV-1a 64-bit: offset basis 0xcbf29ce484222325, prime 0x100000001b3.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in topic.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("/cer_db_{ns}_{h:016x}")
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

#[cfg(not(target_os = "linux"))]
mod imp {
    use std::io;
    use std::sync::atomic::AtomicU64;

    /// No-op doorbell stub (incl. macOS) so the code COMPILES off Linux.
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
    /// Returns the first open error (Linux) — on the non-Linux stub every open
    /// succeeds.
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

    /// Unique namespace per test run (pid-scoped) so concurrent test binaries
    /// and re-runs never collide on a `/dev/shm` object.
    fn test_ns(tag: &str) -> String {
        format!("doorbell_{}_{tag}", std::process::id())
    }

    /// On Linux, `shm_unlink` every `(ns, topic)` an unowned-registry test
    /// created (unowned doorbells never unlink themselves). No-op off Linux.
    #[cfg(target_os = "linux")]
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
    #[cfg(not(target_os = "linux"))]
    fn cleanup(_ns: &str, _topics: &[&str]) {}

    // ---- PURE name derivation (all OS) ----

    #[test]
    fn name_is_deterministic_and_has_ns_prefix() {
        let n1 = doorbell_shm_name("g", "a");
        let n2 = doorbell_shm_name("g", "a");
        assert_eq!(n1, n2, "same (ns,topic) → identical name (the dedup key)");
        assert!(
            n1.starts_with("/cer_db_g_"),
            "ns appears as a literal prefix: {n1}"
        );
    }

    #[test]
    fn name_oracle_fixed_fnv() {
        // FNV-1a-64 hand-computed oracle (independent Python reference):
        //   fnv1a64("a")     = 0xaf63dc4c8601ec8c
        //   fnv1a64("b")     = 0xaf63df4c8601f1a5
        //   fnv1a64("topic") = 0x520c8b7d6934ac64
        assert_eq!(doorbell_shm_name("g", "a"), "/cer_db_g_af63dc4c8601ec8c");
        assert_eq!(doorbell_shm_name("g", "b"), "/cer_db_g_af63df4c8601f1a5");
        assert_eq!(
            doorbell_shm_name("g", "topic"),
            "/cer_db_g_520c8b7d6934ac64"
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

        // An EQUAL snapshot → no advance. On the macOS stub seq()≡0 so this holds
        // trivially; on Linux it's the real no-ring case.
        assert!(
            !reg.any_advanced_since(&reg.snapshot_all()),
            "an equal snapshot must report NO advance"
        );

        drop(reg);
        cleanup(&ns, &["a", "b"]);
    }

    /// Linux-only: the ONLY place the ring→advance path is exercised end-to-end
    /// (it is inert on the macOS stub where seq()≡0). An OWNED producer doorbell
    /// and an `open_unowned` registry on the SAME `(ns, topic)` map the same page,
    /// so a producer `ring()` is observed by the consumer registry's poll-all.
    #[cfg(target_os = "linux")]
    #[test]
    fn linux_any_advanced_since_detects_a_ring() {
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

    // ---- Linux lifecycle (real SHM): create → ring → observe → drop-unlinks ----

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_owner_rings_consumer_observes_and_owner_drop_unlinks() {
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

    // ---- non-Linux stub: open/ring/seq/drop are no-ops that never panic ----

    #[cfg(not(target_os = "linux"))]
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
}

//! SHM-safety startup guard (detection + warn half).
//!
//! # Why this exists
//!
//! iceoryx2 `Static` SHM pools are lazy / demand-paged, so Cerulion oversizes
//! payload tiers freely — the large virtual reservation costs nothing until a
//! page is actually touched (resident memory tracks the working set, not the
//! reservation). Two OPERATOR-SET Linux configs break that assumption:
//!
//! 1. **Transparent Huge Pages for shmem**
//!    (`/sys/kernel/mm/transparent_hugepage/shmem_enabled` != `never`): the
//!    kernel may fault in whole 2MiB huge pages, inflating resident memory
//!    (RSS) by up to ~512x on our pools.
//! 2. **A bounded `RLIMIT_AS`** (address-space soft limit, e.g. `ulimit -v`):
//!    our oversized `mmap` reservation may fail with `ENOMEM` even though
//!    physical RAM is free, because `RLIMIT_AS` caps VIRTUAL address space.
//!
//! We also surface a soft warning when the process approaches
//! `vm.max_map_count` (each iceoryx2 SHM segment consumes VMA map entries).
//!
//! # Contract: loud inference, never mutate, never abort
//!
//! This guard DETECTS these conditions and emits a loud [`tracing::warn!`],
//! then PROCEEDS. It NEVER writes a system setting and NEVER aborts — the
//! operator stays in control; we merely make the footgun visible (monitor RSS,
//! not VSZ). It is Linux-only: macOS has no user-shm THP and this is a no-op
//! there.
//!
//! # The `madvise` MITIGATION
//!
//! Beyond detect-and-warn, this module now also MITIGATES the huge-page
//! hazard directly via [`advise_shm_pools_no_hugepage`]: it scans
//! `/proc/self/maps` for our own iceoryx2 SHM pool VMAs (pathname contains the
//! ACTIVE config's `global.prefix` — `iox2_` by default (including a
//! multi-process deployment's DATA plane), or a `cer_p_{hex}`-style
//! custom prefix such as the supervisor's planning namespace; option 2, the
//! VMA-scan approach) and calls
//! `madvise(addr, len, MADV_NOHUGEPAGE)` on each. This sets the per-VMA
//! `VM_NOHUGEPAGE` flag, which the kernel's tmpfs/shmem huge-page path honors,
//! reliably preventing a small write from faulting in a whole 2MiB huge page.
//! It is surgical (touches ONLY our pool mappings, never a machine-wide setting),
//! non-destructive (advice only — no unmap, no data mutation), and applied at
//! publisher AND subscriber creation, once the pool SHM is mapped. On non-Linux
//! targets it is a no-op (no user-shm THP).

// The pure decision helpers + verdict enums below are consumed by the
// Linux-only warn/advise layer AND by the unit tests. On a non-Linux build
// without `cfg(test)` (e.g. `cargo build` on macOS) they have no caller, so
// rather than exist-but-`allow(dead_code)` (which masks a genuine dead-code
// regression on that cell), we compile them OUT entirely there via a
// per-item `#[cfg(any(test, target_os = "linux"))]`. Linux builds and all
// test builds still enforce `dead_code = "deny"` on them.

/// Verdict for the shmem Transparent-Huge-Pages policy.
#[cfg(any(test, target_os = "linux"))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum ThpVerdict {
    /// Active policy is `never` — safe for our oversized lazy pools.
    Safe,
    /// Active policy is anything other than `never` (or unreadable/unknown) —
    /// huge pages may inflate resident memory. Carries the active token for
    /// the warn.
    Unsafe { active: String },
}

/// Decide the THP verdict from the raw contents of
/// `/sys/kernel/mm/transparent_hugepage/shmem_enabled`.
///
/// The sysfs file is a space-separated list of policies with the ACTIVE one in
/// brackets, e.g. `"always within_size advise [never] deny"`. We extract the
/// `[bracketed]` token: `never` is [`ThpVerdict::Safe`]; ANY other active
/// token — `always`, `within_size`, `advise`, `force`, `deny`, an unknown
/// token, or NO bracket at all — is [`ThpVerdict::Unsafe`].
///
/// NOTE: `within_size` is UNSAFE for us. We `ftruncate` pools to their full
/// length so the shmem inode's `i_size` is large, and `within_size` grants
/// huge pages up to `i_size` — behaving like `always` for our pools.
#[cfg(any(test, target_os = "linux"))]
pub(crate) fn thp_verdict(shmem_enabled_contents: &str) -> ThpVerdict {
    let trimmed = shmem_enabled_contents.trim();
    // Pull the token between the first '[' and the next ']'.
    let bracketed = trimmed
        .split('[')
        .nth(1)
        .and_then(|after| after.split(']').next())
        .map(str::trim);
    match bracketed {
        Some("never") => ThpVerdict::Safe,
        Some(token) if !token.is_empty() => ThpVerdict::Unsafe {
            active: token.to_string(),
        },
        // No bracketed token (or an empty one): `never` cannot be confirmed, so
        // treat it as unsafe and surface whatever the file held (or a marker
        // when it was empty) so the operator can see what we read.
        _ => ThpVerdict::Unsafe {
            active: if trimmed.is_empty() {
                "unknown".to_string()
            } else {
                trimmed.to_string()
            },
        },
    }
}

/// Verdict for the process's `RLIMIT_AS` (address-space) soft limit.
#[cfg(any(test, target_os = "linux"))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RlimitVerdict {
    /// Soft limit is `RLIM_INFINITY` — no virtual-address ceiling, safe.
    Unbounded,
    /// Soft limit is finite — an oversized reservation may `mmap`-fail.
    Bounded { soft_bytes: u64 },
}

/// Decide the `RLIMIT_AS` verdict from the soft limit and the platform's
/// `RLIM_INFINITY` sentinel. `soft == infinity` is [`RlimitVerdict::Unbounded`];
/// anything smaller is [`RlimitVerdict::Bounded`].
#[cfg(any(test, target_os = "linux"))]
pub(crate) fn rlimit_as_verdict(soft: u64, infinity: u64) -> RlimitVerdict {
    if soft == infinity {
        RlimitVerdict::Unbounded
    } else {
        RlimitVerdict::Bounded { soft_bytes: soft }
    }
}

/// Verdict for how close the process is to `vm.max_map_count`.
#[cfg(any(test, target_os = "linux"))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MapCountVerdict {
    /// Comfortably below the map-count ceiling.
    Ok,
    /// At or above 80% of the ceiling — a many-topic graph may exhaust it.
    Approaching { current: usize, max: usize },
}

/// Decide the map-count verdict: [`MapCountVerdict::Approaching`] once the
/// current mapping count is at least 80% of `max` (`current * 5 >= max * 4`),
/// otherwise [`MapCountVerdict::Ok`]. A `max` of 0 (unreadable / degenerate)
/// yields [`MapCountVerdict::Ok`] — no warning fires on a meaningless ceiling.
#[cfg(any(test, target_os = "linux"))]
pub(crate) fn map_count_verdict(current: usize, max: usize) -> MapCountVerdict {
    if max == 0 {
        return MapCountVerdict::Ok;
    }
    // >= 80% without floating point: current/max >= 4/5  <=>  current*5 >= max*4.
    if current * 5 >= max * 4 {
        MapCountVerdict::Approaching { current, max }
    } else {
        MapCountVerdict::Ok
    }
}

/// Read the operator-set Linux configs that break our
/// oversize-freely assumption and emit a loud [`tracing::warn!`] for each
/// unsafe one. Detect-and-warn only — never mutates a system setting, never
/// aborts. Called ONCE at transport init (cold path).
#[cfg(target_os = "linux")]
pub(crate) fn check_and_warn_shm_safety() {
    warn_if_thp_unsafe();
    warn_if_rlimit_as_bounded();
    warn_if_map_count_approaching();
}

/// Non-Linux no-op: macOS (and other targets) have no user-shm THP and no
/// equivalent oversize-reservation hazard, so there is nothing to check.
#[cfg(not(target_os = "linux"))]
pub(crate) fn check_and_warn_shm_safety() {}

#[cfg(target_os = "linux")]
fn warn_if_thp_unsafe() {
    const THP_PATH: &str = "/sys/kernel/mm/transparent_hugepage/shmem_enabled";
    // Absent/unreadable file => a kernel without shmem THP support; nothing to
    // warn about, so skip silently.
    let Ok(contents) = std::fs::read_to_string(THP_PATH) else {
        return;
    };
    let ThpVerdict::Unsafe { active } = thp_verdict(&contents) else {
        return;
    };
    tracing::warn!(
        active = %active,
        config = THP_PATH,
        "Transparent Huge Pages for shmem is enabled (active policy is not \
         `never`): Cerulion oversizes iceoryx2 Static SHM pools assuming \
         lazy / demand-paged backing, but THP can fault in whole 2MiB huge \
         pages — up to ~512x resident (RSS) inflation on our pools. Fix: \
         `echo never > /sys/kernel/mm/transparent_hugepage/shmem_enabled` \
         (or mount the shm tmpfs with `huge=never`). Monitor RSS, not VSZ — \
         the large virtual reservation is expected and harmless."
    );
}

#[cfg(target_os = "linux")]
fn warn_if_rlimit_as_bounded() {
    let mut rl = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `getrlimit` only reads the RLIMIT_AS resource id and writes the
    // current/max limits into `rl`, which we fully own here — no aliasing and
    // no other effects. A non-zero return means we couldn't read the limit.
    if unsafe { libc::getrlimit(libc::RLIMIT_AS, &mut rl) } != 0 {
        return;
    }
    let RlimitVerdict::Bounded { soft_bytes } = rlimit_as_verdict(rl.rlim_cur, libc::RLIM_INFINITY)
    else {
        return;
    };
    tracing::warn!(
        soft_bytes,
        "RLIMIT_AS (address-space) soft limit is bounded: Cerulion oversizes \
         iceoryx2 Static SHM pools (large virtual reservations that stay \
         lazily backed), so an oversized mmap may fail with ENOMEM under this \
         limit even though physical RAM is free. Prefer a cgroup `memory.max` \
         (which bounds RESIDENT memory) over `ulimit -v` (`RLIMIT_AS`, which \
         bounds VIRTUAL address space)."
    );
}

#[cfg(target_os = "linux")]
fn warn_if_map_count_approaching() {
    // Both reads must succeed; either failing => skip (kernel without the knob,
    // or /proc unavailable in a restricted sandbox).
    let Ok(max_raw) = std::fs::read_to_string("/proc/sys/vm/max_map_count") else {
        return;
    };
    let Ok(maps) = std::fs::read_to_string("/proc/self/maps") else {
        return;
    };
    let Ok(max) = max_raw.trim().parse::<usize>() else {
        return;
    };
    let current = maps.lines().count();
    let MapCountVerdict::Approaching { current, max } = map_count_verdict(current, max) else {
        return;
    };
    tracing::warn!(
        current,
        max,
        config = "/proc/sys/vm/max_map_count",
        "this process is within 80% of vm.max_map_count: each iceoryx2 SHM \
         segment consumes VMA map entries, so a many-topic graph can exhaust \
         the limit and fail further mmaps. Raise it with \
         `sysctl -w vm.max_map_count=<larger>` if node/topic growth continues."
    );
}

/// Parse the VMA start/end ranges of every SHARED mapping in a
/// `/proc/self/maps` blob whose PATHNAME contains `marker`.
///
/// Each `/proc/self/maps` line has the shape
/// `START-END perms offset dev inode   pathname`, where `START` and `END` are
/// lowercase hex addresses and the leading five whitespace-separated columns
/// are fixed; everything after the inode column is the pathname (which itself
/// may contain spaces). We return a range ONLY when the mapping is SHARED —
/// the `perms` token's last char is `'s'` (e.g. `rw-s`/`r--s`) — AND its
/// pathname CONTAINS `marker`; then we decode `START` and `END` via
/// [`usize::from_str_radix`] (base 16) and push `(start, end)`. PRIVATE
/// mappings (perms ending in `'p'`) are excluded even when their pathname
/// contains the marker — this deliberately skips the process's OWN executable,
/// whose path may embed the marker (e.g. a test binary named
/// `…/deps/…iox2…`), which `/proc/self/maps` always lists as a private
/// mapping. Our real iceoryx2 SHM pool segments are `MAP_SHARED`, so scoping to
/// shared perms is what makes a bare marker match name a genuine pool.
///
/// Lines are skipped when they have no `perms` column, when the mapping is not
/// shared, when they have no pathname column (fewer than six fields — e.g. an
/// anonymous mapping, whose absent pathname can't contain the marker), when
/// their pathname does not contain the marker, or when the `START-END` range is
/// missing its `-` or holds unparseable hex.
///
/// Pure and platform-independent (unit-tested on macOS): the caller supplies
/// the maps contents, so this does no I/O.
#[cfg(any(test, target_os = "linux"))]
pub(crate) fn shm_vma_ranges(maps_contents: &str, marker: &str) -> Vec<(usize, usize)> {
    let mut ranges = Vec::new();
    for line in maps_contents.lines() {
        // Columns: `START-END perms offset dev inode   pathname`. Split on
        // ASCII whitespace (tolerating the alignment space-runs before the
        // pathname): field 0 is the range, field 1 is perms, fields 2..=4 are
        // the fixed offset/dev/inode columns, and everything from field 5 on is
        // the pathname (which may itself contain spaces).
        let mut fields = line.split_whitespace();
        let Some(range) = fields.next() else {
            continue;
        };
        // Require a SHARED mapping: the perms token ends in `'s'` (private maps
        // end in `'p'`). Our iceoryx2 SHM pool segments are `MAP_SHARED`; the
        // process's own executable (whose path may embed the marker) is private,
        // so this gate excludes it. Absent perms or a non-shared mapping => skip.
        let Some(perms) = fields.next() else {
            continue;
        };
        if !perms.ends_with('s') {
            continue;
        }
        // Consume offset, dev, inode (3 fixed columns). Any remaining fields are
        // the pathname tokens; a line with fewer than 6 fields has no pathname
        // (e.g. an anonymous mapping) so `any` sees nothing and we skip. The
        // `iox2_` marker never straddles a space, so a per-token `contains`
        // faithfully answers "does the pathname contain marker?".
        if fields.by_ref().take(3).count() < 3 {
            continue;
        }
        if !fields.any(|tok| tok.contains(marker)) {
            continue;
        }
        let Some((start_hex, end_hex)) = range.split_once('-') else {
            continue;
        };
        let (Ok(start), Ok(end)) = (
            usize::from_str_radix(start_hex, 16),
            usize::from_str_radix(end_hex, 16),
        ) else {
            continue;
        };
        // A real VMA always has start < end. Skip a degenerate/inverted range
        // (malformed line, or a raced remap) — this also removes any
        // `end - start` underflow risk in the downstream madvise length.
        if start >= end {
            continue;
        }
        ranges.push((start, end));
    }
    ranges
}

/// Which create hook is invoking [`advise_shm_pools_no_hugepage`]. The two sites
/// differ in exactly ONE way that matters to the scan-anomaly guard: a publisher
/// allocates and maps its OWN data pool eagerly at creation, so finding zero
/// `iox2_` pool VMAs there means the scan is broken; a subscriber maps a
/// producer's pool LAZILY — an already-live producer during
/// `force_update_connections`, otherwise on the first `receive` — so an empty
/// scan at subscriber creation is legitimate (e.g. subscriber-before-publisher),
/// NOT an anomaly. Constructed at both call sites on every platform (the
/// non-Linux no-op ignores it), so it carries no `#[cfg]`.
#[derive(Debug, Clone, Copy)]
pub(crate) enum PoolAdviseSite {
    /// Publisher create hook: this process's own pool was just mapped, so an
    /// empty `iox2_` scan is anomalous (a broken scan).
    Publisher,
    /// Subscriber create hook: producer pools map lazily, so an empty scan is a
    /// legitimate pre-publisher state, never an anomaly.
    Subscriber,
}

/// The `/proc/self/maps` pathname marker for OUR iceoryx2 SHM segments: the
/// ACTIVE config's `global.prefix` (every iceoryx2 on-disk/SHM object name
/// starts with it), falling back to the iceoryx2 default `iox2_` when the
/// caller hands an empty prefix. A HARDCODED default marker would make
/// any custom-prefix config (such as the multi-process
/// supervisor's `cer_p_{hex}` planning namespace) scan for the wrong name —
/// the THP mitigation would silently protect NOTHING and the publisher-site anomaly
/// warn would fire on every run.
#[cfg(any(test, target_os = "linux"))]
pub(crate) fn scan_marker(active_prefix: &str) -> &str {
    if active_prefix.is_empty() {
        "iox2_"
    } else {
        active_prefix
    }
}

/// Apply `MADV_NOHUGEPAGE` to our iceoryx2 SHM
/// pool VMAs so a small write cannot fault in a whole 2MiB huge page (up to
/// ~512x resident inflation on our oversized pools). Scans the CURRENT
/// `/proc/self/maps` for mappings whose pathname carries the ACTIVE config's
/// segment-name prefix (`shm_prefix`, via [`scan_marker`]) and advises each.
/// Best-effort: an unreadable maps file returns
/// silently; a non-zero `madvise` return (e.g. `EINVAL` on an already-unmapped
/// or invalid range) is ignored. Called at publisher AND subscriber creation,
/// after the pool SHM is (potentially) mapped (cold path).
///
/// A one-shot [`tracing::warn!`] fires (at most once per process) if the
/// mitigation anomalously found no pool at a site that DEFINITELY just mapped
/// one — i.e. only the [`PoolAdviseSite::Publisher`] site; see
/// [`should_warn_scan_anomaly`].
#[cfg(target_os = "linux")]
static SHM_MITIGATION_ANOMALY_WARNED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Pure scan-anomaly decision (split out for hermetic oracle-vector testing):
/// warn ONLY when a pool was DEFINITELY just mapped by this hook (the publisher
/// site) yet the scan found none — an unambiguously broken scan (most likely an
/// iceoryx2 `iox2_` segment-name prefix change). An empty scan at the subscriber
/// site is legitimate (lazy producer-pool mapping), so it must NEVER warn there
/// — otherwise a subscriber-before-publisher startup would spuriously fire AND
/// permanently consume the one-shot token, masking any genuine future
/// scan-breakage alert.
#[cfg(target_os = "linux")]
fn should_warn_scan_anomaly(site: PoolAdviseSite, found: usize) -> bool {
    matches!(site, PoolAdviseSite::Publisher) && found == 0
}

#[cfg(target_os = "linux")]
pub(crate) fn advise_shm_pools_no_hugepage(site: PoolAdviseSite, shm_prefix: &str) {
    let Ok(maps) = std::fs::read_to_string("/proc/self/maps") else {
        return;
    };
    let marker = scan_marker(shm_prefix);
    let ranges = shm_vma_ranges(&maps, marker);
    let mut advised = 0usize;
    for (start, end) in &ranges {
        // SAFETY: the ranges come from our OWN current `/proc/self/maps`, so
        // each is a live mapping this process owns. `MADV_NOHUGEPAGE` is
        // non-destructive advice — it neither unmaps nor mutates any byte, it
        // only sets the per-VMA `VM_NOHUGEPAGE` flag. A stale/invalid range
        // (raced unmap) yields `EINVAL`, which we ignore.
        let ret = unsafe {
            libc::madvise(
                *start as *mut libc::c_void,
                end - start,
                libc::MADV_NOHUGEPAGE,
            )
        };
        if ret == 0 {
            advised += 1;
        }
        // The SAME VMA is also excluded from `fork` inheritance.
        // This runs at the BIRTH hook rather than in an arm-time sweep because
        // iceoryx2 mappings appear continuously AFTER arm — bagd's own taps, its
        // 250 ms discovery rescan, vizd / `topic echo` demands — so a one-shot sweep
        // would leave unmarked exactly the segments the recorder's own activity
        // creates. A capture child's stray read of a live pool would otherwise be torn,
        // concurrently-mutated bytes written into the bag as a point-in-time image;
        // excluded, it is a SIGSEGV in a disposable child.
        //
        // SAFETY: the range comes from our OWN current `/proc/self/maps`, so it is a
        // live mapping this process owns.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                *start as *mut libc::c_void,
                end - start,
                crate::state_carrier::ForkExcludedMapping::IceoryxPool,
            );
        }
    }
    // Warn-once anomaly guard: at the PUBLISHER site a pool was JUST mapped by
    // this create hook, so finding ZERO shared marker-matching VMAs
    // (`ranges.is_empty()`) means our scan is broken — most likely iceoryx2
    // changed its segment-name scheme. At the SUBSCRIBER site an empty scan is legitimate
    // (producer pools map lazily — subscriber-before-publisher maps nothing
    // until the first `receive`), so `should_warn_scan_anomaly` never warns
    // there; otherwise a normal subscriber-first startup would spuriously fire
    // AND burn the one-shot token, masking a genuine future scan breakage.
    //
    // We also deliberately do NOT warn on a partial shortfall (`advised <
    // ranges.len()`): that is AMBIGUOUS. On a `CONFIG_TRANSPARENT_HUGEPAGE=n`
    // kernel `MADV_NOHUGEPAGE` returns `EINVAL` for EVERY call, so `advised == 0
    // < found` with ZERO real risk (no THP means no inflation to prevent) — a
    // partial-shortfall warn would be a false positive there. The found-zero
    // case at the publisher site, by contrast, is unambiguously a broken scan.
    if should_warn_scan_anomaly(site, ranges.len()) {
        // The token-consuming `swap` is NESTED inside the `should_warn` gate
        // (deliberately NOT `&&`-chained with it) so it is STRUCTURALLY
        // impossible for a subscriber-site or non-empty-scan call to consume the
        // one-shot `SHM_MITIGATION_ANOMALY_WARNED` token. A later refactor cannot
        // silently downgrade a `&&` short-circuit into a `&` bitand (which
        // `clippy` has no stable lint for) and start burning the token — masking
        // every future genuine broken-scan warn — on each subscriber-first
        // startup: here the `swap` is simply unreachable unless `should_warn` is
        // already true.
        if !SHM_MITIGATION_ANOMALY_WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
            tracing::warn!(
                found = ranges.len(),
                advised,
                marker = %marker,
                "MADV_NOHUGEPAGE mitigation found NO shared iceoryx2 SHM pool \
                 VMAs matching the active segment-name prefix even though a \
                 publisher pool was just mapped by this create hook — our \
                 /proc/self/maps scan is broken, most likely an iceoryx2 \
                 segment-name scheme change. Oversized pools may not be \
                 protected from THP resident inflation. The detect+warn \
                 THP/RLIMIT_AS half is unaffected; monitor RSS."
            );
        }
    }
    tracing::debug!(
        advised,
        found = ranges.len(),
        marker = %marker,
        "applied MADV_NOHUGEPAGE to iceoryx2 SHM pool VMAs"
    );
}

/// Non-Linux no-op: no user-shm Transparent Huge Pages, so there is nothing to
/// advise against. Takes [`PoolAdviseSite`] + the active prefix to keep the
/// call sites uniform across platforms; both are irrelevant off Linux.
#[cfg(not(target_os = "linux"))]
pub(crate) fn advise_shm_pools_no_hugepage(_site: PoolAdviseSite, _shm_prefix: &str) {}

#[cfg(test)]
mod tests {
    use super::*;

    // --- should_warn_scan_anomaly: oracle vectors ---------------------------
    // The load-bearing contract: an empty
    // `iox2_` scan is a broken-scan anomaly ONLY at the publisher site (its pool
    // was just mapped). At the subscriber site an empty scan is a legitimate
    // pre-publisher state (producer pools map lazily), so it must NEVER warn —
    // else a subscriber-first startup would spuriously fire AND permanently burn
    // the one-shot `SHM_MITIGATION_ANOMALY_WARNED` token, masking genuine future
    // scan breakage. Mutation check: dropping the `Publisher` gate (warn on any
    // empty scan) flips the `(Subscriber, 0)` case to `true` and this fails.
    #[cfg(target_os = "linux")]
    #[test]
    fn scan_anomaly_warns_only_for_publisher_empty_scan() {
        // (site, found_pool_vmas) -> should the one-shot anomaly warn fire?
        let cases = [
            (PoolAdviseSite::Publisher, 0usize, true), // just mapped, none found => broken scan
            (PoolAdviseSite::Publisher, 3, false),     // mapped and found => healthy
            (PoolAdviseSite::Subscriber, 0, false),    // lazy producer pool: empty is LEGIT
            (PoolAdviseSite::Subscriber, 5, false),    // live producer pools mapped => healthy
        ];
        for (site, found, want) in cases {
            assert_eq!(
                should_warn_scan_anomaly(site, found),
                want,
                "site={site:?} found={found}"
            );
        }
    }

    // --- thp_verdict: oracle vectors over the sysfs bracket format ----------

    #[test]
    fn thp_never_is_safe() {
        assert_eq!(
            thp_verdict("always within_size advise [never] deny"),
            ThpVerdict::Safe
        );
    }

    #[test]
    fn thp_always_is_unsafe_with_active_token() {
        assert_eq!(
            thp_verdict("[always] within_size advise never deny"),
            ThpVerdict::Unsafe {
                active: "always".to_string()
            }
        );
    }

    #[test]
    fn thp_within_size_is_unsafe() {
        // CRITICAL FOOTGUN: `within_size` looks conservative but is UNSAFE for
        // us — we ftruncate pools to full length (large i_size), and
        // `within_size` grants huge pages up to i_size, behaving like
        // `always` for our pools.
        assert_eq!(
            thp_verdict("always [within_size] advise never deny"),
            ThpVerdict::Unsafe {
                active: "within_size".to_string()
            }
        );
    }

    #[test]
    fn thp_advise_is_unsafe() {
        assert_eq!(
            thp_verdict("always within_size [advise] never deny"),
            ThpVerdict::Unsafe {
                active: "advise".to_string()
            }
        );
    }

    #[test]
    fn thp_no_bracket_is_unsafe() {
        // A file with no active-policy bracket: we can't confirm `never`, so
        // it's unsafe and we surface what we read.
        assert_eq!(
            thp_verdict("always within_size advise never deny"),
            ThpVerdict::Unsafe {
                active: "always within_size advise never deny".to_string()
            }
        );
    }

    #[test]
    fn thp_trailing_newline_never_is_safe() {
        // Real sysfs reads carry a trailing newline; it must be trimmed.
        assert_eq!(thp_verdict("always [never] deny\n"), ThpVerdict::Safe);
    }

    #[test]
    fn thp_empty_is_unsafe_unknown() {
        assert_eq!(
            thp_verdict(""),
            ThpVerdict::Unsafe {
                active: "unknown".to_string()
            }
        );
    }

    // --- rlimit_as_verdict --------------------------------------------------

    #[test]
    fn rlimit_soft_equals_infinity_is_unbounded() {
        assert_eq!(
            rlimit_as_verdict(u64::MAX, u64::MAX),
            RlimitVerdict::Unbounded
        );
    }

    #[test]
    fn rlimit_soft_below_infinity_is_bounded_with_exact_value() {
        assert_eq!(
            rlimit_as_verdict(2 * 1024 * 1024 * 1024, u64::MAX),
            RlimitVerdict::Bounded {
                soft_bytes: 2 * 1024 * 1024 * 1024
            }
        );
    }

    // --- map_count_verdict: 80% boundary ------------------------------------

    #[test]
    fn map_count_just_under_80_percent_is_ok() {
        // 7999/10000 = 79.99% -> Ok.
        assert_eq!(map_count_verdict(7999, 10_000), MapCountVerdict::Ok);
    }

    #[test]
    fn map_count_exactly_80_percent_is_approaching() {
        // 8000/10000 = 80.0% -> Approaching.
        assert_eq!(
            map_count_verdict(8000, 10_000),
            MapCountVerdict::Approaching {
                current: 8000,
                max: 10_000
            }
        );
    }

    #[test]
    fn map_count_over_80_percent_is_approaching() {
        assert_eq!(
            map_count_verdict(9500, 10_000),
            MapCountVerdict::Approaching {
                current: 9500,
                max: 10_000
            }
        );
    }

    #[test]
    fn map_count_zero_max_is_ok() {
        // Degenerate/unreadable ceiling never warns.
        assert_eq!(map_count_verdict(42, 0), MapCountVerdict::Ok);
    }

    // --- shm_vma_ranges: oracle vectors over the /proc/self/maps format -----

    // A realistic multi-line maps blob: two iceoryx2 pool lines (both SHARED,
    // `rw-s` — our real SHM pools are `MAP_SHARED`) under DIFFERENT directories
    // (proving config-independence — production `/dev/shm/...` and an isolated
    // test-root `/custom/root/...`), plus `[heap]`, `[stack]`, an anonymous
    // mapping (no pathname column), and a non-iox2 `/dev/shm/other`. Only the
    // two shared `iox2_` lines must be returned, with hex-decoded ranges.
    const MAPS_SAMPLE: &str = "\
7f0000001000-7f0000003000 rw-s 00000000 00:1f 12345    /dev/shm/iox2_svc_a
55a000000000-55a000021000 rw-p 00000000 00:00 0                          [heap]
7f0000005000-7f000000a000 rw-s 00000000 00:1f 67890    /custom/root/iox2_svc_b
7ffdd0000000-7ffdd0021000 rw-p 00000000 00:00 0
7f0000100000-7f0000110000 rw-s 00000000 00:1f 99999    /dev/shm/other
7ffff7ffd000-7ffff8000000 rw-p 00000000 00:00 0                          [stack]
";

    #[test]
    fn shm_vma_ranges_returns_exactly_the_iox2_lines() {
        assert_eq!(
            shm_vma_ranges(MAPS_SAMPLE, "iox2_"),
            vec![
                (0x7f0000001000, 0x7f0000003000),
                (0x7f0000005000, 0x7f000000a000),
            ]
        );
    }

    #[test]
    fn shm_vma_ranges_marker_absent_is_empty() {
        // No pathname contains this marker => nothing to advise.
        assert_eq!(
            shm_vma_ranges(MAPS_SAMPLE, "nonexistent_marker"),
            Vec::<(usize, usize)>::new()
        );
    }

    #[test]
    fn shm_vma_ranges_skips_malformed_hex_without_panicking() {
        // An iox2_ line whose START-END range holds non-hex characters must be
        // skipped, not panicked on. Both lines are SHARED (`rw-s`) so the bad
        // one reaches (and is rejected by) the hex-decode branch rather than the
        // shared gate. The well-formed sibling still parses.
        let maps = "\
zzzz-yyyy rw-s 00000000 00:1f 11111    /dev/shm/iox2_bad
7f0000005000-7f000000a000 rw-s 00000000 00:1f 67890    /dev/shm/iox2_good
";
        assert_eq!(
            shm_vma_ranges(maps, "iox2_"),
            vec![(0x7f0000005000, 0x7f000000a000)]
        );
    }

    #[test]
    fn shm_vma_ranges_marker_in_non_pathname_column_is_ignored() {
        // The `iox2_` token here is the PERMS column, not the pathname. It fails
        // the shared gate (`iox2_` does not end in `s`) AND the pathname
        // (`/real/path`) carries no marker, so matching — scoped to a SHARED
        // mapping whose pathname contains the marker — yields nothing.
        assert_eq!(
            shm_vma_ranges(
                "7f000000a000-7f000000b000 iox2_ 0 00:00 0 /real/path",
                "iox2_"
            ),
            Vec::<(usize, usize)>::new()
        );
    }

    #[test]
    fn shm_vma_ranges_range_missing_dash_is_skipped() {
        // Pathname matches `iox2_` and the mapping is SHARED (`rw-s`, so it
        // passes the shared gate), but the range column has no `-` separator —
        // `split_once('-')` fails, so the line is skipped at the DASH branch
        // (not the shared gate) rather than panicking.
        assert_eq!(
            shm_vma_ranges("7f0000001000 rw-s 0 00:1f 12345 /dev/shm/iox2_x", "iox2_"),
            Vec::<(usize, usize)>::new()
        );
    }

    #[test]
    fn private_iox2_mapping_is_skipped() {
        // The exact exe-contamination case: a PRIVATE mapping (`r-xp`, perms end
        // in `p`) whose PATHNAME contains `iox2_` — here the test binary's own
        // executable, which /proc/self/maps ALWAYS lists as private. Scoping to
        // SHARED mappings excludes it, so it yields nothing (a bare marker match
        // would wrongly return it and make the Linux smaps test tautological).
        assert_eq!(
            shm_vma_ranges(
                "7f000000a000-7f000000b000 r-xp 00000000 00:1f 999 \
                 /home/u/target/debug/deps/shm_guard_madvise_iox2_test-abc123",
                "iox2_"
            ),
            Vec::<(usize, usize)>::new()
        );
    }

    #[test]
    fn shm_vma_ranges_pathname_with_space_still_matches() {
        // A pathname containing a space (`iox2_a b`) still matches on the token
        // that carries the marker, and the range decodes normally.
        assert_eq!(
            shm_vma_ranges(
                "7f000000a000-7f000000b000 rw-s 0 00:1f 12345 /dev/shm/iox2_a b",
                "iox2_"
            ),
            vec![(0x7f000000a000, 0x7f000000b000)]
        );
    }

    #[test]
    fn shm_vma_ranges_empty_maps_is_empty() {
        assert_eq!(shm_vma_ranges("", "iox2_"), Vec::<(usize, usize)>::new());
    }

    // --- scan_marker: the custom-prefix regression -------------------

    /// The marker is the ACTIVE config prefix; the hardcoded-default fallback
    /// fires only on an empty prefix. Regression pin: under
    /// a custom multi-process prefix a hardcoded scan looks for the literal
    /// `iox2_` and protects nothing.
    #[test]
    fn scan_marker_uses_active_prefix_with_default_fallback() {
        assert_eq!(scan_marker(""), "iox2_", "empty prefix -> iceoryx2 default");
        assert_eq!(scan_marker("iox2_"), "iox2_");
        assert_eq!(
            scan_marker("cer_p_d4806b18fa57bd34"),
            "cer_p_d4806b18fa57bd34"
        );
    }

    /// End-to-end scan vector under a CUSTOM prefix: a maps blob whose shared
    /// SHM lines carry a `cer_p_{hex}` name (the planning namespace) is
    /// INVISIBLE to a hardcoded `iox2_` marker but found by the
    /// active-prefix marker.
    #[test]
    fn custom_prefix_segments_found_by_active_marker_not_by_default() {
        let maps = "\
7f0000001000-7f0000003000 rw-s 00000000 00:1f 12345    /dev/shm/cer_p_d4806b18fa57bd34_svc
55a000000000-55a000021000 rw-p 00000000 00:00 0                          [heap]
";
        let marker = scan_marker("cer_p_d4806b18fa57bd34");
        assert_eq!(
            shm_vma_ranges(maps, marker),
            vec![(0x7f0000001000, 0x7f0000003000)],
            "the active-prefix marker must find the custom-prefix pool"
        );
        assert_eq!(
            shm_vma_ranges(maps, "iox2_"),
            Vec::<(usize, usize)>::new(),
            "a hardcoded default marker misses it (the custom-prefix regression)"
        );
    }
}

//! SHM footprint: Linux-only proof that the MADV_NOHUGEPAGE
//! mitigation actually applies to our iceoryx2 SHM pool VMAs.
//!
//! The pure `shm_vma_ranges` parser and the detect-and-warn half are covered
//! by in-crate unit tests that run everywhere. THIS test closes the loop: it
//! creates a real, oversized iceoryx2 publisher (firing the publisher-create
//! hook `shm_guard::advise_shm_pools_no_hugepage`), then inspects
//! `/proc/self/smaps` and asserts every one of our `iox2_` pool mappings now
//! carries the `nh` (no-hugepage) VmFlag that `MADV_NOHUGEPAGE` sets.
//!
//! It requires a real Linux kernel with `/proc/self/smaps` (macOS has none),
//! so it is `#[ignore]`d and Linux-only. Run it on a Linux host with:
//!
//! ```bash
//! cargo test -p cerulion_core --test shm_guard_madvise_iox2_test \
//!   -- --ignored --nocapture --test-threads=1
//! ```

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use serial_test::serial;

/// One `/proc/self/smaps` mapping: whether its header names an iceoryx2 pool
/// (a SHARED mapping whose pathname contains `iox2_`), plus the raw `VmFlags:`
/// value line if present.
struct SmapsBlock {
    is_iox2: bool,
    vmflags: Option<String>,
}

/// Parse `/proc/self/smaps` into per-mapping blocks. A block starts with a
/// header line whose FIRST token is a `START-END` hex range (smaps `Key:`
/// lines never contain `-`), followed by `Key: value` detail lines including a
/// `VmFlags:` line. smaps headers share the `range perms offset dev inode
/// pathname` shape of `/proc/self/maps`. We only need, per block, whether the
/// header names one of OUR SHARED `iox2_` pools and what its `VmFlags:` line
/// says.
fn parse_smaps_blocks(smaps: &str) -> Vec<SmapsBlock> {
    let mut blocks: Vec<SmapsBlock> = Vec::new();
    for line in smaps.lines() {
        let is_header = line
            .split_whitespace()
            .next()
            .is_some_and(|first| first.contains('-'));
        if is_header {
            // Scope to SHARED mappings: the header's perms token (2nd
            // whitespace field, e.g. `rw-s`) ends in `'s'`. Our real iceoryx2
            // SHM pool segments are MAP_SHARED; the test binary's OWN executable
            // (whose path contains `iox2_`) is a PRIVATE (`..p`) mapping that
            // /proc/self/smaps always lists — without the shared scope it would
            // misclassify as a pool block and break the every-block-has-`nh`
            // assertion below.
            let is_shared = line
                .split_whitespace()
                .nth(1)
                .is_some_and(|perms| perms.ends_with('s'));
            blocks.push(SmapsBlock {
                is_iox2: line.contains("iox2_") && is_shared,
                vmflags: None,
            });
        } else if let Some(rest) = line.strip_prefix("VmFlags:") {
            if let Some(current) = blocks.last_mut() {
                current.vmflags = Some(rest.trim().to_string());
            }
        }
    }
    blocks
}

/// Returns true iff `vmflags` (a space-separated token list) contains the
/// exact `nh` token (not a substring of some other flag).
fn vmflags_has_nh(vmflags: &str) -> bool {
    vmflags.split_whitespace().any(|tok| tok == "nh")
}

#[test]
#[ignore = "box-only: needs a real Linux /proc/self/smaps"]
#[serial]
fn advise_sets_nh_flag_on_our_iox2_pool_vmas() {
    let manager = TransportManager::get_or_init().expect("transport init");

    // A UNIQUE, oversized (128 MiB) topic. Oversized so the pool reservation is
    // large enough to be a THP candidate; unique so this test's mappings do not
    // collide with a concurrently-run graph. Creating it fires the
    // publisher-create hook (`shm_guard::advise_shm_pools_no_hugepage` with
    // `PoolAdviseSite::Publisher`, which DOES warn on an empty scan). Bind
    // (underscore-prefixed) to keep the mapping alive across the smaps read.
    let topic = format!("/madvise_{}", std::process::id());
    let _publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(128 * 1024 * 1024))
        .expect("create oversized publisher (fires the madvise hook)");

    let smaps = std::fs::read_to_string("/proc/self/smaps")
        .expect("Linux-only test: /proc/self/smaps must be readable on Linux");
    let blocks = parse_smaps_blocks(&smaps);

    let iox2_blocks: Vec<&SmapsBlock> = blocks.iter().filter(|b| b.is_iox2).collect();

    // (a) Anti-tautology: the scan/hook actually found our pool mappings. If
    // this is empty the assertion below would vacuously pass, so guard it.
    assert!(
        !iox2_blocks.is_empty(),
        "expected at least one iox2_ SHM pool mapping in /proc/self/smaps after \
         creating an oversized publisher — the hook or the scan did not run"
    );

    // (b) EVERY iox2_ pool mapping carries the `nh` VmFlag, proving
    // MADV_NOHUGEPAGE was actually applied to each of our pool VMAs.
    for block in &iox2_blocks {
        let vmflags = block
            .vmflags
            .as_deref()
            .expect("every smaps mapping block has a VmFlags: line");
        assert!(
            vmflags_has_nh(vmflags),
            "an iox2_ pool mapping is missing the `nh` (no-hugepage) VmFlag \
             (VmFlags: {vmflags}) — MADV_NOHUGEPAGE did not apply"
        );
    }

    // (c) NEGATIVE CONTROL: `nh` is SELECTIVE — set by OUR per-VMA madvise, not
    // ambient. Assert that at least one NON-iox2 mapping that HAS a `VmFlags:`
    // line does NOT carry the `nh` token. On a host where EVERY mapping carries
    // `nh` (e.g. a process-wide THP-disable), the primary (b) assertion would
    // be tautological; this control proves our per-VMA MADV_NOHUGEPAGE is the
    // thing setting `nh` on our pools.
    assert!(
        blocks
            .iter()
            .any(|b| !b.is_iox2 && b.vmflags.as_deref().is_some_and(|f| !vmflags_has_nh(f))),
        "expected at least one non-iox2 mapping WITHOUT the `nh` VmFlag — if EVERY \
         mapping carries `nh` (e.g. the host runs a process-wide THP-disable) the \
         primary per-VMA assertion above would be tautological rather than proving \
         our madvise is load-bearing"
    );
}

/// CI counterpart to the `#[ignore]`d `advise_sets_nh_flag_on_our_iox2_pool_vmas`
/// above. That test needs `/proc/self/smaps` (run by hand, `#[ignore]`d). THIS one
/// needs only `/proc/self/maps`, present on every Linux host INCLUDING CI, so it
/// is NOT `#[ignore]`d — it runs in the ordinary Linux CI suite (the whole
/// cerulion_core suite runs serially in CI). It guards the load-bearing
/// assumption that iceoryx2's real segment pathnames carry the `iox2_` prefix
/// that `shm_vma_ranges` (and thus the madvise mitigation) matches on: an
/// upstream segment-name rename fails THIS assertion in Linux CI, not only the
/// hand-run `smaps` test. macOS: cfg'd out (no `/proc`, no user-shm THP).
#[cfg(target_os = "linux")]
#[test]
#[serial]
fn iox2_marker_present_in_real_pool_maps_ci() {
    let manager = TransportManager::get_or_init().expect("transport init");

    // Unique + oversized (128 MiB) so the pool reservation is real and this
    // test's mappings do not collide with a concurrently-run graph. Creating it
    // maps a real iceoryx2 pool (and fires the madvise create-hook).
    let topic = format!("/marker_{}", std::process::id());
    let _publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(128 * 1024 * 1024))
        .expect("create oversized publisher (maps a real iox2_ pool)");

    let maps = std::fs::read_to_string("/proc/self/maps")
        .expect("Linux CI: /proc/self/maps must be readable");
    // A bare `maps.contains("iox2_")` is a TAUTOLOGY: the test binary's OWN path
    // (`…/deps/shm_guard_madvise_iox2_test-<hash>`) contains `iox2_` and is
    // always listed in /proc/self/maps as a PRIVATE (`..p`) mapping, so it would
    // pass even with no SHM pool mapped. Require a SHARED mapping (perms end in
    // `s`) instead — the exe is private, so this demands a real iceoryx2 SHM
    // segment, and an upstream segment-name prefix rename actually fails here.
    assert!(
        maps.lines().any(|l| l.contains("iox2_")
            && l.split_whitespace()
                .nth(1)
                .is_some_and(|perms| perms.ends_with('s'))),
        "expected a SHARED `iox2_` segment pathname in /proc/self/maps after \
         creating a real iceoryx2 pool — an upstream segment-name rename would \
         silently break the `shm_vma_ranges` scan the MADV_NOHUGEPAGE mitigation \
         depends on"
    );
}

/// Behavioral regression pin: creating a
/// SUBSCRIBER first (no publisher on the topic yet) must NOT emit the
/// MADV_NOHUGEPAGE scan-anomaly warn. The subscriber-create hook
/// `advise_shm_pools_no_hugepage(PoolAdviseSite::Subscriber)` runs here; because
/// a subscriber maps producer pools LAZILY, an empty `iox2_` scan at the
/// subscriber site is legitimate, so the anomaly warn is suppressed there
/// unconditionally. Without that suppression it spuriously fires AND burns
/// the one-shot `SHM_MITIGATION_ANOMALY_WARNED` token (masking every genuine
/// future broken-scan warn). A FULL revert of the gate (subscriber site warns on
/// an empty scan) re-emits the warn on this path and fails this test.
///
/// NON-FLAKY but a CONDITIONAL discriminator — read this before trusting it as a
/// mutation guard. On correct code the assertion holds regardless of the scan
/// count (the subscriber site structurally never reaches the warn), so it never
/// false-fails. Its power to ALSO catch the narrower mutation "subscriber site
/// re-wired to `Publisher`" depends on the scan being EMPTY here — and
/// `shm_vma_ranges` scans `/proc/self/maps` PROCESS-WIDE, not this one topic. A
/// sibling test's leftover publisher pool VMA (tests share this binary + the
/// `TransportManager` singleton) or a persistent iceoryx2 management/config
/// `iox2_` SHARED segment can leave `found > 0`, under which even the mis-wired
/// `Publisher` variant would not warn (`found != 0`) and this test passes
/// vacuously. The UNCONDITIONAL, hermetic guard for the gate DECISION is the
/// in-crate oracle-vector unit test `scan_anomaly_warns_only_for_publisher_
/// empty_scan`; THIS test adds real-subscriber-hook integration coverage plus a
/// full-revert behavioral pin.
///
/// Runs in Linux CI (NOT `#[ignore]`d — like `iox2_marker_present_..._ci`). The
/// actual subscriber-site `found` count is
/// recorded (see the `--nocapture` debug line) to confirm whether the mutation
/// discriminator is live on a given host. macOS: cfg'd out (the warn path is
/// Linux-only).
#[cfg(target_os = "linux")]
#[test]
#[serial]
#[tracing_test::traced_test]
fn subscriber_before_publisher_emits_no_scan_anomaly_warn_ci() {
    let manager = TransportManager::get_or_init().expect("transport init");

    // SUBSCRIBER FIRST: a unique topic with no publisher. `create_subscriber`
    // uses the open-or-create path, so it succeeds with no producer and fires
    // the subscriber-create madvise hook with (at most) an empty pool scan.
    let topic = format!("/sub_first_{}", std::process::id());
    let _subscriber = manager
        .create_subscriber(&topic)
        .expect("create subscriber-first (fires the subscriber madvise hook)");

    // The scan-anomaly warn carries the distinctive "scan is broken" phrase; it
    // must NOT have fired from the subscriber site. `logs_contain` is injected by
    // `#[traced_test]` and sees this test's captured tracing events.
    assert!(
        !logs_contain("scan is broken"),
        "subscriber-before-publisher must NOT emit the MADV_NOHUGEPAGE \
         scan-anomaly warn — an empty scan is legitimate at the subscriber site \
         (producer pools map lazily). Firing it here is the regression \
         that also burns the one-shot anomaly token, disabling all future \
         genuine broken-scan warns."
    );
}

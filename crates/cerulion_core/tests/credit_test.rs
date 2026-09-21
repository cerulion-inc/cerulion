// SPDX-License-Identifier: AGPL-3.0-only
//! Behavioral tests for the cross-process block-credit word
//! (`cerulion_core::credit`) and its SHM productization [`MappedCredit`].
//!
//! Real POSIX SHM on macOS AND Linux (there is no stub arm on either — the
//! `MappedBarrier` precedent), so these exercise the actual
//! `MAP_SHARED` page rather than a registry. No iceoryx2, no `#[serial]`: every
//! segment name is pid- AND tag-scoped, so concurrent test binaries and re-runs
//! can never collide, and `create_owned` owns + unlinks on `Drop` while
//! `open_unowned` is strict (never creates), so no test leaks a `/dev/shm`
//! object (the one deliberate `mem::forget` leak is documented at its call site).
//!
//! Oracles are HAND-WRITTEN throughout — a claim/publish/drain sequence is
//! compared against a count computed by hand in the test, never against a second
//! run of the same code.
//!
//! Where a test waits on another thread it uses a WALL-CLOCK-BOUNDED helper that
//! PANICS with the observed state rather than hanging CI (the `shm_ring_test`
//! lesson: an unbounded cross-thread spin fails as a 55-minute job timeout with
//! no attributable red).
//!
//! # What no arm in this file can prove: that a park reached the KERNEL
//!
//! Every park arm here orders its wake AFTER the parker's snapshot and then
//! narrows the remaining snapshot→syscall window with a bounded settle. Neither
//! closes it, and no assertion can, because there is no observable that
//! separates the two ways `park_wait_credit` returns `true`:
//!
//! - `parked_mask` is set by `ParkedEdgeGuard::enter`, i.e. BEFORE the snapshot,
//!   so it proves the guard was entered and nothing about the syscall;
//! - a genuine kernel block and a compare-failed-immediately return are the SAME
//!   return value, and a thread PREEMPTED between the flag and its syscall is
//!   indistinguishable from one blocked inside it — both accrue no CPU time and
//!   both return at the instant the wake landed;
//! - reading the parker's own elapsed wall cannot separate them either: it is
//!   measured from before the block, so a preemption that pushes the syscall
//!   past the wake produces the same duration a real block does.
//!
//! So the residual is stated rather than papered over, and what BOUNDS it is
//! measurement: the module's kill matrix records real `SHARED`→`PRIVATE`/`NONE`
//! variants failing the subprocess arm (9.95 s / 9.92 s against 10 s caps) and
//! the spurious-ring arm (5.00 s against its 5 s cap). Those walls are reachable
//! only if the block was genuinely entered, on the runs that were actually made.
//! A vacuous run is possible; a vacuous SUITE is what the matrix rules out.

use cerulion_core::credit::{
    credit_edge_id, credit_shm_name, credit_wake_word_primitive_available, CreditShared,
    MappedCredit, ParkedEdgeGuard, ProducerSlot, CREDIT_BYTES, PARKED_MASK_BITS,
};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// pid- and tag-scoped namespace so concurrent test binaries / re-runs never
/// collide on a POSIX SHM object.
fn test_ns(tag: &str) -> String {
    format!("c2a_{}_{tag}", std::process::id())
}

/// A representative per-edge id — the production recipe, not a bare string, so
/// these tests exercise the name a real `credit_edges` entry would derive.
fn edge_id() -> String {
    credit_edge_id("/perception/scan", "planner", "scan_in")
}

/// Spin (yielding) until `cond` holds, or PANIC with `what` after `budget`.
/// A generous LIVENESS ceiling in seconds against work measured in
/// milliseconds — load can delay it, never invert it.
fn await_until(budget: Duration, what: &str, mut cond: impl FnMut() -> bool) {
    let deadline = Instant::now() + budget;
    while !cond() {
        if Instant::now() >= deadline {
            panic!("timed out after {budget:?} waiting for {what}");
        }
        thread::yield_now();
    }
}

// ---------------------------------------------------------------- layout ----

/// The mapped struct's size/alignment are what make the cross-process mapping
/// sound; the const-asserts in the module fail the BUILD on drift, and this
/// states the same facts from OUTSIDE the crate (where the private field
/// offsets are not reachable).
#[test]
fn the_mapped_layout_is_pinned() {
    assert_eq!(std::mem::size_of::<CreditShared>(), 24);
    assert!(std::mem::align_of::<CreditShared>() >= 8);
    assert_eq!(
        CREDIT_BYTES, 64,
        "one cache line — the anti-false-sharing pad"
    );
    assert!(
        std::mem::size_of::<CreditShared>() <= CREDIT_BYTES,
        "the word must fit the region MappedCredit maps"
    );
}

// -------------------------------------------------- mapping / lifecycle ----

/// Rendezvous through the shared segment: an owner and a peer mapping of the
/// same `(ns, id)` operate on the SAME `CreditShared`. The producer's publish is
/// visible to the consumer's handle and vice versa, which is the entire point of
/// the word.
#[test]
fn owner_and_peer_share_one_credit_word() {
    let ns = test_ns("rendezvous");
    let id = edge_id();
    let owner = MappedCredit::create_owned(&ns, &id, 4).unwrap();
    let peer = MappedCredit::open_unowned(&ns, &id).unwrap();

    assert_eq!(peer.depth(), 4, "the peer reads the owner's stamped depth");
    assert_eq!(peer.outstanding(), 0);

    // The PRODUCER's side (say, the owner's handle) publishes twice…
    owner.record_published();
    owner.record_published();
    // …and the CONSUMER's side sees it through its own mapping.
    assert_eq!(peer.outstanding(), 2, "publishes cross the shared page");

    // The consumer drains one; the producer sees the room.
    peer.record_drained(1);
    assert_eq!(owner.outstanding(), 1, "drains cross the shared page");

    drop(peer);
    drop(owner);
}

/// Two mappings of the SAME object have DISTINCT virtual addresses yet share the
/// SAME physical page (proven by cross-mapping state visibility). On macOS this
/// is also the "the real POSIX arm is in use" tell — a by-name registry stub
/// would return equal addresses.
#[test]
fn two_mappings_have_distinct_addresses_and_one_physical_page() {
    let ns = test_ns("distinct_addr");
    let id = edge_id();
    let owner = MappedCredit::create_owned(&ns, &id, 2).unwrap();
    let peer = MappedCredit::open_unowned(&ns, &id).unwrap();

    // POSIX-only. The non-Unix arm is a by-name REGISTRY that deliberately hands
    // both handles ONE `Arc<CreditShared>` (so its behavioural tests stay real
    // shared-state tests rather than tautologies), and `addr()` there is
    // `Arc::as_ptr` — equal BY DESIGN, not by defect. The state-sharing half
    // below is the assertion that means something on every platform.
    #[cfg(unix)]
    assert_ne!(
        owner.addr(),
        peer.addr(),
        "two mappings of the same object → distinct virtual addresses"
    );
    owner.record_published();
    assert_eq!(peer.outstanding(), 1, "…yet the same physical page");
}

/// A STRICT `open_unowned` of a never-created segment ERRORS with `NotFound`.
/// A silent create would be the worst available failure: the worker would get
/// its OWN zeroed word, read `depth == 0`, and defer forever while the real edge
/// sat untouched.
///
/// # The assertion is the error KIND, and that is not pedantry
///
/// A bare `is_err()` here is satisfied for the WRONG REASON by the exact change
/// it exists to catch. MEASURED: adding `O_CREAT` to the open makes `shm_open`
/// SUCCEED and the subsequent `mmap` of a zero-length object fail, so the
/// function still returns `Err` — with an orphaned zero-length segment now
/// left behind — and the test passes. `NotFound` (ENOENT here, the registry's
/// `NotFound` off Unix) is reachable only by refusing to create.
#[test]
fn strict_open_of_a_missing_segment_errs_with_not_found() {
    let err = MappedCredit::open_unowned(&test_ns("missing"), &edge_id())
        .expect_err("strict open of a never-created segment must error (no silent create)");
    assert_eq!(
        err.kind(),
        std::io::ErrorKind::NotFound,
        "the refusal must be `the name does not exist`, not a create that then failed to map: {err}"
    );
}

/// A raw POSIX SHM object at `(ns, id)`'s PRODUCTION name, sized to `len` and
/// left ZERO-FILLED — byte for byte the state `create_owned` leaves behind if it
/// dies after `ftruncate` and before `reinit`.
///
/// The name comes from the production `credit_shm_name`, not from a copy of the
/// recipe: a second copy could drift from the one the open under test derives,
/// at which point the fixture would be crafting a segment nothing ever looks at
/// and every assertion below would pass for the wrong reason.
///
/// `Drop` unlinks, so this fixture leaks no `/dev/shm` object (the file header's
/// standing rule).
///
/// POSIX-only, for BOTH halves of the reason: it crafts the segment with raw
/// `libc`, and the state it crafts is unreachable on the non-Unix registry arm
/// (which runs `reinit` before it inserts, so there is no torn create to leave a
/// bare entry behind).
#[cfg(unix)]
struct RawSegment {
    name: std::ffi::CString,
}

#[cfg(unix)]
impl RawSegment {
    fn create(ns: &str, id: &str, len: usize) -> Self {
        let name = std::ffi::CString::new(credit_shm_name(ns, id)).expect("name has no NUL");
        // SAFETY: FFI create of a name this fixture derived; `O_EXCL` so a
        // stale object from an earlier run fails LOUDLY here rather than
        // silently supplying the page under test.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "fixture shm_open failed: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: sizing the object this call just created; POSIX zero-fills.
        let rc = unsafe { libc::ftruncate(fd, len as libc::off_t) };
        assert_eq!(
            rc,
            0,
            "fixture ftruncate failed: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: closing the descriptor we opened — the NAME keeps the object
        // alive, which is exactly the orphan state being crafted.
        unsafe { libc::close(fd) };
        Self { name }
    }
}

#[cfg(unix)]
impl Drop for RawSegment {
    fn drop(&mut self) {
        // SAFETY: best-effort unlink of the name this fixture created.
        unsafe { libc::shm_unlink(self.name.as_ptr()) };
    }
}

/// A full-size but UNARMED segment is REFUSED, not mapped and believed.
///
/// The size check in `open_unowned` covers a creator that died between
/// `shm_open` and `ftruncate`. This covers the NEXT instant — one that died
/// between `ftruncate` and `reinit` — and the size check structurally cannot
/// reach it: what is left behind is correctly named, correctly SIZED and
/// zero-filled, so it passes every byte-count test there is.
///
/// What makes that a silent WEDGE rather than a slow edge is the arithmetic:
/// the page reads `depth == 0`, `is_full` is `outstanding >= depth`, and
/// `outstanding >= 0` holds unconditionally — so the peer's producer defers
/// entry to EVERY tick, forever, with no drain that can ever recover it. Refused
/// loudly is the only correct answer (the loud-degrade-over-silent rule);
/// this is the same wedge `reject_zero_depth` refuses on the create side,
/// arriving through the open side instead.
///
/// The crash is triggered directly, not modelled at the API: [`RawSegment`] builds the
/// object with `shm_open` + `ftruncate` and nothing else.
///
/// # The anti-tautology half is IN BODY, and it is not redundant
///
/// `two_mappings_have_distinct_addresses_and_one_physical_page` and the
/// round-trip arms already open ARMED pages, but they do it in a different
/// namespace under a different fixture. The in-body control opens an ARMED page
/// in THIS test's namespace, so a refusal above cannot be explained by the
/// namespace, by the raw-crafted name, or by `open_unowned` being broken
/// outright — only by the arm.
#[cfg(unix)]
#[test]
fn a_full_size_but_unarmed_segment_is_refused_rather_than_wedging_its_producer() {
    let ns = test_ns("unarmed");
    let id = edge_id();
    let _seg = RawSegment::create(&ns, &id, CREDIT_BYTES);

    let err = MappedCredit::open_unowned(&ns, &id)
        .expect_err("a full-size but UNARMED segment must be refused, never mapped and believed");

    assert_eq!(
        err.kind(),
        std::io::ErrorKind::InvalidData,
        "an unarmed page is malformed DATA, not a missing name: {err}"
    );
    let msg = err.to_string();
    assert!(
        msg.contains("never ARMED"),
        "the refusal must name the UNARMED state — an operator reading it has to know the \
         difference between this and a missing segment: {msg}"
    );
    assert!(
        msg.contains("epoch 0"),
        "…and the evidence it was read from: {msg}"
    );
    assert!(
        msg.contains(&credit_shm_name(&ns, &id)),
        "…and WHICH segment, or an operator cannot go and look at it: {msg}"
    );
    assert!(
        msg.contains("died between ftruncate and reinit"),
        "…and the likely cause: {msg}"
    );
    assert!(msg.contains("Restart the owner"), "…and the remedy: {msg}");

    // ANTI-TAUTOLOGY — an ARMED page in the SAME namespace still opens.
    let armed_id = format!("{id}\u{1f}armed");
    let owner = MappedCredit::create_owned(&ns, &armed_id, 4).expect("armed create");
    let peer = MappedCredit::open_unowned(&ns, &armed_id)
        .expect("an ARMED page in this same namespace must still open");
    assert_eq!(peer.depth(), 4, "…and carries the owner's stamped depth");
    assert!(peer.try_claim(), "…and is a working, claimable word");
    drop(peer);
    drop(owner);
}

/// Owner drop unlinks the NAME while a still-held peer mapping stays valid
/// (unlink is not unmap). This is the contract a producer PARKED on the word
/// depends on: an owner tearing down mid-run can never invalidate the memory a
/// futex/os_sync wait is blocked on.
#[test]
fn owner_drop_unlinks_the_name_but_a_held_mapping_survives() {
    let ns = test_ns("owner_drop");
    let id = edge_id();
    let owner = MappedCredit::create_owned(&ns, &id, 8).unwrap();
    let peer = MappedCredit::open_unowned(&ns, &id).unwrap();

    owner.record_published();
    owner.record_published();
    owner.record_published();
    drop(owner);

    // The name is gone…
    assert!(
        MappedCredit::open_unowned(&ns, &id).is_err(),
        "owner drop unlinked the name — a fresh open_unowned now fails"
    );
    // …but the held mapping reads the intact state and remains fully usable.
    assert_eq!(
        peer.outstanding(),
        3,
        "the surviving mapping still sees the pre-drop state"
    );
    peer.record_drained(2);
    assert_eq!(peer.outstanding(), 1, "and still mutates it");
    assert!(peer.try_claim(), "and still answers the gate");
}

/// Same `id`, DIFFERENT `ns` → DIFFERENT segments: no cross-deployment
/// rendezvous. `a` and `b` are independent working credit words.
#[test]
fn a_different_namespace_is_a_different_segment() {
    let id = edge_id();
    let a = MappedCredit::create_owned(&test_ns("ns_a"), &id, 2).unwrap();
    let b = MappedCredit::create_owned(&test_ns("ns_b"), &id, 2).unwrap();

    a.record_published();
    a.record_published();
    assert!(a.is_full(), "a is at its depth");
    assert_eq!(b.outstanding(), 0, "b is untouched by a");
    assert!(b.try_claim(), "b is an independent, working word");
}

/// Reusing an ORPHANED name resets to a FRESH word. A single owner drives the
/// count to its depth and is `mem::forget`-ed to simulate a CRASH (no Drop → no
/// unlink → an orphan sitting at full). A new `create_owned` of the same
/// `(ns, id)` must observe an empty, claimable word — proving the orphan was
/// cleared (unlink-then-`O_EXCL`), because a producer inheriting a crashed run's
/// "full" would defer forever.
#[test]
fn reusing_an_orphaned_segment_resets_it() {
    let ns = test_ns("orphan_reuse");
    let id = edge_id();
    let owner = MappedCredit::create_owned(&ns, &id, 2).unwrap();
    owner.record_published();
    owner.record_published();
    assert!(owner.is_full(), "the crashed run left the word FULL");
    assert!(
        owner.park_enter(ProducerSlot::new(0)),
        "…and a producer parked on it"
    );

    // Simulate a CRASH: no Drop runs, so the segment is orphaned at full.
    // NOTE: `mem::forget` intentionally leaks one bounded (64-byte) segment for
    // the rest of the process (reclaimed at process exit) — exactly what a
    // crash-without-unlink would do.
    std::mem::forget(owner);

    let fresh = MappedCredit::create_owned(&ns, &id, 2).unwrap();
    assert_eq!(fresh.outstanding(), 0, "orphan reuse resets the count");
    assert!(fresh.try_claim(), "a fresh word is claimable");
    assert_eq!(
        fresh.parked_mask(),
        0,
        "a crashed peer's stale parked bit must not tax every drain forever"
    );
    assert_eq!(fresh.depth(), 2);
    // EXACTLY 1, not `>= stale_epoch`. The loose form was near-vacuous: the
    // orphan's epoch is 1 and the reuse's is 1, so `1 >= 1` holds no matter
    // what the create path does with the field. The tight form states the real
    // property — reuse is a FRESH page whose epoch starts over, which is
    // precisely why the field cannot serve as a staleness signal.
    assert_eq!(
        fresh.epoch(),
        1,
        "orphan reuse mints a FRESH page, so its epoch starts at 1 rather than continuing"
    );
}

/// A SECOND `create_owned` of a live name SUCCEEDS and yields a FRESH,
/// INDEPENDENT page — the direct consequence of the unlink-first design.
///
/// The `O_EXCL` is a best-effort RACING-supervisor detector, not a lock: because
/// the create unlinks first (the orphan clear the test above depends on), a
/// NON-racing second create simply replaces the name. This pins that the first
/// owner's mapping is left alive on the old, now-unnamed object rather than
/// silently sharing state with the newcomer.
#[test]
fn a_second_create_replaces_the_name_with_an_independent_page() {
    let ns = test_ns("double_create");
    let id = edge_id();
    let first = MappedCredit::create_owned(&ns, &id, 4).unwrap();
    first.record_published();
    first.record_published();
    first.record_published();

    let second = MappedCredit::create_owned(&ns, &id, 4).unwrap();
    assert_eq!(second.outstanding(), 0, "the second create is FRESH");
    assert_eq!(
        first.outstanding(),
        3,
        "the first owner's mapping survives on the old object, unshared"
    );
    second.record_published();
    assert_eq!(
        first.outstanding(),
        3,
        "and the two pages are genuinely independent"
    );
    assert_eq!(second.outstanding(), 1);
}

/// A "re-arm" is always a FRESH PAGE, so NO held mapping ever observes an epoch
/// bump and a re-opened one always reads 1.
///
/// This test is named for what it PROVES rather than for what the field is
/// called. `reinit` bumps the epoch, but its only caller (`create_owned`)
/// unlinks the name and `O_EXCL`-creates a new object first — so the bump lands
/// on a page nobody else is mapped to, every time. The field is therefore a
/// RESERVED seam for a future in-place re-arm, NOT a live staleness signal, and
/// this arm is the standing evidence for that (see the `epoch` field docs).
///
/// Three claims, and the second two are the ones that matter:
/// 1. a fresh arm reads 1 (the create path really does run `reinit`);
/// 2. a HELD mapping across a re-create reads its ORIGINAL value — it is mapped
///    to the old, now-unnamed object, which nothing ever touches again;
/// 3. a RE-OPEN by name reads 1, not 2 — so a peer cannot distinguish "re-armed"
///    from "fresh" on this path either.
#[test]
fn a_re_arm_is_a_fresh_page_so_no_held_mapping_ever_sees_an_epoch_bump() {
    let ns = test_ns("epoch");
    let id = edge_id();
    let first = MappedCredit::create_owned(&ns, &id, 2).unwrap();
    let e1 = first.epoch();
    assert_eq!(
        e1, 1,
        "a fresh O_EXCL page is zeroed, so the first arm is 1"
    );

    let second = MappedCredit::create_owned(&ns, &id, 2).unwrap();
    assert_eq!(
        second.epoch(),
        1,
        "a NEW page starts its own epoch at 1 (the old page is unnamed, not re-armed)"
    );
    assert_eq!(
        first.epoch(),
        e1,
        "a HELD mapping never observes the bump — it is on the old, unnamed object"
    );
    // …and neither does a peer that re-opens BY NAME afterwards.
    let reopened = MappedCredit::open_unowned(&ns, &id).unwrap();
    assert_eq!(
        reopened.epoch(),
        1,
        "a re-opened mapping always reads 1: a stale mapping is NOT detectable here"
    );
}

// --------------------------------------------------------- the gate math ----

/// Claim / publish / drain against a HAND-COMPUTED count, over the real shared
/// page.
#[test]
fn claim_publish_drain_round_trip_matches_a_hand_oracle() {
    let ns = test_ns("round_trip");
    let id = edge_id();
    let credit = MappedCredit::create_owned(&ns, &id, 10).unwrap();

    // Hand oracle: publish 7, drain 3, publish 2, drain 6 → 0 outstanding.
    let mut expected: u64 = 0;
    for _ in 0..7 {
        assert!(credit.try_claim(), "room remains below depth 10");
        credit.record_published();
        expected += 1;
        assert_eq!(credit.outstanding(), expected);
    }
    credit.record_drained(3);
    expected -= 3;
    assert_eq!(credit.outstanding(), expected);
    assert_eq!(expected, 4, "hand oracle: 7 published, 3 drained");

    for _ in 0..2 {
        credit.record_published();
        expected += 1;
    }
    assert_eq!(expected, 6, "hand oracle: 4 held plus 2 more");
    assert_eq!(credit.outstanding(), expected);

    credit.record_drained(expected);
    assert_eq!(credit.outstanding(), 0, "hand oracle: fully drained");
    assert!(credit.try_claim());
}

/// The decrement SATURATES at zero. An interleaving non-drain removal (a late
/// joiner's history replay) could otherwise drive the mirror below zero, and the
/// far side has no way to notice — clamping keeps `outstanding == 0` the floor.
#[test]
fn the_drain_saturates_at_zero_rather_than_wrapping() {
    let ns = test_ns("saturate");
    let id = edge_id();
    let credit = MappedCredit::create_owned(&ns, &id, 4).unwrap();

    credit.record_published();
    credit.record_drained(1_000_000);
    assert_eq!(
        credit.outstanding(),
        0,
        "a drain larger than the count clamps to the floor, never wraps to u64::MAX"
    );
    assert!(
        credit.try_claim(),
        "and the word is claimable rather than permanently full"
    );

    // From an already-empty word, too.
    credit.record_drained(u64::MAX);
    assert_eq!(credit.outstanding(), 0);
}

/// The full-at-depth boundary, pinned on BOTH sides: `outstanding == depth`
/// defers, `depth - 1` does not. A `>` instead of `>=` would let the producer
/// enter one tick too many and overflow a declared-LOSSLESS queue (Principle #6).
#[test]
fn the_defer_boundary_is_pinned_on_both_sides() {
    let ns = test_ns("boundary");
    let id = edge_id();
    const DEPTH: u32 = 3;
    let credit = MappedCredit::create_owned(&ns, &id, DEPTH).unwrap();

    for i in 1..DEPTH {
        credit.record_published();
        assert!(
            credit.try_claim(),
            "at outstanding {i} of depth {DEPTH} there is still room"
        );
        assert!(!credit.is_full());
    }
    // One more brings it to exactly `depth`.
    credit.record_published();
    assert_eq!(credit.outstanding(), u64::from(DEPTH));
    assert!(credit.is_full(), "outstanding == depth MUST defer");
    assert!(!credit.try_claim());

    // One drain re-opens it, and exactly one.
    credit.record_drained(1);
    assert!(credit.try_claim(), "depth - 1 must NOT defer");
    credit.record_published();
    assert!(
        !credit.try_claim(),
        "…and the very next publish closes it again"
    );
}

/// THE CHUNK-2 SCOPE FENCE, pinned as CURRENT TRUTH: `try_claim` RESERVES
/// NOTHING, so two claims taken before either publish both succeed and the word
/// ends up ABOVE its declared depth.
///
/// Driven single-threaded, so this is not a race — it is the semantics. Under
/// the fence (exactly ONE producer per edge, one publish per tick entry) the
/// sequence below is unreachable in production: a single producer claims, ticks,
/// publishes, and only then asks again. The reason to write it down anyway is
/// that `try_claim`'s NAME promises a reservation the body does not make, and
/// the multi-producer extension the module defers is precisely what turns this
/// assertion around.
///
/// **When the CAS reservation lands (claim-iff-below-depth, publish-or-release),
/// this test must FLIP**: the second `try_claim` will return `false` and
/// `outstanding` will stay at 1. Failing here is therefore the SIGNAL that the
/// extension arrived, not a regression — which is why the assertion carries the
/// expected new value rather than only the old one.
#[test]
fn a_second_claim_before_publishing_over_fills_the_word_under_the_chunk_2_fence() {
    let ns = test_ns("scope_fence");
    let id = edge_id();
    let credit = MappedCredit::create_owned(&ns, &id, 1).unwrap();

    // Two claims, THEN two publishes — the shape the fence exists to exclude.
    assert!(credit.try_claim(), "the first claim sees an empty word");
    assert!(
        credit.try_claim(),
        "CURRENT TRUTH: the second claim also succeeds, because try_claim is \
         `!is_full()` and reserves nothing. Under the CAS extension this becomes false."
    );
    credit.record_published();
    credit.record_published();

    assert_eq!(
        credit.outstanding(),
        2,
        "CURRENT TRUTH: the word sits ABOVE its declared depth of 1. Under the CAS \
         reservation (claim-iff-below-depth, publish-or-release) this becomes 1, and this \
         assertion flipping is the SIGNAL that the extension landed."
    );
    assert!(credit.is_full(), "and it reads full either way");
}

// ------------------------------------------------------------ the park -----

/// The wake epoch advances when credit is FREED and at no other time — over the
/// real shared page, so a wake rung by the consumer's handle is observable
/// through the producer's.
///
/// With `parked_mask() == 0` throughout, no wake SYSCALL is reachable: this is
/// the "atomics-only when nobody parks" contract, stated where it is checkable.
#[test]
fn the_wake_epoch_advances_only_when_credit_is_freed() {
    let ns = test_ns("wake_seq");
    let id = edge_id();
    let consumer = MappedCredit::create_owned(&ns, &id, 8).unwrap();
    let producer = MappedCredit::open_unowned(&ns, &id).unwrap();

    assert_eq!(producer.parked_mask(), 0, "nobody parks in this test");
    let base = producer.wake_seq_snapshot();

    assert!(producer.try_claim());
    let _ = producer.is_full();
    let _ = producer.outstanding();
    assert_eq!(
        producer.wake_seq_snapshot(),
        base,
        "a READ never rings the word"
    );

    producer.record_published();
    assert_eq!(
        producer.wake_seq_snapshot(),
        base,
        "a publish frees no credit — nothing to wake for"
    );

    consumer.record_drained(0);
    assert_eq!(
        producer.wake_seq_snapshot(),
        base,
        "an EMPTY drain must not tax a parked producer with a spurious wake"
    );

    consumer.record_drained(1);
    assert_eq!(
        producer.wake_seq_snapshot(),
        base.wrapping_add(1),
        "a freeing drain rings the word exactly once, across the shared page"
    );
}

/// The `ParkedEdgeGuard` clears its bit on a NORMAL exit and on a PANIC UNWIND.
/// The unwind arm is the one that matters: a producer whose park panics must not
/// leave a bit that taxes every future drain with a no-op wake syscall forever.
#[test]
fn the_park_guard_clears_its_bit_on_normal_exit_and_on_unwind() {
    let credit = CreditShared::new(4);

    {
        let _g = ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 is in-mask");
        assert_eq!(
            credit.parked_mask(),
            1,
            "the bit is held for the guard's life"
        );
    }
    assert_eq!(credit.parked_mask(), 0, "normal exit clears it");

    let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _g = ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 is in-mask");
        assert_eq!(credit.parked_mask(), 1);
        panic!("the producer's park panicked");
    }));
    assert!(unwound.is_err(), "the panic really propagated");
    assert_eq!(
        credit.parked_mask(),
        0,
        "a panicking park must not leak its parked bit"
    );
}

/// THE HEADLINE for the park: a producer kernel-blocked on the credit word is
/// WOKEN by the consumer's drain, over the REAL shared page and the REAL
/// primitive on this machine.
///
/// The oracle is bounded and deterministic: the block must return far inside its
/// 5 s cap, and the woken producer's re-derive must see room. On a machine with no
/// wake primitive the whole arm is skipped — there is nothing to observe, and
/// asserting a wall against the sleep-recheck fallback would be the
/// load-inversion class.
///
/// # No bias sleep — the drain waits on the producer's own parked BIT
///
/// A `sleep(5ms)` before the drain is a bet that 5 ms is enough for a spawn +
/// an `open_unowned` + a kernel block, which is the load-inversion class: under load
/// the drain lands FIRST, the block's kernel compare fails immediately, and the
/// test passes while proving nothing about the wake. `await_until` on
/// `parked_mask() != 0` waits for the producer's own `ParkedEdgeGuard` to
/// publish that it is about to block — a CONDITION load can delay but never
/// invert.
///
/// The bit is set BEFORE the snapshot (the SB-litmus protocol requires it), so
/// a residual window remains in which the drain lands between the guard and the
/// producer's final `try_claim`. That is why the "still full at the moment of
/// parking" precondition is CAPTURED and REPORTED rather than asserted:
/// tripping it means the harness raced, not that the code is wrong, and a
/// harness race must DEGRADE (skip the claims it can no longer support), never
/// invert into a red run.
///
/// # SCOPE: this arm does not, on its own, prove a WAKE
///
/// MEASURED against reverting `OS_SYNC_*_SHARED` to `NONE` (which breaks the
/// wake for anything but the waker's own address): **this arm stayed GREEN.**
/// The parked bit is published before the snapshot AND before the kernel entry,
/// so the drain routinely lands in that window, the kernel's
/// compare against the stale snapshot ends the park immediately, and the result
/// is indistinguishable from a wake. What it reliably pins is the PROTOCOL —
/// guard, snapshot, block, re-derive — and that the freed credit is visible
/// across two mappings.
///
/// The arms that DO isolate the wake, and catch that regression, are
/// `a_spurious_ring_wakes_the_producer_who_re_derives_no_room` (a ring that
/// leaves the predicate false, so only a wake can end the block) and
/// `subprocess_harness::a_child_process_parks_on_the_shared_word_and_is_woken_by_the_parents_drain`.
#[test]
fn a_parked_producer_is_woken_by_the_consumers_drain() {
    if !credit_wake_word_primitive_available() {
        eprintln!("no wake primitive on this host — skipping (fallback tier)");
        return;
    }
    let ns = test_ns("park_wake");
    let id = edge_id();
    let consumer = Arc::new(MappedCredit::create_owned(&ns, &id, 1).unwrap());
    // Fill it, so the producer genuinely has no credit and must park.
    consumer.record_published();
    assert!(!consumer.try_claim(), "the word is full before the park");

    let producer = {
        let ns = ns.clone();
        let id = id.clone();
        thread::spawn(move || {
            let producer = MappedCredit::open_unowned(&ns, &id).expect("peer opens");
            let _g =
                ParkedEdgeGuard::enter(&producer, ProducerSlot::new(0)).expect("index 0 in-mask");
            let snap = producer.wake_seq_snapshot();
            // CAPTURED, not asserted — see the doc comment.
            let full_at_park = !producer.try_claim();
            let start = Instant::now();
            let performed = producer.park_wait_credit(snap, Duration::from_secs(5));
            (
                performed,
                start.elapsed(),
                producer.try_claim(),
                full_at_park,
            )
        })
    };

    // Wait for the producer's own parked bit, then free credit.
    await_until(
        Duration::from_secs(10),
        "the producer to publish its parked bit",
        || consumer.parked_mask() != 0,
    );
    consumer.record_drained(1);

    let (performed, elapsed, saw_room, full_at_park) =
        producer.join().expect("producer thread panicked");
    if !full_at_park {
        // The drain landed inside the guard→snapshot window. The producer's
        // behaviour is still CORRECT (the snapshot is the lost-wakeup guard),
        // but this run cannot say anything about the wake — degrade loudly.
        eprintln!(
            "DEGRADE: the drain raced the producer's pre-block re-derive, so this run \
             proves nothing about the wake (performed={performed}, elapsed={elapsed:?}, \
             saw_room={saw_room})"
        );
        return;
    }
    assert!(performed, "the kernel block must report performed");
    assert!(
        elapsed < Duration::from_secs(2),
        "the drain must wake the parked producer far inside the 5s cap — took {elapsed:?}"
    );
    assert!(
        saw_room,
        "the woken producer's re-derive must see the freed credit"
    );
}

/// A wake with NO state change is a bounded no-op re-park — the
/// spurious-tolerance contract that lets the consumer side over-signal freely
/// (e.g. the live loop ringing the word on a dead-consumer release). The block
/// returns promptly and the re-derive correctly still reads "no room".
///
/// # No bias sleep — the ring is ordered AFTER the snapshot, deterministically
///
/// This arm's whole subject is a ring that lands after a snapshot was taken:
/// ring FIRST and the snapshot captures the already-bumped value, so the block
/// simply times out at 5 s and the "promptly" assertion inverts. A
/// `sleep(5ms)` only makes that unlikely, which is the load-inversion class. The
/// producer instead PUBLISHES an `AtomicBool` immediately after its snapshot
/// and main `await_until`s it before ringing, so ring-strictly-after-snapshot
/// is a fact rather than a bet.
///
/// # RESIDUAL: nothing here proves the block reached the KERNEL
///
/// The flag is set after the snapshot and before the block, so the ring can
/// still land in the pre-block window — the kernel's compare against the stale
/// snapshot then fails and the call returns at once, which is CORRECT and
/// bounded but does NOT exercise the wake. Both interleavings satisfy every
/// assertion here, and that is not fixable by a better assertion: see the file
/// header's `# What no arm in this file can prove` — `park_wait_credit` returns
/// `true` either way, and a thread PREEMPTED between the flag and its syscall is
/// indistinguishable from one blocked inside it.
///
/// So the window is NARROWED, never closed, in three ways that can each only
/// make a run MORE probative:
///
/// - the ring is ordered strictly after the snapshot by the flag (above);
/// - `PRE_RING_SETTLE` covers the handful of instructions between the flag and
///   the syscall — admissible for the same reason it is in
///   `subprocess_harness`: too short costs a VACUOUS PASS, never a false
///   failure;
/// - the parked bit is asserted VISIBLE ACROSS THE MAPPING before the ring, so
///   a run whose producer never even reached its guard fails attributably here
///   instead of passing quietly.
///
/// What bounds the residual beyond that is MEASUREMENT, not argument: the
/// module's kill matrix records this arm failing the
/// `OS_SYNC_*_SHARED`→`NONE` revert at 5.00 s against its 5 s cap — a wall only
/// reachable if the block was genuinely entered.
#[test]
fn a_spurious_ring_wakes_the_producer_who_re_derives_no_room() {
    /// Slack between the producer's snapshot flag and the ring, covering the
    /// instructions between the flag and the syscall. Safe in ONE direction
    /// only (a vacuous pass, never a wrong verdict), which is why it is a
    /// narrowing and not the ordering mechanism — the flag is that.
    const PRE_RING_SETTLE: Duration = Duration::from_millis(50);

    if !credit_wake_word_primitive_available() {
        eprintln!("no wake primitive on this host — skipping (fallback tier)");
        return;
    }
    let ns = test_ns("spurious");
    let id = edge_id();
    let consumer = Arc::new(MappedCredit::create_owned(&ns, &id, 1).unwrap());
    consumer.record_published();
    let snapped = Arc::new(AtomicBool::new(false));

    let producer = {
        let ns = ns.clone();
        let id = id.clone();
        let snapped = Arc::clone(&snapped);
        thread::spawn(move || {
            let producer = MappedCredit::open_unowned(&ns, &id).expect("peer opens");
            let _g =
                ParkedEdgeGuard::enter(&producer, ProducerSlot::new(0)).expect("index 0 in-mask");
            let snap = producer.wake_seq_snapshot();
            snapped.store(true, Ordering::Release);
            let start = Instant::now();
            let performed = producer.park_wait_credit(snap, Duration::from_secs(5));
            (performed, start.elapsed(), producer.try_claim())
        })
    };

    await_until(
        Duration::from_secs(10),
        "the producer to take its wake-word snapshot",
        || snapped.load(Ordering::Acquire),
    );
    // …then a short settle for the last instructions before the syscall.
    thread::sleep(PRE_RING_SETTLE);
    assert_ne!(
        consumer.parked_mask(),
        0,
        "the producer's parked bit must be visible through the CONSUMER's mapping before the \
         ring — a run whose producer never reached its guard proves nothing about the wake"
    );

    // Ring the word WITHOUT freeing anything.
    consumer.note_credit_freed();

    let (performed, elapsed, saw_room) = producer.join().expect("producer thread panicked");
    assert!(performed);
    assert!(
        elapsed < Duration::from_secs(2),
        "a spurious ring must still wake the block promptly — took {elapsed:?}"
    );
    assert!(
        !saw_room,
        "the re-derive after a spurious wake must read no-room (a bounded no-op re-park)"
    );
    assert_eq!(
        consumer.parked_mask(),
        0,
        "the guard released its bit on the way out, so no stale bit taxes a future drain"
    );
}

/// THE SB-LITMUS HAMMER — parker and consumer race with NO bias sleeps, 100
/// fresh rounds, every round's park bounded strictly inside its 1 s cap.
///
/// Ported from `barrier.rs`'s `sb_hammer_every_round_bounded`, and its ACTUAL
/// STRENGTH is stated the same way, because the temptation is to over-claim it:
///
/// **It pins bounded latency + PROTOCOL ORDERING, not the fences per se.** What
/// it proves is that one of the three guards always catches the free-credit
/// event — the post-bit-set predicate re-derive, the kernel's compare against
/// the pre-block snapshot, or the `parked`-gated wake syscall — so no round ever
/// rides to its cap. It does NOT isolate the `SeqCst` fences: the UNCONDITIONAL
/// `wake_seq` bump means a fence-less build still gives the parker a compare
/// value that already differs, and the KERNEL's own atomic compare-and-block
/// bounds it. (Measured, and recorded in the module's kill matrix: deleting the
/// fence in `note_credit_freed` SURVIVES this hammer, as expected.) The fences
/// make the litmus model-correct rather than merely hardware-likely; what this
/// arm catches is a PROTOCOL inversion — a snapshot taken before the bit-set, a
/// wake gated on the wrong word, a drain that frees credit without ringing.
///
/// Uses `Arc<CreditShared>` rather than `MappedCredit` for the same reason the
/// barrier's hammer does: 100 rounds × (unlink + O_EXCL create + mmap + unlink)
/// would make this a filesystem test. The atomics under test are identical —
/// the SHM page holds this very struct — and the cross-ADDRESS-SPACE half is
/// proven by `subprocess_harness`.
#[test]
fn the_sb_litmus_hammer_bounds_every_round() {
    const ROUNDS: usize = 100;
    const CAP: Duration = Duration::from_secs(1);
    for round in 0..ROUNDS {
        // Depth 1, one frame outstanding ⇒ FULL, so the producer must park.
        let credit = Arc::new(CreditShared::new(1));
        credit.record_published();

        let producer = {
            let credit = Arc::clone(&credit);
            thread::spawn(move || {
                let _g =
                    ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 in-mask");
                let snap = credit.wake_seq_snapshot();
                let start = Instant::now();
                if !credit.try_claim() {
                    credit.park_wait_credit(snap, CAP);
                }
                start.elapsed()
            })
        };
        let consumer = {
            let credit = Arc::clone(&credit);
            // NO bias sleep: the two threads race, which is the point.
            thread::spawn(move || credit.record_drained(1))
        };

        consumer.join().expect("consumer thread panicked");
        let elapsed = producer.join().expect("producer thread panicked");
        assert!(
            elapsed < Duration::from_millis(900),
            "round {round}: the park must return inside its cap (no lost wake) — took {elapsed:?}"
        );
    }
}

/// A QUIET park — nobody rings the word — SPENDS its cap and comes back with
/// the gate still shut.
///
/// The oracle is a wall LOWER bound and nothing else. Load can only make a
/// timed wait LONGER, so a floor is an assertion contention cannot invert,
/// while a ceiling on the same quantity is exactly the load-inversion class. The floor
/// is deliberately half the cap rather than the cap itself: the kernel times
/// out against its own clock source (`OS_CLOCK_MACH_ABSOLUTE_TIME` on macOS,
/// `CLOCK_MONOTONIC` on Linux) while the test measures `Instant`, so a few
/// percent of skew is expected — and half a cap still separates "blocked" from
/// a compare-failed-immediately return, which happens in microseconds.
///
/// Skipped without a primitive: there the call reports not-performed at once by
/// design, so there is no block to time.
#[test]
fn a_quiet_park_spends_its_cap_and_still_reads_no_room() {
    if !credit_wake_word_primitive_available() {
        eprintln!("no wake primitive on this host — skipping (fallback tier)");
        return;
    }
    const CAP: Duration = Duration::from_millis(200);
    let ns = test_ns("quiet_park");
    let id = edge_id();
    let credit = MappedCredit::create_owned(&ns, &id, 1).unwrap();
    credit.record_published();
    assert!(!credit.try_claim(), "the word is full before the park");

    let _g = ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 in-mask");
    let snap = credit.wake_seq_snapshot();
    let start = Instant::now();
    let performed = credit.park_wait_credit(snap, CAP);
    let elapsed = start.elapsed();

    assert!(performed, "a real bounded block must report performed");
    assert!(
        elapsed >= CAP / 2,
        "nobody rang the word, so the park must have BLOCKED for its cap — took {elapsed:?} \
         against a {CAP:?} cap"
    );
    assert!(
        !credit.try_claim(),
        "a park that expired on its own frees no credit — the gate is still shut"
    );
}

/// A ZERO cap returns immediately and reports NOT-PERFORMED, without ever
/// reaching a kernel call.
///
/// This is the `cap.is_zero()` guard's only oracle, and the skip above is what
/// gives it teeth: WITH a primitive present, `park_wait_credit` returns `true`
/// on every path except that guard (a timeout, a spurious wake, an `EAGAIN`
/// compare miss and even a degraded errno all report performed). So `false`
/// here can mean ONE thing, and deleting the guard hands a
/// zero-length timeout to the kernel and reports performed instead.
///
/// The return value is the whole observable on purpose: "no syscall was made"
/// has no wall-clock signature a loaded runner could not fake in either
/// direction, whereas the boolean is exact.
#[test]
fn a_zero_cap_park_returns_without_a_syscall() {
    if !credit_wake_word_primitive_available() {
        eprintln!("no wake primitive on this host — skipping (fallback tier)");
        return;
    }
    let ns = test_ns("zero_cap");
    let id = edge_id();
    let credit = MappedCredit::create_owned(&ns, &id, 1).unwrap();
    credit.record_published();

    let _g = ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 in-mask");
    let snap = credit.wake_seq_snapshot();
    assert!(
        !credit.park_wait_credit(snap, Duration::ZERO),
        "a zero cap has nothing to block for: report NOT-performed and make no kernel call"
    );
    // Anti-tautology, in the same body: the SAME word with a real cap DOES
    // block, so the `false` above is the guard rather than a dead primitive.
    let start = Instant::now();
    assert!(
        credit.park_wait_credit(snap, Duration::from_millis(50)),
        "a non-zero cap on this host must perform a real block"
    );
    assert!(
        start.elapsed() >= Duration::from_millis(25),
        "…and that block must really have waited"
    );
}

// ------------------------------------------------------------- the SPSC ----

/// A 2-thread SPSC stress over the real shared page: ONE producer (the
/// single-producer scope fence) claiming and publishing `N` frames against a shallow depth, ONE
/// consumer draining. Asserts NO LOSS and NO OVER-PUBLISH against hand oracles —
/// exactly `N` published, exactly `N` drained, `outstanding == 0` at rest — and,
/// most importantly, that the producer NEVER entered a tick over its declared
/// depth (which is what the whole word exists to prevent).
#[test]
fn a_single_producer_and_consumer_never_exceed_the_declared_depth() {
    const N: u64 = 20_000;
    const DEPTH: u32 = 4;

    let ns = test_ns("spsc");
    let id = edge_id();
    let owner = Arc::new(MappedCredit::create_owned(&ns, &id, DEPTH).unwrap());
    let published = Arc::new(AtomicU64::new(0));
    let drained = Arc::new(AtomicU64::new(0));
    // The oracle for the invariant: the highest `outstanding` the producer ever
    // created. A `>` instead of `>=` in the gate, or a missing claim, shows up
    // here as a value above DEPTH.
    //
    // It is captured ATOMICALLY AT THE PUBLISH — `record_published` returns the
    // count its own RMW created — and NOT by a later `outstanding()` read.
    // The read is a DIFFERENT instant and the consumer thread is draining
    // throughout, so an over-depth count could be drained away between the
    // publish and the sample and the violation would go unseen; the RMW's own
    // result cannot be raced away, whatever the consumer does next.
    let peak = Arc::new(AtomicU64::new(0));

    let producer = {
        let credit = Arc::clone(&owner);
        let published = Arc::clone(&published);
        let peak = Arc::clone(&peak);
        thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_secs(60);
            for _ in 0..N {
                // The pre-fire gate: spin until there is credit.
                while !credit.try_claim() {
                    if Instant::now() >= deadline {
                        panic!(
                            "producer starved: outstanding={} depth={}",
                            credit.outstanding(),
                            credit.depth()
                        );
                    }
                    thread::yield_now();
                }
                let outstanding_at_publish = credit.record_published();
                published.fetch_add(1, Ordering::Relaxed);
                peak.fetch_max(outstanding_at_publish, Ordering::Relaxed);
            }
        })
    };

    let consumer = {
        let ns = ns.clone();
        let id = id.clone();
        let drained = Arc::clone(&drained);
        thread::spawn(move || {
            let credit = MappedCredit::open_unowned(&ns, &id).expect("peer opens");
            let deadline = Instant::now() + Duration::from_secs(60);
            while drained.load(Ordering::Relaxed) < N {
                let available = credit.outstanding();
                if available == 0 {
                    if Instant::now() >= deadline {
                        panic!(
                            "consumer starved at {} of {N}",
                            drained.load(Ordering::Relaxed)
                        );
                    }
                    thread::yield_now();
                    continue;
                }
                // Never drain more than the producer has actually published —
                // a real consumer drains frames it received.
                let take = available.min(N - drained.load(Ordering::Relaxed));
                credit.record_drained(take);
                drained.fetch_add(take, Ordering::Relaxed);
            }
        })
    };

    producer.join().expect("producer panicked");
    consumer.join().expect("consumer panicked");

    assert_eq!(
        published.load(Ordering::Relaxed),
        N,
        "every frame published"
    );
    assert_eq!(drained.load(Ordering::Relaxed), N, "every frame drained");
    assert_eq!(owner.outstanding(), 0, "the word settles back to empty");
    assert!(
        peak.load(Ordering::Relaxed) <= u64::from(DEPTH),
        "the producer must NEVER exceed the declared depth — peak was {}, depth {DEPTH}",
        peak.load(Ordering::Relaxed)
    );
}

/// The word survives a producer that parks and a consumer that frees credit
/// across a real SHM boundary under repeated rounds — the shape the live
/// loop drives. Bounded per round so a lost wake fails attributably rather
/// than hanging.
///
/// A lost wake here is ABSORBED by the fallback cap (each round re-derives
/// `try_claim` after a 500 ms park and loops), so this arm pins LIVENESS and the
/// guard lifecycle, NOT the wake itself — the wake pin is
/// `the_sb_litmus_hammer_bounds_every_round`.
#[test]
fn repeated_park_and_release_rounds_stay_bounded() {
    if !credit_wake_word_primitive_available() {
        eprintln!("no wake primitive on this host — skipping (fallback tier)");
        return;
    }
    const ROUNDS: usize = 25;
    let ns = test_ns("rounds");
    let id = edge_id();
    let consumer = Arc::new(MappedCredit::create_owned(&ns, &id, 1).unwrap());
    let parked_rounds = Arc::new(AtomicU64::new(0));

    let producer = {
        let ns = ns.clone();
        let id = id.clone();
        let parked_rounds = Arc::clone(&parked_rounds);
        thread::spawn(move || {
            let credit = MappedCredit::open_unowned(&ns, &id).expect("peer opens");
            for _ in 0..ROUNDS {
                // Claim, publish, then park until the consumer frees room.
                while !credit.try_claim() {
                    let _g = ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0))
                        .expect("index 0 in-mask");
                    let snap = credit.wake_seq_snapshot();
                    if credit.try_claim() {
                        break;
                    }
                    credit.park_wait_credit(snap, Duration::from_millis(500));
                    parked_rounds.fetch_add(1, Ordering::Relaxed);
                }
                credit.record_published();
            }
        })
    };

    for round in 0..ROUNDS {
        await_until(
            Duration::from_secs(10),
            &format!("round {round}'s frame to be published"),
            || consumer.outstanding() > 0,
        );
        consumer.record_drained(1);
    }
    producer.join().expect("producer panicked");

    assert_eq!(consumer.outstanding(), 0, "every round's frame was drained");
    assert_eq!(
        consumer.parked_mask(),
        0,
        "every guard released its bit — no leaked parker taxes future drains"
    );
    // Anti-vacuity: the producer really did park at least once (depth 1 against
    // a lockstep consumer makes that near-certain, and a zero here would mean
    // the test never exercised the park at all).
    assert!(
        parked_rounds.load(Ordering::Relaxed) > 0,
        "the producer never actually parked — this arm proved nothing"
    );
}

// ----------------------------------------------- the cross-process wake ----

/// The CROSS-ADDRESS-SPACE wake proof.
///
/// Every other park arm in this file runs its producer on a THREAD of this
/// process, which shares one address space and one `libc` — so two whole classes
/// of defect are invisible to them, and both are one identifier wide:
///
/// - a Linux `FUTEX_WAKE`/`FUTEX_WAIT` that lost its `SHARED` (i.e. gained
///   `_PRIVATE`) keys on the process's own address space, so two threads still
///   rendezvous perfectly while two PROCESSES never do;
/// - a macOS `OS_SYNC_WAKE_BY_ADDRESS_SHARED` / `OS_SYNC_WAIT_ON_ADDRESS_SHARED`
///   flipped to the `NONE` variant, with exactly the same shape.
///
/// A credit word exists PRECISELY to be operated by two processes, so a wake
/// that only works in one is the whole feature silently gone. Ported from
/// `barrier_test.rs::two_subprocess_rendezvous_over_one_mapped_barrier` (the
/// stub-removal pin) and, like it, NOT `#[ignore]`d: it runs in the
/// normal suite on every Unix.
///
/// Self-re-exec via `std::process::Command` (NOT `fork` — fork-after-threads is
/// unsafe): the test checks `CHILD_ENV` at its TOP and, when set, acts as the
/// CHILD participant and `exit`s.
///
/// # The ordering discipline, and why the obvious version proves NOTHING
///
/// Gating the parent's drain on the child's parked BIT alone — set by
/// `ParkedEdgeGuard::enter`, i.e. BEFORE the child takes its snapshot and
/// long before it enters the kernel — is not enough: under a
/// `SHARED`→`NONE` revert, it passes in **0.04 s**: the drain lands
/// inside the bit-set→block window every time, the kernel's compare
/// against the stale snapshot ends the park immediately, and the wake is
/// never exercised: a green arm that never runs the code it is named for.
///
/// Two things fix it, and both are load-SAFE (they can only make the run more
/// probative; neither can invert a verdict):
///
/// 1. **The child WARMS the wake backend before the timed window.** `os_sync` is
///    resolved lazily — two `dlsym`s and a kill-switch env read behind
///    `OnceLock`s — and that resolution used to happen INSIDE the first
///    `park_wait_credit`, i.e. after the bit was already published. Tens of µs
///    of slack, handed to the parent, on the exact path being raced. Calling
///    `credit_wake_word_primitive_available()` up front pays it earlier.
/// 2. **A separate READY word, published AFTER the snapshot.** The signal the
///    parent waits on is a second credit segment the child bumps once its
///    snapshot is taken, so the drain is ordered strictly after the value the
///    kernel will compare against — the same discipline as
///    `a_spurious_ring_wakes_the_producer_who_re_derives_no_room`. A short
///    settle then covers the last few instructions before the syscall; it can
///    only cost a vacuous pass, never a false failure, which is why a sleep is
///    admissible HERE and not as a substitute for the signal.
#[cfg(unix)]
mod subprocess_harness {
    use super::{await_until, edge_id, test_ns};
    use cerulion_core::credit::{
        credit_wake_word_primitive_available, MappedCredit, ParkedEdgeGuard, ProducerSlot,
    };
    use std::io;
    use std::process::{Child, Command, ExitStatus};
    use std::time::{Duration, Instant};

    const CHILD_ENV: &str = "CERULION_CREDIT_CHILD";
    const NS_ENV: &str = "CERULION_CREDIT_NS";
    const ID_ENV: &str = "CERULION_CREDIT_ID";

    /// The child's park cap. Deliberately far LONGER than the wake it is
    /// proving, so a lost wake shows up as a wall the parent can bound rather
    /// than as a failure the child could report by luck.
    const CHILD_PARK_CAP: Duration = Duration::from_secs(10);
    /// Slack between the child's READY signal and the parent's drain, covering
    /// the handful of instructions between the snapshot and the syscall. Safe
    /// in one direction only: too short costs a vacuous pass (the kernel's
    /// compare catches the drain), never a wrong verdict.
    const PRE_DRAIN_SETTLE: Duration = Duration::from_millis(50);

    /// Child exit codes — the whole reporting channel, so each one names a
    /// distinct thing that went wrong.
    const EXIT_OK: i32 = 0;
    /// The word was NOT full when the child arrived: the parent drains only
    /// after the READY signal, which the child raises later, so this cannot be
    /// a race — it means the setup did not fill the word.
    const EXIT_NOT_FULL: i32 = 3;
    /// The park returned but the re-derive still saw no room: the drain never
    /// became visible across the page.
    const EXIT_NO_ROOM: i32 = 4;
    /// The park reported NOT-performed — no primitive in the child process,
    /// which the parent's own gate should have excluded.
    const EXIT_NOT_PERFORMED: i32 = 5;

    /// The READY word's edge id — a SECOND segment, used purely as a
    /// cross-process flag, so the parent can order its drain after the child's
    /// snapshot without inventing a harness IPC mechanism.
    fn ready_id() -> String {
        format!("{}\u{1f}ready", edge_id())
    }

    /// RAII child: `Drop` SIGKILLs then reaps, so a parent unwind can never
    /// orphan a spawned process (`std::process::Child` does not kill on drop).
    struct ChildGuard {
        child: Child,
    }

    impl Drop for ChildGuard {
        fn drop(&mut self) {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }

    /// Bounded join — a wedged child fails attributably instead of hanging CI
    /// to the job timeout (the `shm_ring_test` lesson).
    fn wait_bounded(guard: &mut ChildGuard, budget: Duration) -> ExitStatus {
        let end = Instant::now() + budget;
        loop {
            match guard.child.try_wait() {
                Ok(Some(status)) => return status,
                Ok(None) if Instant::now() < end => std::thread::sleep(Duration::from_millis(10)),
                Ok(None) => panic!("child did not exit within {budget:?} (guard will SIGKILL)"),
                Err(e) => panic!("try_wait on child failed: {e}"),
            }
        }
    }

    /// `open_unowned` with a bounded retry: the parent creates before spawning,
    /// but exec startup can still race the create on a loaded box.
    fn open_with_retry(ns: &str, id: &str) -> MappedCredit {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match MappedCredit::open_unowned(ns, id) {
                Ok(c) => return c,
                Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(5)),
                Err(e) => panic!("child could not open the credit segment within 10s: {e}"),
            }
        }
    }

    /// The re-exec'd CHILD: open the shared word, confirm it is FULL, park on
    /// it, and report through the exit code. Never returns.
    ///
    /// ORDERING is chosen, not incidental — see the module docs. The full-check
    /// runs before anything is signalled, so `EXIT_NOT_FULL` is a real setup
    /// failure rather than a harness race; the guard is entered BEFORE the
    /// snapshot (the SB-litmus protocol the production park requires); and the
    /// READY word is bumped AFTER the snapshot, so the parent's drain is
    /// strictly later than the value the kernel compares against.
    ///
    /// The exit code is computed INSIDE a scope that drops the guard, so the
    /// parent can also assert the `parked` bit was RELEASED — `exit` does not
    /// run destructors, so an inline `exit` would leave the bit set and make
    /// that assertion unwritable.
    // `std::process::exit` is disallowed repo-wide (`clippy.toml`): a LIBRARY that
    // exits steals the caller's cleanup and swallows the error no test could then
    // observe. This function is the other case — the body of a SELF-RE-EXEC CHILD
    // process, which is a process entrypoint by construction (it returns `!`) and
    // whose exit code IS the channel the parent reads its verdict from. Scoped to
    // this function rather than the file, so the ban stays armed for the parent
    // half of the harness and for every test in it.
    #[allow(clippy::disallowed_methods)]
    fn credit_child() -> ! {
        // Resolve the os_sync backend + kill switch BEFORE the timed window, or
        // the two dlsym calls behind it land between the snapshot and the
        // syscall and hand the parent the race.
        let _warm = credit_wake_word_primitive_available();
        let ns = std::env::var(NS_ENV).expect(NS_ENV);
        let id = std::env::var(ID_ENV).expect(ID_ENV);
        let credit = open_with_retry(&ns, &id);
        let ready = open_with_retry(&ns, &ready_id());
        if credit.try_claim() {
            std::process::exit(EXIT_NOT_FULL);
        }
        let code = {
            let _g =
                ParkedEdgeGuard::enter(&credit, ProducerSlot::new(0)).expect("index 0 in-mask");
            let snap = credit.wake_seq_snapshot();
            ready.record_published();
            if !credit.park_wait_credit(snap, CHILD_PARK_CAP) {
                EXIT_NOT_PERFORMED
            } else if credit.try_claim() {
                EXIT_OK
            } else {
                EXIT_NO_ROOM
            }
        };
        std::process::exit(code);
    }

    fn spawn_child(ns: &str, id: &str) -> io::Result<ChildGuard> {
        let exe = std::env::current_exe().expect("current_exe");
        Ok(ChildGuard {
            child: Command::new(exe)
                .args([
                    "--exact",
                    "subprocess_harness::a_child_process_parks_on_the_shared_word_and_is_woken_by_the_parents_drain",
                    "--nocapture",
                ])
                .env(CHILD_ENV, "1")
                .env(NS_ENV, ns)
                .env(ID_ENV, id)
                .spawn()?,
        })
    }

    /// A REAL child PROCESS parks on the shared credit word and the PARENT's
    /// drain wakes it — over a real `MAP_SHARED` page and two page tables.
    ///
    /// The load-bearing oracle is the WALL from the parent's `record_drained`
    /// to the child's exit, bounded at 3 s against the child's 10 s cap. That
    /// 7 s margin is what keeps it load-safe: contention can only lengthen the
    /// wake, and a lost cross-process wake does not merely arrive late — it
    /// arrives at the CAP, which no amount of scheduling noise reaches from
    /// below. (The child's exit code alone cannot carry this: on a lost wake it
    /// still exits 0 after the cap, because by then the parent's drain IS
    /// visible on the page — which is exactly why the wall is asserted.)
    #[test]
    fn a_child_process_parks_on_the_shared_word_and_is_woken_by_the_parents_drain() {
        if std::env::var(CHILD_ENV).is_ok() {
            credit_child(); // re-exec'd participant — never returns
        }
        if !credit_wake_word_primitive_available() {
            eprintln!("no wake primitive on this host — skipping (fallback tier)");
            return;
        }
        let ns = test_ns("xproc_wake");
        let id = edge_id();
        let owner = MappedCredit::create_owned(&ns, &id, 1).expect("owner create");
        let ready = MappedCredit::create_owned(&ns, &ready_id(), 1).expect("ready create");
        // Fill it, so the child genuinely has no credit and must park.
        owner.record_published();
        assert!(
            !owner.try_claim(),
            "the word is full before the child opens"
        );

        let mut kid = spawn_child(&ns, &id).expect("spawn child");
        // The child raises READY once its snapshot is taken — no bias sleep
        // decides this ordering.
        await_until(
            Duration::from_secs(20),
            "the child process to signal that it has snapshotted the wake word",
            || ready.outstanding() > 0,
        );
        // …then a short settle for the last instructions before the syscall.
        std::thread::sleep(PRE_DRAIN_SETTLE);
        assert_ne!(
            owner.parked_mask(),
            0,
            "the child's parked bit must be visible across the page before the drain"
        );

        let t0 = Instant::now();
        owner.record_drained(1);
        let status = wait_bounded(&mut kid, Duration::from_secs(30));
        let wall = t0.elapsed();

        assert_eq!(
            status.code(),
            Some(EXIT_OK),
            "child exited {:?} (3 = word not full, 4 = re-derive saw no room, \
             5 = park not performed)",
            status.code()
        );
        assert!(
            wall < Duration::from_secs(3),
            "the parent's drain must WAKE the child's cross-process park far inside its \
             {CHILD_PARK_CAP:?} cap — took {wall:?}. A wake keyed on this process's own \
             address space (a PRIVATE futex / a NONE os_sync flag) rides to the cap instead."
        );
        assert_eq!(
            owner.parked_mask(),
            0,
            "the child's ParkedEdgeGuard released its bit before exiting, so no stale bit \
             taxes a future drain"
        );
    }
}

/// A slot at or beyond `PARKED_MASK_BITS` claims NO bit, and `park_enter` says
/// so — the mask observed WHILE parked, the only moment a claim is visible.
///
/// This replaces an assertion in `credit_block_iox2_test` that read the mask
/// AFTER the park had exited. `ParkedEdgeGuard` clears the bit on the way out,
/// so that read was 0 regardless of what the slot claimed: an implementation
/// aliasing slot 32 onto bit 0 and then clearing it passed. Here the mask is
/// read BETWEEN enter and exit, so an aliasing implementation FAILS.
///
/// `park_exit` is deliberately NOT called on the beyond-mask slots: `park_enter`
/// returns `false` for them, so `ParkedEdgeGuard` never mints a guard and the
/// exit never runs. Calling it anyway trips that method's `debug_assert`,
/// catching the mistake immediately.
///
/// The in-mask arms are the anti-vacuity control: without them, an
/// implementation that never set ANY bit would satisfy the beyond-mask pin.
#[test]
fn a_slot_beyond_the_mask_claims_no_parked_bit() {
    let ns = test_ns("beyond_mask");
    let id = edge_id();
    let w = MappedCredit::create_owned(&ns, &id, 4).unwrap();

    // CONTROL — slot 0 claims EXACTLY bit 0 while parked, and gives it back.
    assert!(
        w.park_enter(ProducerSlot::new(0)),
        "CONTROL: an in-mask slot must be accepted"
    );
    assert_eq!(
        w.parked_mask(),
        1,
        "CONTROL: slot 0 must claim exactly bit 0 while parked — if this is 0 the \
         mask is read at the wrong moment and every pin below is vacuous"
    );
    w.park_exit(ProducerSlot::new(0));
    assert_eq!(w.parked_mask(), 0, "park_exit gives the bit back");

    // CONTROL — the LAST in-mask slot claims its own top bit, not a low one.
    let last = ProducerSlot::new(PARKED_MASK_BITS - 1);
    assert!(
        w.park_enter(last),
        "CONTROL: the last in-mask slot is accepted"
    );
    assert_eq!(
        w.parked_mask(),
        1u32 << (PARKED_MASK_BITS - 1),
        "CONTROL: the last in-mask slot claims the TOP bit"
    );
    w.park_exit(last);
    assert_eq!(w.parked_mask(), 0);

    // THE PIN — one past the mask owns no bit, and enter REFUSES it.
    let beyond = ProducerSlot::new(PARKED_MASK_BITS);
    assert!(
        !beyond.fits_mask(),
        "slot {PARKED_MASK_BITS} must not fit a {PARKED_MASK_BITS}-wide mask"
    );
    assert!(
        !w.park_enter(beyond),
        "park_enter must REFUSE a beyond-mask slot — its false return is what \
         stops ParkedEdgeGuard minting a guard that would later clear bit 0"
    );
    assert_eq!(
        w.parked_mask(),
        0,
        "a slot at PARKED_MASK_BITS must claim NO bit — aliasing it onto bit 0 \
         would steal an in-mask producer's wake"
    );

    // And far beyond, where `1u32 << n` would WRAP (Rust masks the shift count
    // by the type width, so `1u32 << 32` is `1u32 << 0` — bit 0 again).
    let far = ProducerSlot::new(PARKED_MASK_BITS + 1);
    assert!(
        !w.park_enter(far),
        "park_enter must refuse a far-beyond slot"
    );
    assert_eq!(
        w.parked_mask(),
        0,
        "a far-beyond-mask slot must claim NO bit — a wrapping shift would alias \
         it onto a low bit"
    );

    // A genuinely parked in-mask producer is UNDISTURBED by the refused slots:
    // the failure mode being prevented is theft of ANOTHER producer's bit.
    assert!(w.park_enter(ProducerSlot::new(0)));
    assert!(!w.park_enter(beyond));
    assert_eq!(
        w.parked_mask(),
        1,
        "a refused beyond-mask enter must not touch a live in-mask bit"
    );
    w.park_exit(ProducerSlot::new(0));
}

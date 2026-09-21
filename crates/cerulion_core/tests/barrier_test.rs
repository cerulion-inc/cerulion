// SPDX-License-Identifier: AGPL-3.0-only
//! Hermetic, parallel-safe tests for the lock-free,
//! count-down sense-reversing [`BarrierShared`] state machine
//! (`cerulion_core::barrier`).
//!
//! These model N participants with `Arc<BarrierShared>` + `std::thread` — a
//! FAITHFUL test of the lock-free algorithm, since the SAME atomics will later
//! live directly in an mmap'd SHM page shared across processes. No iceoryx2, no
//! SHM. Parallel-safe EXCEPT the marked os_sync trio (the availability
//! probe, the kill-switch test, and the wake-latency A/B), which are
//! `#[serial]`: they touch the process-global `CERULION_BARRIER_OS_SYNC` env
//! window and/or the os_sync `OnceLock` caches — every other test is
//! self-contained and parallel-safe.
//!
//! Where a test waits, it uses `wait_open_capped` — a yielding spin bounded by a
//! WALL-CLOCK deadline that PANICS loudly rather than hanging CI if the barrier
//! never opens.

use cerulion_core::barrier::{
    barrier_os_sync_backend_available, barrier_wake_tier, ArriveOutcome, BarrierShared,
    DeadPeerDrop, DropAttempt, MappedBarrier, WaitOutcome,
};
use serial_test::serial;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Wait until generation `my_gen` opens, yielding to the laggard each iteration
/// and failing loudly if it is not open within a WALL-CLOCK deadline.
///
/// A correct barrier opens almost immediately; the deadline exists so a BROKEN
/// barrier (a lost / duplicated opening) fails loudly instead of hanging CI.
/// `yield_now` cedes the core to the participant that still has to arrive, and a
/// wall-clock bound (rather than an iteration count) is robust to per-iteration
/// speed variance on an oversubscribed CI runner. `Instant::now` is far costlier
/// than the atomic load, so the deadline is only checked every Nth iteration.
fn wait_open_capped(b: &BarrierShared, my_gen: u64) {
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut spins: u32 = 0;
    while !b.is_open(my_gen) {
        thread::yield_now();
        spins = spins.wrapping_add(1);
        if spins.is_multiple_of(1024) && Instant::now() >= deadline {
            panic!(
                "barrier generation {my_gen} did not open within 30s \
                 (likely a lost/duplicated opening — the barrier is stuck)"
            );
        }
    }
}

/// Test 1 — multi-generation lockstep. Pins that, with 2 participants over 3
/// generations, EACH generation opens EXACTLY once and the opens happen in order
/// `[1, 2, 3]`. The oracle is the hand-written vector `[1, 2, 3]` (NOT a
/// self-compare): exactly one thread per generation observes `Opened`, the other
/// `Pending`, so collecting every `Opened(_)` across both threads must yield
/// precisely those three values, one each.
#[test]
fn lockstep_two_participants_three_generations_open_in_order() {
    let b = Arc::new(BarrierShared::new(2));
    let mut handles = Vec::new();
    for _ in 0..2 {
        let b = Arc::clone(&b);
        handles.push(thread::spawn(move || {
            let mut opened = Vec::new();
            for g in 0..3u64 {
                if let ArriveOutcome::Opened(ng) = b.arrive(g) {
                    opened.push(ng);
                }
                wait_open_capped(&b, g);
            }
            opened
        }));
    }

    let mut all_opened: Vec<u64> = handles
        .into_iter()
        .flat_map(|h| h.join().expect("thread panicked"))
        .collect();
    all_opened.sort_unstable();

    assert_eq!(
        all_opened,
        vec![1, 2, 3],
        "each generation must open exactly once, in order 1,2,3 (hand oracle)"
    );
    assert_eq!(
        b.current_generation(),
        3,
        "three generations completed → generation ends at 3"
    );
}

/// Larger-cohort lockstep. 8 participants over 3 generations; same
/// hand oracle (`[1, 2, 3]`) — each generation opens exactly once regardless of
/// cohort width, and the final generation is 3.
#[test]
fn lockstep_eight_participants_three_generations() {
    const N: u32 = 8;
    let b = Arc::new(BarrierShared::new(N));
    let mut handles = Vec::new();
    for _ in 0..N {
        let b = Arc::clone(&b);
        handles.push(thread::spawn(move || {
            let mut opened = Vec::new();
            for g in 0..3u64 {
                if let ArriveOutcome::Opened(ng) = b.arrive(g) {
                    opened.push(ng);
                }
                wait_open_capped(&b, g);
            }
            opened
        }));
    }

    let mut all_opened: Vec<u64> = handles
        .into_iter()
        .flat_map(|h| h.join().expect("thread panicked"))
        .collect();
    all_opened.sort_unstable();

    assert_eq!(
        all_opened,
        vec![1, 2, 3],
        "each generation opened exactly once across an 8-wide cohort (hand oracle)"
    );
    assert_eq!(b.current_generation(), 3);
}

/// Test 2 — fast re-entry / count-down reuse race (the LOAD-BEARING pin for the
/// single per-generation `remaining` counter). Three participants loop AS FAST
/// AS POSSIBLE across many generations with no synchronization beyond the barrier
/// itself, so the unique opener of generation `G` (which re-arms `remaining`
/// from `expected` then bumps the generation) immediately re-enters generation
/// `G+1` and `fetch_sub`s that very counter WHILE the laggards are still leaving
/// generation `G` — exercising the count-down reuse boundary thousands of times.
///
/// The correctness this pins: the re-arm is sequenced BEFORE the generation
/// publish and waiters gate on the SENSE (`generation`), so a fast participant
/// entering `G+1` always observes the re-armed counter (never a stale / wiped
/// value). Oracle: every generation `1..=ROUNDS` opens EXACTLY once (sorted
/// `Opened` values == `[1, 2, ..., ROUNDS]`) and the final generation is exactly
/// the number of completed rounds — no lost or duplicated opening.
#[test]
fn fast_reentry_countdown_reuse_has_no_lost_or_duplicate_open() {
    const N: u32 = 3;
    const ROUNDS: u64 = 5_000;

    let b = Arc::new(BarrierShared::new(N));
    let mut handles = Vec::new();
    for _ in 0..N {
        let b = Arc::clone(&b);
        handles.push(thread::spawn(move || {
            let mut opened = Vec::new();
            for g in 0..ROUNDS {
                if let ArriveOutcome::Opened(ng) = b.arrive(g) {
                    opened.push(ng);
                }
                wait_open_capped(&b, g);
            }
            opened
        }));
    }

    let mut all_opened: Vec<u64> = handles
        .into_iter()
        .flat_map(|h| h.join().expect("thread panicked"))
        .collect();
    all_opened.sort_unstable();

    let expected: Vec<u64> = (1..=ROUNDS).collect();
    assert_eq!(
        all_opened, expected,
        "every generation 1..=ROUNDS must open exactly once — no lost/duplicated opening \
         (a count-down reuse collision would drop or double an opening)"
    );
    assert_eq!(
        b.current_generation(),
        ROUNDS,
        "final generation equals the number of completed rounds"
    );
}

/// Test 3 — single participant (`expected == 1`). Every `arrive` is immediately
/// the last arriver (counts `remaining` 1→0), so each opens at once and the
/// generation advances by one.
#[test]
fn single_participant_every_arrive_opens_immediately() {
    let b = BarrierShared::new(1);
    for g in 0..5u64 {
        assert_eq!(
            b.arrive(g),
            ArriveOutcome::Opened(g + 1),
            "a sole participant opens its generation immediately"
        );
        assert!(b.is_open(g), "generation {g} is open right after it opened");
    }
    assert_eq!(b.current_generation(), 5);
}

/// `drop_participant` opens a generation EXACTLY once under a
/// START-GATED drop/arrive race, looped. Each round: a fresh `new(3)`, two live
/// arrivers + the drop all released TOGETHER by a `std::sync::Barrier::new(3)`
/// start-gate so the three race for real. Asserts EVERY round that exactly one of
/// the three callers observes `Opened(1)` (the count-down uniqueness the two-slot
/// design could NOT guarantee — there a live arrival and the drop could BOTH
/// open, the second wiping a fast next-generation arrival), plus
/// `current_generation()==1` and `expected()==2`.
///
/// The start-gate drives a REAL split between the two opener branches — "the drop
/// counts `remaining` to 0" and "a live arrive counts it to 0 (drop returns
/// Pending)" — and the test FAILS LOUDLY if either branch never ran across the
/// 5000 rounds (an un-gated single shot almost always exercises only one branch).
#[test]
fn drop_participant_opens_exactly_once_under_gated_race() {
    const ROUNDS: usize = 5_000;
    let mut drop_opened = 0usize;
    let mut arrive_opened = 0usize;

    for _ in 0..ROUNDS {
        let b = Arc::new(BarrierShared::new(3));
        let start = Arc::new(std::sync::Barrier::new(3));

        let mut handles = Vec::new();
        for _ in 0..2 {
            let b = Arc::clone(&b);
            let start = Arc::clone(&start);
            handles.push(thread::spawn(move || {
                start.wait();
                b.arrive(0)
            }));
        }

        // The drop fires on this thread, gated by the same start-barrier.
        start.wait();
        let drop_out = b.drop_participant(0);

        let arrive_outs: Vec<ArriveOutcome> = handles
            .into_iter()
            .map(|h| h.join().expect("arriver panicked"))
            .collect();

        let opened: Vec<u64> = arrive_outs
            .iter()
            .chain(std::iter::once(&drop_out))
            .filter_map(|o| match o {
                ArriveOutcome::Opened(g) => Some(*g),
                ArriveOutcome::Pending => None,
            })
            .collect();
        assert_eq!(
            opened,
            vec![1],
            "generation 0 must open EXACTLY once across the 2 arrives + 1 drop"
        );
        assert_eq!(b.current_generation(), 1);
        assert_eq!(b.expected(), 2);

        if matches!(drop_out, ArriveOutcome::Opened(_)) {
            drop_opened += 1;
        } else {
            arrive_opened += 1;
        }
    }

    // Both opener branches must be exercised by the start-gated race.
    assert!(
        drop_opened > 0,
        "the drop-opens branch never ran ({drop_opened}/{ROUNDS}) — race not exercised"
    );
    assert!(
        arrive_opened > 0,
        "the arrive-opens branch (drop returns Pending) never ran ({arrive_opened}/{ROUNDS})"
    );
    eprintln!(
        "drop/arrive branch split: drop_opened={drop_opened} arrive_opened={arrive_opened} of {ROUNDS}"
    );
}

/// Drop then survivors continue, with a correctly-RESIZED re-arm.
/// 3 expected, 2 live; the dead 3rd is dropped at gen 0. After it opens, the 2
/// survivors run gen 1 AND gen 2 — proving the post-drop re-arm is sized for 2
/// (a stale re-arm width of 3 would deadlock gen 1, which an accessor check
/// alone would miss). Single-threaded — the algorithm is order-independent.
#[test]
fn drop_then_survivors_continue_with_resized_rearm() {
    let b = BarrierShared::new(3);

    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "gen0: 1 of 3 live");
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "gen0: 2 of 3 live");
    assert_eq!(
        b.drop_participant(0),
        ArriveOutcome::Opened(1),
        "drop of the dead 3rd completes gen 0 for the 2 survivors"
    );
    assert_eq!(b.expected(), 2, "expected resized to the 2 survivors");

    // gen 1 with the 2 survivors — the re-arm must be width-2 (not a stale 3).
    assert_eq!(b.arrive(1), ArriveOutcome::Pending, "gen1: 1 of 2");
    assert_eq!(b.arrive(1), ArriveOutcome::Opened(2), "gen1: 2 of 2 opens");
    // gen 2 with the 2 survivors — re-arm still width-2.
    assert_eq!(b.arrive(2), ArriveOutcome::Pending, "gen2: 1 of 2");
    assert_eq!(b.arrive(2), ArriveOutcome::Opened(3), "gen2: 2 of 2 opens");
    assert_eq!(b.current_generation(), 3);
}

/// `drop_participant` on an ALREADY-OPEN generation is a Pending no-op.
/// `new(1)`; `arrive(0)` opens gen 1; a drop targeting the already-open gen 0
/// must early-return `Pending`, leave `expected()` unchanged, and NOT advance the
/// generation (it touches no state).
#[test]
fn drop_on_already_open_generation_is_a_pending_noop() {
    let b = BarrierShared::new(1);
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
    assert_eq!(b.current_generation(), 1);

    assert_eq!(
        b.drop_participant(0),
        ArriveOutcome::Pending,
        "drop for an already-open generation early-returns Pending"
    );
    assert_eq!(
        b.expected(),
        1,
        "expected unchanged (early return before the decrement)"
    );
    assert_eq!(b.current_generation(), 1, "generation not advanced");
}

/// The drop-is-the-OPENER branch (deterministic, single-threaded).
/// `new(2)`; one live arrives (`Pending`); dropping the 2nd counts `remaining` to
/// 0 → the drop opens (`Opened(1)`), and `expected` is now 1.
#[test]
fn drop_is_the_opener_branch() {
    let b = BarrierShared::new(2);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "1 of 2 live");
    assert_eq!(
        b.drop_participant(0),
        ArriveOutcome::Opened(1),
        "the drop's fetch_sub hits 0 → drop is the unique opener"
    );
    assert_eq!(b.expected(), 1);
    assert_eq!(b.current_generation(), 1);
}

/// The drop-is-PENDING branch (deterministic, single-threaded).
/// `new(3)`; one live arrives (`Pending`); the drop does NOT hit 0 (one live
/// still out) → `Pending`, `expected` now 2; the final live arrival opens.
#[test]
fn drop_is_the_pending_branch() {
    let b = BarrierShared::new(3);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "1 of 3 live");
    assert_eq!(
        b.drop_participant(0),
        ArriveOutcome::Pending,
        "the drop's fetch_sub does NOT hit 0 (one live still out) → Pending"
    );
    assert_eq!(b.expected(), 2, "one live participant still out");
    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Opened(1),
        "the final live arrival counts remaining to 0 → opens"
    );
    assert_eq!(b.current_generation(), 1);
}

/// `arrive` on an ALREADY-OPEN generation is an idempotent Pending
/// no-op (pins the stale-generation guard). `new(1)`; `arrive(0)` opens gen 1; a
/// SECOND `arrive(0)` (a stale / duplicate arrival for the already-open gen 0)
/// must return `Pending`. The LOAD-BEARING pin is the `Pending` return: WITHOUT
/// the guard, the duplicate would `fetch_sub` the live counter and re-run
/// `open_generation(0)`, which stores `my_gen + 1 = 1` — so it re-opens
/// generation 1 and returns `Opened(1)` (NOT gen 2). `current_generation()`
/// stays 1 EITHER WAY, so that check does NOT catch guard removal; the `Pending`
/// vs `Opened(1)` return is what does.
#[test]
fn arrive_on_already_open_generation_is_idempotent_pending() {
    let b = BarrierShared::new(1);
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
    assert_eq!(b.current_generation(), 1);

    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Pending,
        "a stale arrival for the already-open gen 0 is an idempotent no-op"
    );
    assert_eq!(
        b.current_generation(),
        1,
        "generation is 1 (holds with OR without the guard — the Pending return above is the real pin)"
    );
}

/// Test 5 (degenerate) — `expected == 0`. A zero-participant barrier is vacuously
/// complete: each `arrive` opens immediately and ADVANCES the generation WITHOUT
/// touching `remaining` (so it never underflows). `drop_participant` on such a
/// barrier hits the underflow guard (`remaining` already 0, `expected == 0` so
/// the loud desync signal is gated OFF) → restores the counter and returns
/// `Pending`, and `expected` SATURATES at 0. The tail re-arms via
/// `set_expected(2)` and runs a normal 2-participant generation to prove the
/// barrier is NOT wedged (the underflow guard restored `remaining` rather than
/// leaving it at `u32::MAX`).
#[test]
fn expected_zero_is_vacuously_open_and_never_underflows() {
    let b = BarrierShared::new(0);
    assert_eq!(b.expected(), 0);

    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Opened(1),
        "exp==0: first arrive opens immediately (vacuously complete)"
    );
    assert!(b.is_open(0));
    assert_eq!(b.current_generation(), 1);

    assert_eq!(
        b.arrive(1),
        ArriveOutcome::Opened(2),
        "exp==0: subsequent arrive keeps advancing"
    );
    assert_eq!(b.current_generation(), 2);

    // drop on an exp==0 barrier: `remaining` already 0 → underflow guard fires
    // (loud signal gated off since expected==0) → restore + Pending.
    assert_eq!(
        b.drop_participant(2),
        ArriveOutcome::Pending,
        "exp==0: drop hits the underflow guard → Pending (no spurious open)"
    );
    assert_eq!(
        b.expected(),
        0,
        "saturating decrement floors at 0 — no underflow to u32::MAX"
    );
    assert_eq!(
        b.current_generation(),
        2,
        "drop did not advance the generation"
    );

    // Resume normal operation — proves the underflow guard restored `remaining`
    // (a corrupted u32::MAX would never count down to 0 here).
    b.set_expected(2);
    assert_eq!(b.expected(), 2);
    assert_eq!(b.arrive(2), ArriveOutcome::Pending, "1 of 2");
    assert_eq!(b.arrive(2), ArriveOutcome::Opened(3), "2 of 2 opens");
    assert_eq!(
        b.current_generation(),
        3,
        "barrier resumed normal operation"
    );
}

/// Test 6 — accessor sanity across a couple of transitions. Exercises
/// `current_generation` / `expected` / `is_open` / `set_expected` directly (a
/// single thread arrives twice to simulate two participants — the algorithm does
/// not care which thread arrives). `set_expected(1)` is called at the gen-1
/// BOUNDARY (no in-flight arrivals — `remaining == expected` there), retargeting
/// the barrier to a single participant.
#[test]
fn accessors_reflect_transitions() {
    let b = BarrierShared::new(2);
    assert_eq!(b.expected(), 2);
    assert_eq!(b.current_generation(), 0);
    assert!(!b.is_open(0));

    // First of two arrivals — not yet open.
    assert_eq!(b.arrive(0), ArriveOutcome::Pending);
    assert_eq!(b.current_generation(), 0, "not open until both arrive");
    assert!(!b.is_open(0));

    // Second arrival opens generation 0.
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
    assert!(b.is_open(0));
    assert_eq!(b.current_generation(), 1);

    // Retarget at the gen-1 boundary (no arrivals yet) → single participant.
    b.set_expected(1);
    assert_eq!(b.expected(), 1);
    assert_eq!(
        b.arrive(1),
        ArriveOutcome::Opened(2),
        "now a single-participant barrier"
    );
    assert_eq!(b.current_generation(), 2);
    assert!(b.is_open(1), "generation 1 is open");
    assert!(!b.is_open(2), "generation 2 has not opened yet");
}

/// `peers_waiting`, the live-loop park's step-start wake predicate
/// (`remaining < expected`). Oracle-vector over the full state walk: idle
/// boundaries (fresh, post-open re-arm) read `false` (no false wakes); ANY
/// in-flight arrival reads `true`; the `expected == 0` degenerate is vacuously
/// `false` before AND after a vacuous arrive (the vacuous CAS opener never
/// touches `remaining`); `set_expected` at a boundary re-establishes equality;
/// and a `drop_participant` racing a stale `remaining` cannot wedge the
/// predicate — the open's re-arm reads the DECREMENTED `expected`, restoring
/// `remaining == expected` at the next boundary.
#[test]
fn peers_waiting_wake_predicate_oracle() {
    // Fresh barrier(3): armed boundary, nobody waiting.
    let b = BarrierShared::new(3);
    assert!(!b.peers_waiting(), "fresh barrier: remaining == expected");

    // ONE arrival: someone is now waiting on the rest of the cohort.
    assert_eq!(b.arrive(0), ArriveOutcome::Pending);
    assert!(b.peers_waiting(), "one arrival in flight ⇒ true");

    // TWO arrivals: still waiting.
    assert_eq!(b.arrive(0), ArriveOutcome::Pending);
    assert!(b.peers_waiting(), "two arrivals in flight ⇒ still true");

    // ALL arrived: the unique opener re-arms remaining = expected ⇒ false
    // again (the idle inter-step boundary — the no-false-wakes half).
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
    assert!(!b.peers_waiting(), "generation opened + re-armed ⇒ false");

    // expected == 0 degenerate: vacuously false, before and after the vacuous
    // CAS opener (which advances the generation without touching remaining).
    let z = BarrierShared::new(0);
    assert!(!z.peers_waiting(), "expected == 0: vacuously false");
    assert_eq!(z.arrive(0), ArriveOutcome::Opened(1));
    assert!(!z.peers_waiting(), "vacuous arrive leaves 0 == 0 ⇒ false");

    // set_expected at a boundary re-establishes remaining == expected.
    z.set_expected(2);
    assert!(
        !z.peers_waiting(),
        "boundary set_expected(2) ⇒ 2 == 2 ⇒ false"
    );
    assert_eq!(z.arrive(1), ArriveOutcome::Pending);
    assert!(z.peers_waiting(), "1 of 2 arrived ⇒ true");
    assert_eq!(z.arrive(1), ArriveOutcome::Opened(2));
    assert!(!z.peers_waiting(), "re-armed at 2 ⇒ false");

    // drop_participant interplay: barrier(3), one live arrival, then a dead
    // peer is dropped (expected 3→2, remaining 2→1). The predicate stays true
    // (one live participant IS still waiting on the last one) — and the final
    // arrival's open re-arms remaining from the DECREMENTED expected (2), so
    // the next boundary reads false: a stale remaining can never wedge it.
    let d = BarrierShared::new(3);
    assert_eq!(d.arrive(0), ArriveOutcome::Pending);
    assert!(d.peers_waiting(), "1 of 3 arrived ⇒ true");
    assert_eq!(d.drop_participant(0), ArriveOutcome::Pending);
    assert!(
        d.peers_waiting(),
        "post-drop (remaining 1, expected 2): the live arriver still waits"
    );
    assert_eq!(d.arrive(0), ArriveOutcome::Opened(1));
    assert!(
        !d.peers_waiting(),
        "open re-arms from the decremented expected (2 == 2) ⇒ false"
    );
    assert_eq!(d.expected(), 2, "survivor-sized cohort");
}

// ---- MappedBarrier SHM mapping (behavioral) ----
//
// Raw POSIX SHM on every Unix (macOS real-maps too; the by-name
// registry stub survives only for non-Unix future targets). No
// iceoryx2, no `#[serial]` — names are pid-scoped + per-test-tag so concurrent
// binaries and re-runs never collide. Cleanup is automatic: `create_owned` owns
// + unlinks on Drop and `open_unowned` is strict (never creates), so no test
// leaks a `/dev/shm` object (the one deliberate `mem::forget` leak is documented
// at its call site).

/// pid-scoped namespace so concurrent test binaries / re-runs never collide on a
/// POSIX SHM object (or, on the non-Unix stub, a registry key).
fn test_ns(tag: &str) -> String {
    format!("barrier_{}_{tag}", std::process::id())
}

/// Rendezvous through the shared segment (ALL-OS): an owner and a peer mapping of
/// the same `(ns, id)` operate on the SAME `BarrierShared` (a `MAP_SHARED` page on
/// every Unix; the by-name registry's `Arc` on the non-Unix stub). The
/// opener's generation bump is visible to the other handle through `Deref`.
#[test]
fn mapped_rendezvous_shared_state() {
    let ns = test_ns("rendezvous");
    let owner = MappedBarrier::create_owned(&ns, "g", 2).unwrap();
    let peer = MappedBarrier::open_unowned(&ns, "g").unwrap();

    assert_eq!(owner.arrive(0), ArriveOutcome::Pending, "owner: 1 of 2");
    assert_eq!(
        peer.arrive(0),
        ArriveOutcome::Opened(1),
        "peer: 2 of 2 opens (shared page/Arc)"
    );
    assert!(
        owner.is_open(0),
        "the opener's bump is visible across the shared segment to the owner"
    );

    drop(peer);
    drop(owner);
}

/// Unix (un-gated from Linux-only — macOS now real-maps too): two
/// mappings of the SAME object have DISTINCT virtual addresses yet share the
/// SAME physical page (proven by cross-mapping state visibility). The
/// non-Unix stub gives both handles the same `Arc`, so distinct-address can
/// only be asserted on real SHM. On macOS this test is ALSO the stub-removal
/// tell: the registry stub returned `Arc::as_ptr`-equal addresses, so a
/// regression back to the stub fails the `assert_ne!` immediately.
#[cfg(unix)]
#[test]
fn mapped_distinct_virtual_addrs_same_physical_page() {
    let ns = test_ns("distinct_addr");
    let owner = MappedBarrier::create_owned(&ns, "g", 2).unwrap();
    let peer = MappedBarrier::open_unowned(&ns, "g").unwrap();

    assert_ne!(
        owner.addr(),
        peer.addr(),
        "two mappings of the same object → distinct virtual addresses"
    );
    // …yet the same physical page: state writes through both.
    assert_eq!(owner.arrive(0), ArriveOutcome::Pending);
    assert_eq!(peer.arrive(0), ArriveOutcome::Opened(1));
    assert!(owner.is_open(0));

    drop(peer);
    drop(owner);
}

/// ALL-OS: a STRICT `open_unowned` of a never-created segment errors — the
/// owner must `create_owned` first; no silent create.
#[test]
fn open_unowned_missing_segment_errs() {
    assert!(
        MappedBarrier::open_unowned(&test_ns("missing"), "never-created").is_err(),
        "strict open of a never-created segment must error (no silent create)"
    );
}

/// Owner drop unlinks the name while a still-held peer mapping stays valid
/// (unlink ≠ unmap), ALL-OS.
///
/// The owner arrives first (count-down 2→1) then drops; the name is gone (Linux
/// `shm_unlink` / non-Unix-stub registry-removal) so a fresh `open_unowned` fails. The peer
/// mapping/Arc survives: its `current_generation()` reads the intact state (0),
/// and `peer.arrive(0)` counts the SAME shared `remaining` 1→0 → `Opened(1)`
/// (the owner's pre-drop decrement is still observed through the peer mapping —
/// `drop` munmaps/unlinks but never calls `drop_participant`, so the count is
/// untouched).
#[test]
fn owner_drop_unlinks_consumer_stays_valid() {
    let ns = test_ns("owner_drop");
    let owner = MappedBarrier::create_owned(&ns, "g", 2).unwrap();
    let peer = MappedBarrier::open_unowned(&ns, "g").unwrap();

    assert_eq!(owner.arrive(0), ArriveOutcome::Pending, "owner: 1 of 2");

    drop(owner);

    // The name is gone (Unix: shm_unlink; non-Unix stub: registry entry removed).
    assert!(
        MappedBarrier::open_unowned(&ns, "g").is_err(),
        "owner drop unlinked the name — a fresh open_unowned now fails"
    );

    // The still-held peer mapping/Arc survives the unlink (unlink ≠ unmap): it
    // reads the intact state and remains fully usable.
    assert_eq!(
        peer.current_generation(),
        0,
        "consumer mapping is alive and the state is intact after the unlink"
    );
    assert_eq!(
        peer.arrive(0),
        ArriveOutcome::Opened(1),
        "peer counts the shared remaining 1→0 (owner's pre-drop arrive is still observed)"
    );

    drop(peer);
}

/// Negative control (ALL-OS): same `id`, DIFFERENT `ns` → DIFFERENT segments,
/// no cross-tenant rendezvous. `a` and `b` are independent working barriers.
#[test]
fn f2_different_ns_no_rendezvous() {
    let ns_a = test_ns("f2a");
    let ns_b = test_ns("f2b");
    let a = MappedBarrier::create_owned(&ns_a, "g", 2).unwrap();
    let b = MappedBarrier::create_owned(&ns_b, "g", 2).unwrap();

    assert_eq!(a.arrive(0), ArriveOutcome::Pending, "a: 1 of 2");
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "b: 1 of 2");
    assert!(
        !a.is_open(0),
        "different ns must be different segments — no cross-tenant rendezvous"
    );

    // `a` is a real working barrier: its 2nd participant opens it, while `b`
    // (still 1 of 2) is untouched.
    assert_eq!(a.arrive(0), ArriveOutcome::Opened(1), "a: 2 of 2 opens");
    assert!(a.is_open(0));
    assert!(!b.is_open(0), "b still has only 1 of 2 — untouched by a");

    drop(a);
    drop(b);
}

/// ALL-OS: reusing an orphaned name resets to a FRESH generation 0. A
/// single-participant owner advances the generation to 3, then is `mem::forget`-ed
/// to simulate a CRASH (no Drop → no unlink → orphan at gen 3). A new
/// `create_owned` of the same `(ns, id)` must observe generation 0, proving the
/// orphan was cleared (Unix: unlink-then-`O_EXCL`; non-Unix stub: replace-in-registry).
#[test]
fn f5_orphan_reuse_resets_generation() {
    let ns = test_ns("f5_orphan");
    let owner = MappedBarrier::create_owned(&ns, "g", 1).unwrap();
    // Single participant → each arrive is the last arriver, advancing generation.
    assert_eq!(owner.arrive(0), ArriveOutcome::Opened(1));
    assert_eq!(owner.arrive(1), ArriveOutcome::Opened(2));
    assert_eq!(owner.arrive(2), ArriveOutcome::Opened(3));
    assert_eq!(owner.current_generation(), 3);

    // Simulate a CRASH: no Drop runs, so the segment is orphaned at generation 3
    // (Unix: the name stays linked to a gen-3 inode; non-Unix stub: the registry keeps the
    // gen-3 Arc). NOTE: `mem::forget` intentionally leaks one bounded (64-byte)
    // segment for the rest of the process (reclaimed at process exit) — exactly
    // what a crash-without-unlink would do.
    std::mem::forget(owner);

    let fresh = MappedBarrier::create_owned(&ns, "g", 1).unwrap();
    assert_eq!(
        fresh.current_generation(),
        0,
        "orphan reuse (unlink-then-O_EXCL on Unix / replace-in-registry on the non-Unix stub) must reset to a FRESH generation 0"
    );
    drop(fresh);
}

// ---- Parking `wait` (monitor-wait park) ----
//
// The parking `wait` replaces the deleted placeholder busy-spin. On a
// WAITPKG/WFE machine it exercises the REAL shallow CPU park-wake; on
// macOS / non-WAITPKG x86 the `Unavailable` backend no-ops so `wait` falls back to
// a sleep-recheck (NEVER a busy-spin). Both paths must return the same outcome,
// so these tests assert OUTCOME (Opened / TimedOut), which is invariant across
// the backend.

/// An already-open generation returns `Opened` immediately (no park).
#[test]
fn wait_returns_opened_immediately_when_already_open() {
    let b = BarrierShared::new(1);
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1)); // already open
    assert_eq!(
        b.wait(0, Duration::from_secs(30)),
        WaitOutcome::Opened,
        "an already-open generation returns Opened immediately"
    );
}

/// A generation that never opens returns `TimedOut` near the timeout, not a hang.
/// This is the survivable exit: the caller must NOT proceed on `TimedOut`,
/// but it must be BOUNDED (a crashed peer can never open the generation, and the
/// ~100µs monitor recheck timer cannot rescue it — only a finite timeout can).
#[test]
fn wait_times_out_when_generation_never_opens() {
    let b = BarrierShared::new(2);
    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Pending,
        "1 of 2 — never completes"
    );

    let t0 = Instant::now();
    assert_eq!(
        b.wait(0, Duration::from_millis(100)),
        WaitOutcome::TimedOut,
        "an un-opened generation must return TimedOut (the survivable exit)"
    );
    assert!(
        t0.elapsed() < Duration::from_secs(5),
        "must return near the 100ms timeout, not hang (returned in {:?})",
        t0.elapsed()
    );
}

/// Headline park-and-wake: a parked waiter wakes with `Opened` when the opener
/// stores the next generation. The opener is DETERMINISTIC — a `std::sync::Barrier`
/// arrival-gate guarantees the waiter has arrived (and is heading into `wait`)
/// before main becomes the 2nd arriver, so there is no flake.
///
/// On a WAITPKG/WFE machine this exercises the REAL CPU park-wake; on macOS /
/// non-WAITPKG it exercises the sleep-recheck fallback. Both must return `Opened`.
#[test]
fn wait_wakes_a_parked_waiter() {
    let b = Arc::new(BarrierShared::new(2));
    let arrived = Arc::new(std::sync::Barrier::new(2));
    let waiter = {
        let b = Arc::clone(&b);
        let arrived = Arc::clone(&arrived);
        thread::spawn(move || {
            assert_eq!(b.arrive(0), ArriveOutcome::Pending, "waiter arrives 1 of 2");
            arrived.wait(); // signal main: waiter has arrived
            b.wait(0, Duration::from_secs(30)) // park until the opener stores
        })
    };
    arrived.wait(); // main waits until the waiter has arrived
    thread::sleep(Duration::from_millis(20)); // bias toward exercising the PARK path (correctness holds without it)
    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Opened(1),
        "main is deterministically the 2nd arriver → opens"
    );
    assert_eq!(
        waiter.join().expect("waiter panicked"),
        WaitOutcome::Opened,
        "the parked waiter must wake with Opened"
    );
}

/// `wait` reaches THROUGH `MappedBarrier`'s `Deref` and wakes across two SEPARATE
/// handles of the same `(ns, id)` (Unix: same physical page; non-Unix stub: same registry
/// Arc). The OWNER handle is moved into the waiter thread (it is `Send`); the peer
/// opens on main. DETERMINISTIC opener via a `std::sync::Barrier` arrival-gate.
#[test]
fn wait_wakes_across_mapped_handles() {
    let ns = test_ns("wait_mapped");
    let owner = MappedBarrier::create_owned(&ns, "g", 2).unwrap();
    let peer = MappedBarrier::open_unowned(&ns, "g").unwrap();
    let arrived = Arc::new(std::sync::Barrier::new(2));
    let waiter = {
        let arrived = Arc::clone(&arrived);
        thread::spawn(move || {
            assert_eq!(owner.arrive(0), ArriveOutcome::Pending);
            arrived.wait();
            owner.wait(0, Duration::from_secs(30)) // MappedBarrier::wait via Deref
        })
    };
    arrived.wait();
    thread::sleep(Duration::from_millis(20));
    assert_eq!(peer.arrive(0), ArriveOutcome::Opened(1));
    assert_eq!(
        waiter.join().expect("waiter panicked"),
        WaitOutcome::Opened,
        "a wait() on one mapped handle wakes when another handle opens the shared segment"
    );
    drop(peer);
}

// ---- Kernel futex wake for the cross-process barrier ----
//
// The wake mechanism is a WHEN optimization, never a WHAT change (the firewall
// contract) — the entire suite above must stay green UNCHANGED. These three
// tests pin the LIVENESS of the wake at each generation-bump site: a parked
// waiter must return `Opened` within 5ms of the triggering call. The bound is
// deliberately generous (it pins liveness, not µs — the real µs-class proof is
// the hardware bench), but it is MEANINGFUL on Linux: the futex tier's re-poll slice
// is 10ms, so a bump site that FORGETS its `FUTEX_WAKE` leaves the waiter
// asleep until the slice expires (~10ms) and FAILS the 5ms bound. (On the
// non-Linux fallback the ~100µs recheck masks a missing wake — these pins are
// Linux-effective, which is where the futex tier lives.)

/// One attempt of the parked-waiter wake-latency scenario: the waiter
/// (optionally) arrives, signals READY via a start-gate, then parks in
/// `wait(0, 2s)`; the caller sleeps 50ms past the gate (the waiter is provably
/// inside `wait`), runs `trigger`, asserts Opened, and returns the
/// trigger→wake latency for the caller's bound check.
fn bump_site_wake_latency(
    b: &Arc<BarrierShared>,
    waiter_arrives_first: bool,
    trigger: &impl Fn(&BarrierShared),
) -> Duration {
    let gate = Arc::new(std::sync::Barrier::new(2));
    let waiter = {
        let b = Arc::clone(b);
        let gate = Arc::clone(&gate);
        thread::spawn(move || {
            if waiter_arrives_first {
                assert_eq!(b.arrive(0), ArriveOutcome::Pending, "waiter arrives 1 of N");
            }
            gate.wait();
            let outcome = b.wait(0, Duration::from_secs(2));
            (outcome, Instant::now())
        })
    };
    gate.wait();
    thread::sleep(Duration::from_millis(50)); // waiter is parked inside wait()
    let t0 = Instant::now();
    trigger(b);
    let (outcome, woke_at) = waiter.join().expect("waiter panicked");
    assert_eq!(
        outcome,
        WaitOutcome::Opened,
        "the bump must open the waiter"
    );
    woke_at.duration_since(t0)
}

/// A parked waiter must wake within 5ms of the generation bump.
///
/// Retry-ONCE on a fresh barrier rather than widening the
/// bound. The 5ms pin stays TIGHT (a bump site that forgets its `FUTEX_WAKE`
/// leaves the waiter asleep for the full 10ms futex slice, which fails BOTH
/// attempts deterministically — the retry cannot mask the regression this pin
/// exists for), while a single-shot wall-clock stall on a loaded CI runner (a
/// preempted waiter thread) is absorbed by the second attempt. Each attempt
/// builds a FRESH barrier via `make_barrier` so the trigger's arrival
/// arithmetic sees pristine state.
fn assert_bump_site_wakes_parked_waiter(
    make_barrier: impl Fn() -> Arc<BarrierShared>,
    waiter_arrives_first: bool,
    trigger: impl Fn(&BarrierShared),
) {
    const WAKE_BOUND: Duration = Duration::from_millis(5);
    const ATTEMPTS: usize = 2;
    let mut last_latency = Duration::ZERO;
    for _ in 0..ATTEMPTS {
        let b = make_barrier();
        last_latency = bump_site_wake_latency(&b, waiter_arrives_first, &trigger);
        if last_latency < WAKE_BOUND {
            return;
        }
    }
    panic!(
        "parked waiter took {last_latency:?} to wake after the generation bump on \
         BOTH of {ATTEMPTS} attempts — the bump site did not kernel-wake it (the \
         Linux futex slice is 10ms, so a missing FUTEX_WAKE fails this {WAKE_BOUND:?} \
         liveness bound deterministically; one slow attempt is CI noise, two is real)"
    );
}

/// Wake pin (a) — the NORMAL opener (`arrive` -> `open_generation`)
/// wakes a parked waiter promptly.
#[test]
fn futex_wake_wakes_parked_waiter_fast() {
    assert_bump_site_wakes_parked_waiter(
        || Arc::new(BarrierShared::new(2)),
        true,
        |b| {
            assert_eq!(
                b.arrive(0),
                ArriveOutcome::Opened(1),
                "main is the 2nd arriver -> the normal opener"
            );
        },
    );
}

/// Wake pin (b) — the VACUOUS `expected == 0` CAS opener wakes a parked
/// waiter promptly (the second generation-bump site).
#[test]
fn vacuous_cas_opener_wakes_parked_waiter() {
    assert_bump_site_wakes_parked_waiter(
        || Arc::new(BarrierShared::new(0)),
        false,
        |b| {
            assert_eq!(
                b.arrive(0),
                ArriveOutcome::Opened(1),
                "exp==0: the CAS opener publishes the generation"
            );
        },
    );
}

/// Wake pin (c) — `drop_dead_peer` as the opener (via
/// `try_drop_participant` -> `open_generation`) wakes a parked waiter promptly
/// (the dead-peer-drop bump site).
#[test]
fn drop_dead_peer_opener_wakes_parked_waiter() {
    assert_bump_site_wakes_parked_waiter(
        || Arc::new(BarrierShared::new(2)),
        true,
        |b| {
            assert_eq!(
                b.drop_dead_peer(Duration::ZERO),
                DeadPeerDrop::Applied { opened: true },
                "dropping the missing 2nd slot completes the generation"
            );
        },
    );
}

/// Post-timeout usability: a `wait` that returns `TimedOut` leaves the
/// barrier state INTACT — a later arrival still opens it and a fresh `wait`
/// returns `Opened`. This proves `TimedOut` is a non-destructive "caller must
/// retry / treat as fatal" signal (it never advanced the generation, never
/// touched the count-down), not a corruption. Without this, a `wait` that
/// silently consumed/wedged state on timeout would pass every other test.
#[test]
fn wait_timeout_then_delayed_arrival_opens() {
    let b = Arc::new(BarrierShared::new(2));
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "1 of 2 — incomplete");

    // First wait times out — the 2nd participant has not arrived.
    assert_eq!(
        b.wait(0, Duration::from_millis(50)),
        WaitOutcome::TimedOut,
        "no 2nd arrival within 50ms → TimedOut"
    );
    // The timeout must NOT have advanced or corrupted the barrier.
    assert_eq!(
        b.current_generation(),
        0,
        "TimedOut did not advance the generation"
    );
    assert!(
        !b.is_open(0),
        "generation 0 is still closed after the timeout"
    );

    // A delayed 2nd arrival opens generation 0; a fresh wait must now see Opened.
    let b2 = Arc::clone(&b);
    let opener = thread::spawn(move || {
        thread::sleep(Duration::from_millis(20));
        b2.arrive(0)
    });
    assert_eq!(
        b.wait(0, Duration::from_secs(5)),
        WaitOutcome::Opened,
        "a delayed arrival after a prior TimedOut still opens (state intact)"
    );
    assert_eq!(
        opener.join().expect("opener panicked"),
        ArriveOutcome::Opened(1),
        "the delayed 2nd arrival is the unique opener"
    );
}

// ---- Wide×deep stress + Tier-3 multi-process harness ----

/// Wide (many participants) × deep (many generations) rendezvous over ONE shared
/// MappedBarrier segment — the scale stress (generations ≈ DAG levels,
/// participants ≈ process groups). Each thread holds its OWN handle (own mmap on
/// Unix / own registry Arc-clone on the non-Unix stub) to the SAME (ns,id). Oracle (NOT a
/// self-compare): every generation 1..=GENS opens EXACTLY once across all threads
/// (sorted collected Opened == [1..=GENS]) and the final generation == GENS — a
/// lost/duplicated open at any level would break it. Uses the bounded yield-spin
/// `wait_open_capped` (not the parking wait()) so it is fast on every platform.
#[test]
fn wide_cohort_many_generation_mapped_stress() {
    const PARTICIPANTS: u32 = 16;
    const GENS: u64 = 1_000;
    let ns = test_ns("wide_stress");
    let owner = MappedBarrier::create_owned(&ns, "g", PARTICIPANTS).unwrap(); // creates+sizes; does NOT arrive
    let mut handles = Vec::new();
    for _ in 0..PARTICIPANTS {
        let ns = ns.clone();
        handles.push(std::thread::spawn(move || {
            let bar = MappedBarrier::open_unowned(&ns, "g").expect("peer open");
            let mut opened = Vec::new();
            for g in 0..GENS {
                if let ArriveOutcome::Opened(ng) = bar.arrive(g) {
                    opened.push(ng);
                }
                wait_open_capped(&bar, g);
            }
            opened
        }));
    }
    let mut all_opened: Vec<u64> = handles
        .into_iter()
        .flat_map(|h| h.join().expect("thread panicked"))
        .collect();
    all_opened.sort_unstable();
    let expected: Vec<u64> = (1..=GENS).collect();
    assert_eq!(
        all_opened, expected,
        "every generation 1..=GENS opens exactly once across {PARTICIPANTS} participants (hand oracle)"
    );
    assert_eq!(owner.current_generation(), GENS, "final generation == GENS");
    drop(owner);
}

// ===========================================================================
// try_drop_participant (Stale vs Applied) + drop_dead_peer
// (owner-side dead-peer drop). Hermetic, oracle-vector, parallel-safe.
// ===========================================================================

/// A short stall-grace for the deterministic `drop_dead_peer` tests where the dead
/// peer never pre-arrived (gen is stable, so the disambiguation phase waits out the
/// full grace before dropping at g0). Kept small so the tests stay fast; the
/// OUTCOME is grace-independent.
const SHORT_GRACE: Duration = Duration::from_millis(20);

/// T-drop-1 — `try_drop_participant` reports `Stale` (a FULL no-op) for an
/// already-open generation and leaves BOTH counters untouched. `new(1)`;
/// `arrive(0)` opens gen 1; a `try_drop_participant(0)` targeting the now-open gen 0
/// must return `Stale` with `expected()` unchanged (the distinction
/// `drop_participant` folds into an ambiguous `Pending`).
#[test]
fn try_drop_participant_stale_leaves_counters_untouched() {
    let b = BarrierShared::new(1);
    assert_eq!(b.arrive(0), ArriveOutcome::Opened(1));
    assert_eq!(b.current_generation(), 1);

    assert_eq!(
        b.try_drop_participant(0),
        DropAttempt::Stale,
        "a drop for an already-open generation is Stale (a full no-op)"
    );
    assert_eq!(
        b.expected(),
        1,
        "Stale must NOT decrement expected (the full-no-op contract)"
    );
    assert_eq!(b.current_generation(), 1, "Stale must not advance the gen");
    // `drop_participant` folds Stale back into the ambiguous Pending (back-compat).
    assert_eq!(
        b.drop_participant(0),
        ArriveOutcome::Pending,
        "drop_participant collapses Stale to Pending for back-compat"
    );
    assert_eq!(b.expected(), 1, "still untouched via the wrapper");
}

/// T-drop-2 — `try_drop_participant` reports `Applied` (decrementing `expected`) for
/// the current generation, in BOTH the opener and the non-opener sub-cases. Opener:
/// `new(2)` with one live arrival (Pending), then the drop hits 0, yielding
/// `Applied(Opened(1))` and `expected == 1`. Non-opener: `new(3)` with one live
/// arrival, then the drop does NOT hit 0, yielding `Applied(Pending)` and
/// `expected == 2`.
#[test]
fn try_drop_participant_applied_decrements_expected() {
    // Opener sub-case.
    let b = BarrierShared::new(2);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "1 of 2 live");
    assert_eq!(
        b.try_drop_participant(0),
        DropAttempt::Applied(ArriveOutcome::Opened(1)),
        "the drop's fetch_sub hits 0 → Applied(Opened)"
    );
    assert_eq!(b.expected(), 1, "Applied decremented expected 2→1");
    assert_eq!(b.current_generation(), 1);

    // Non-opener sub-case.
    let b = BarrierShared::new(3);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "1 of 3 live");
    assert_eq!(
        b.try_drop_participant(0),
        DropAttempt::Applied(ArriveOutcome::Pending),
        "the drop does NOT hit 0 (one live still out) → Applied(Pending)"
    );
    assert_eq!(b.expected(), 2, "Applied decremented expected 3→2");
    assert_eq!(
        b.current_generation(),
        0,
        "gen not advanced (still incomplete)"
    );
}

/// T-drop-3 — `drop_dead_peer` with all survivors already arrived and ONLY the dead
/// peer missing OPENS the generation. `new(3)`, two arrivals (`remaining == 1`), no
/// pre-arrival from the dead 3rd → the disambiguation phase waits out `SHORT_GRACE`
/// (gen stable), then drops at g0: `Applied { opened: true }`, gen opens to 1,
/// `expected == 2`.
#[test]
fn drop_dead_peer_opens_when_only_dead_peer_missing() {
    let b = BarrierShared::new(3);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "survivor 1 of 3");
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "survivor 2 of 3");

    assert_eq!(
        b.drop_dead_peer(SHORT_GRACE),
        DeadPeerDrop::Applied { opened: true },
        "dropping the missing 3rd completes gen 0 for the 2 survivors"
    );
    assert_eq!(b.current_generation(), 1, "generation opened");
    assert_eq!(b.expected(), 2, "expected resized to the 2 survivors");
}

/// T-drop-4 — the dead-peer-PRE-ARRIVED case: the disambiguation phase OBSERVES the
/// generation advance past `g0` (the pre-arrival consumed), then applies the drop at
/// the STALLED next generation where the peer is now genuinely missing.
///
/// # Deterministic orchestration — NO sleep-as-synchronization
///
/// The dead peer pre-arrives at gen 0 (`Pending`, `remaining == 1`). A drop thread
/// runs `drop_dead_peer`'s body via the test seam
/// [`drop_dead_peer_with_post_snapshot_hook`](BarrierShared::drop_dead_peer_with_post_snapshot_hook),
/// whose hook fires the instant the method snapshots `g0`. The two orderings this
/// test must pin — neither left to a fragile bias sleep — are REAL
/// happens-befores over shared observable state:
///
/// 1. **`g0 == 0` → the "observed gen advance" branch.** Main spins until the hook
///    reports the snapshot was taken, and only THEN completes gen 0. Because the
///    snapshot is observed while `generation` is still 0, `g0` is provably 0, so the
///    disambiguation loop is guaranteed to break on `current_generation() > g0` (NOT
///    the stall-grace timeout) — the exact branch this test exists to cover.
/// 2. **survivor-before-drop → the opener is the DROP.** The hook then PARKS the
///    dropper (still inside `drop_dead_peer`, right after the snapshot) until main has
///    completed gen 0 (`Opened(1)`) AND its gen-1 arrival (`Pending`, `remaining
///    2→1`). Only then is the dropper released; its `try_drop_participant(1)` finds
///    `remaining == 1`, so ITS `fetch_sub` hits 0 → the drop is the unique opener
///    (`Applied { opened: true }`).
///
/// # Hand-computed oracle (not a two-run self-compare)
///
/// `arrive(0) == Opened(1)`, `arrive(1) == Pending`, the drop `== Applied { opened:
/// true }`, final `generation == 2`, `expected == 1`.
///
/// # Why a bias sleep would be the bug
///
/// A 20 ms bias sleep ("dropper snapshots g0=0 first") is not a synchronization. On a
/// stalled Ubuntu CI runner the
/// dropper is descheduled PAST the bias, snapshots a later generation, and the
/// `arrive(1) == Pending` assertion instead observes the equally-legal early-apply
/// `Opened(2)` — the SURVIVOR, not the drop, winning the shared `remaining`
/// count-down at gen 1 (the barrier's unique-opener contract: exactly one of the
/// survivor's `arrive(1)` and the dead peer's drop observes `prev == 1`; WHICH one is
/// a legitimate race). Both outcomes are valid product behavior; the defect would be
/// pinning ONE of them with a sleep. The hook seam makes the intended ordering
/// deterministic. Per-test `AtomicBool` flags (no process-global state) keep the test
/// parallel-safe alongside its siblings.
#[test]
fn drop_dead_peer_observes_gen_advance_then_applies_at_next_gen() {
    use std::sync::atomic::{AtomicBool, Ordering};

    let b = Arc::new(BarrierShared::new(2));
    // Dead peer pre-arrives at gen 0 (it will never arrive at gen 1).
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "dead peer pre-arrived");

    // Per-test happens-before flags (no process-global state → parallel-safe):
    //   `snapshotted` — set by the hook the instant the dropper captures g0.
    //   `release`     — set by main to unblock the dropper once gens 0 + 1 are done.
    let snapshotted = Arc::new(AtomicBool::new(false));
    let release = Arc::new(AtomicBool::new(false));

    let dropper = {
        let b = Arc::clone(&b);
        let snapshotted = Arc::clone(&snapshotted);
        let release = Arc::clone(&release);
        thread::spawn(move || {
            // Fires right after `drop_dead_peer` snapshots g0. It (a) publishes "g0
            // captured" (forcing g0 == 0 for main, since main has not advanced the
            // generation yet) and (b) parks the dropper until main has set up the
            // gen-1 stall — so the survivor deterministically counts `remaining` down
            // to 1 BEFORE the drop's `fetch_sub`, making the drop the unique opener.
            // The `SHORT_GRACE` is irrelevant to the OUTCOME: by the time the hook
            // returns, `generation` is already 1 > g0, so the disambiguation loop
            // breaks on its FIRST `current_generation() > g0` check — the grace
            // deadline (started only after the hook returns) is never reached.
            let hook = || {
                snapshotted.store(true, Ordering::Release);
                while !release.load(Ordering::Acquire) {
                    std::hint::spin_loop();
                }
            };
            b.drop_dead_peer_with_post_snapshot_hook(SHORT_GRACE, &hook)
        })
    };

    // (1) Wait for the dropper to snapshot g0 (== 0, since the generation is still 0).
    // This is a real happens-before, where a bias sleep would only guess.
    while !snapshotted.load(Ordering::Acquire) {
        std::hint::spin_loop();
    }

    // (2) With the dropper parked in the hook, drive gen 0 → gen 1 deterministically.
    assert_eq!(
        b.arrive(0),
        ArriveOutcome::Opened(1),
        "survivor completes gen 0 (dropper is parked in the post-snapshot hook)"
    );
    assert_eq!(
        b.arrive(1),
        ArriveOutcome::Pending,
        "survivor waits at gen 1 (remaining 2→1); the drop has not applied yet"
    );

    // (3) Release the dropper: it observes generation 1 > g0 0, breaks disambiguation
    // on the "gen advanced" branch, and applies at gen 1 where remaining == 1 → its
    // fetch_sub hits 0 → the drop is the unique opener.
    release.store(true, Ordering::Release);

    assert_eq!(
        dropper.join().expect("dropper panicked"),
        DeadPeerDrop::Applied { opened: true },
        "the drop lands at gen 1 (the stalled boundary), releasing the survivor"
    );
    assert_eq!(b.current_generation(), 2, "gen 1 opened by the drop");
    assert_eq!(b.expected(), 1, "expected resized to the 1 survivor");
}

// T-drop-5 (NOTE, no test) — the apply-loop `Stale`->retry branch has NO standalone
// deterministic test, by design. It IS reachable, but only through ONE narrow live
// interleave: the residual-ambiguity case where the dead peer PRE-ARRIVED at the
// stall-gated generation g0 and a slow survivor's late arrival OPENS g0 in the
// window between the apply loop's `current_generation()` read and its
// `try_drop_participant` call — the drop then observes `Stale` and retries at the
// (provably-safe) next generation. In every other scenario the boundary cannot open
// without the drop itself (the dead peer holds the only missing slot), so the loop
// applies on the first iteration. Orchestrating that pre-arrival race
// deterministically requires a hostile fixed interleave that violates the barrier's
// one-generation max-skew invariant on the way in (tripping its `debug_assert`), so
// no valid standalone fixture exists. The `Stale`->re-read->`Applied` building block
// the apply loop composes is pinned deterministically by
// `try_drop_participant_stale_leaves_counters_untouched` +
// `try_drop_participant_applied_decrements_expected`; the "gen advanced during
// disambiguation, apply at the NEW gen" path is pinned by
// `drop_dead_peer_observes_gen_advance_then_applies_at_next_gen`.

/// T-drop-6 — `NoSlots` when `expected == 0`: a zero-participant cohort has no slot
/// to drop, so `drop_dead_peer` returns immediately WITHOUT touching state.
#[test]
fn drop_dead_peer_no_slots_on_empty_cohort() {
    let b = BarrierShared::new(0);
    assert_eq!(
        b.drop_dead_peer(SHORT_GRACE),
        DeadPeerDrop::NoSlots,
        "an empty cohort (expected == 0) has no slot to drop"
    );
    assert_eq!(b.expected(), 0, "state untouched");
    assert_eq!(b.current_generation(), 0, "gen untouched");
}

/// T-drop-7 — sequential TWO-peer drops ratchet `expected` N→N-2 with the correct
/// opens. `new(4)`, two survivors arrived (`remaining == 2`). Drop #1 does NOT hit 0
/// (`Applied{opened:false}`, `expected 4→3`); drop #2 hits 0
/// (`Applied{opened:true}`, `expected 3→2`, gen opens). The 2 survivors are then
/// released. Single-threaded (order-independent); each drop waits `SHORT_GRACE`
/// (gen stable) before applying at g0.
#[test]
fn drop_dead_peer_sequential_two_drops_ratchet_expected() {
    let b = BarrierShared::new(4);
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "survivor 1 of 4");
    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "survivor 2 of 4");

    assert_eq!(
        b.drop_dead_peer(SHORT_GRACE),
        DeadPeerDrop::Applied { opened: false },
        "drop #1: remaining 2→1, not yet open"
    );
    assert_eq!(b.expected(), 3, "expected 4→3 after the first drop");
    assert_eq!(b.current_generation(), 0, "gen still 0");

    assert_eq!(
        b.drop_dead_peer(SHORT_GRACE),
        DeadPeerDrop::Applied { opened: true },
        "drop #2: remaining 1→0 → opens gen 0 for the 2 survivors"
    );
    assert_eq!(b.expected(), 2, "expected 3→2 after the second drop");
    assert_eq!(b.current_generation(), 1, "generation opened");
}

/// T-drop-8 — SIGINT-cascade simulation: `k` participants each SELF-drop
/// (`drop_participant`, the worker path) at the SAME boundary generation, released
/// together by a `std::sync::Barrier` start-gate. Oracle: `expected` ratchets to 0,
/// the generation opens EXACTLY once (a single `Opened(1)` across all k drops), and
/// there is no underflow (exactly k drops on `remaining == k` hit `prev = k..=1`,
/// none hits `prev == 0`).
#[test]
fn self_drop_cascade_ratchets_to_zero_opens_once() {
    const K: u32 = 6;
    let b = Arc::new(BarrierShared::new(K));
    let start = Arc::new(std::sync::Barrier::new(K as usize));

    let mut handles = Vec::new();
    for _ in 0..K {
        let b = Arc::clone(&b);
        let start = Arc::clone(&start);
        handles.push(thread::spawn(move || {
            start.wait();
            b.drop_participant(0)
        }));
    }

    let mut opened: Vec<u64> = handles
        .into_iter()
        .map(|h| h.join().expect("self-dropper panicked"))
        .filter_map(|o| match o {
            ArriveOutcome::Opened(g) => Some(g),
            ArriveOutcome::Pending => None,
        })
        .collect();
    opened.sort_unstable();

    assert_eq!(
        opened,
        vec![1],
        "exactly one of the k self-drops opens the generation (no lost/duplicated open, no underflow)"
    );
    assert_eq!(
        b.expected(),
        0,
        "all k participants dropped → expected ratchets to 0"
    );
    assert_eq!(
        b.current_generation(),
        1,
        "the boundary opened exactly once"
    );
}

/// T-drop-9 — stress: 3 participants × many generations, where ONE drops at a fixed,
/// index-derived (NOT `rand`) generation and the other two continue in lockstep to a
/// HAND-computed final generation. Oracle: every generation `1..=GENS` opens EXACTLY
/// once (sorted collected `Opened` == `[1..=GENS]`), the final generation is `GENS`,
/// and `expected` ends at 2 (3 minus the one drop).
#[test]
fn one_participant_drops_midway_rest_continue_to_final_generation() {
    const N: u32 = 3;
    const GENS: u64 = 2_000;
    // Fixed, deterministic "random-ish" drop generation (a prime well inside the
    // range) — NOT `rand`. Participant index 1 is the dropper; 0 and 2 run to GENS.
    const DROP_GEN: u64 = 997;
    const DROPPER: u32 = 1;

    let b = Arc::new(BarrierShared::new(N));
    let mut handles = Vec::new();
    for idx in 0..N {
        let b = Arc::clone(&b);
        handles.push(thread::spawn(move || {
            let is_dropper = idx == DROPPER;
            let mut opened = Vec::new();
            let mut g = 0u64;
            while g < GENS {
                if is_dropper && g == DROP_GEN {
                    // Leave the cohort: count out of this boundary, then STOP.
                    if let ArriveOutcome::Opened(ng) = b.drop_participant(g) {
                        opened.push(ng);
                    }
                    break;
                }
                if let ArriveOutcome::Opened(ng) = b.arrive(g) {
                    opened.push(ng);
                }
                wait_open_capped(&b, g);
                g += 1;
            }
            opened
        }));
    }

    let mut all_opened: Vec<u64> = handles
        .into_iter()
        .flat_map(|h| h.join().expect("participant thread panicked"))
        .collect();
    all_opened.sort_unstable();

    let expected: Vec<u64> = (1..=GENS).collect();
    assert_eq!(
        all_opened, expected,
        "every generation 1..=GENS opens exactly once across the drop + continued lockstep"
    );
    assert_eq!(b.current_generation(), GENS, "final generation == GENS");
    assert_eq!(
        b.expected(),
        2,
        "one of the 3 participants dropped → expected ends at 2"
    );
}

/// T-drop-10 — TERMINAL drop: the dead peer was the LAST
/// participant. `new(1)`, NO arrivals; `drop_dead_peer` (short grace — the
/// generation is stable, so the disambiguation phase waits it out and drops at g0)
/// must: saturating-decrement `expected` 1→0, hit `prev == 1` on the `remaining`
/// count-down → the UNIQUE opener with ZERO survivors → `Applied { opened: true }`,
/// and open the generation to 1. Oracle-vector (absolute expected values, not a
/// self-compare).
#[test]
fn drop_dead_peer_terminal_last_participant_opens_with_zero_survivors() {
    let b = BarrierShared::new(1);
    assert_eq!(
        b.drop_dead_peer(SHORT_GRACE),
        DeadPeerDrop::Applied { opened: true },
        "dropping the sole (never-arrived) participant is the unique opener"
    );
    assert_eq!(
        b.expected(),
        0,
        "expected saturating-decremented 1→0 (zero survivors)"
    );
    assert_eq!(
        b.current_generation(),
        1,
        "the drop opened generation 0 → generation is 1"
    );
}

// ============ macOS os_sync barrier-wake tier (public API) ============
//
// The pure oracles (kill-switch parse / errno classification) and the
// private-method hermetic waits (`wait_os_sync` / `wait_recheck_fallback`) live
// in the INTERNAL `mod tests` in `barrier.rs` (codebase convention — mirrors
// `futex_errno_classification_oracle` / `barrier_spin_us_parse_oracle`, which
// need private access). These integration tests cover the PUBLIC surface: the
// diagnostic tier accessors, the kill-switch env wiring, and the wake-latency
// A/B. All os_sync-specific arms are macOS-gated (the primitive is macOS-only);
// on Linux only the always-on tier-name probe compiles.

/// Availability probe — PRINT-ONLY (never asserts a value): eprintln
/// the wake tier + os_sync backend presence so CI logs show whether the real
/// primitive ran on this host. Runs on every OS; on macOS ≥ 14.4 it prints
/// `macos-os_sync`, on Linux `linux-futex`, on an older macOS
/// `macos-recheck-fallback`. `#[serial]` (the os_sync trio):
/// `barrier_wake_tier()` reads the kill-switch/backend `OnceLock` caches, so
/// this read must never land inside the kill-switch test's `=0` env window
/// (the caches latch process-wide on first resolution).
#[test]
#[serial]
fn os_sync_availability_probe_prints_tier() {
    let tier = barrier_wake_tier();
    let backend = barrier_os_sync_backend_available();
    eprintln!("barrier wake tier on this host: {tier}");
    eprintln!("os_sync backend available: {backend}");
    // The only invariant we assert: a NON-empty tier name (the accessor is
    // always wired for some tier). The concrete value is host-dependent.
    assert!(!tier.is_empty(), "the wake-tier accessor must name a tier");
}

/// The no-inert-shipping class: the os_sync
/// backend must RESOLVE on a modern macOS. Without this, a future dlsym
/// symbol-string typo (or a libSystem export change) would make
/// `resolve_symbol` return `None`, every barrier wait would silently take the
/// sleep-recheck fallback forever, and CI would stay green while the headline
/// kernel wake is dead. Reads the HOST OS version (`sw_vers -productVersion`)
/// and at ≥ 14.4 HARD-ASSERTS backend PRESENCE via
/// `barrier_os_sync_backend_available()` — dlsym resolution only, deliberately
/// NOT the kill switch / env / disable latch, so no `CERULION_BARRIER_OS_SYNC`
/// setting or latched errno can mask a resolution failure. Older hosts skip
/// loudly. Turns silent-inert into a red CI on macos-latest.
#[cfg(target_os = "macos")]
#[test]
fn os_sync_backend_resolves_on_modern_macos() {
    let out = std::process::Command::new("sw_vers")
        .arg("-productVersion")
        .output()
        .expect("sw_vers -productVersion must run on macOS");
    let ver = String::from_utf8_lossy(&out.stdout).trim().to_string();
    let mut parts = ver.split('.');
    let major: u32 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let minor: u32 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    assert!(
        major > 0,
        "unparseable macOS version from sw_vers -productVersion: {ver:?}"
    );
    if (major, minor) < (14, 4) {
        eprintln!(
            "macOS {ver} < 14.4 — the os_sync_* symbols are not expected \
             on this host; skipping the backend-presence assert"
        );
        return;
    }
    assert!(
        barrier_os_sync_backend_available(),
        "macOS {ver} is >= 14.4 but the os_sync backend did NOT resolve — \
         dlsym(RTLD_DEFAULT) failed for `os_sync_wait_on_address_with_timeout` \
         and/or `os_sync_wake_by_address_all` (symbol-string typo? libSystem \
         export change?). Every barrier wait would silently run the sleep-recheck \
         fallback forever — the os_sync no-inert-shipping guard"
    );
}

/// macOS: the `CERULION_BARRIER_OS_SYNC=0` kill switch is HONORED
/// end-to-end (env var NAME + `=0` parse), and the route it selects — the
/// recheck fallback — still OPENS a 2-thread wait/arrive round. The env is read
/// FRESH via `os_sync_disabled_from_env_uncached` (NOT the process-cached
/// `os_sync_active` reader), and the fallback round runs through the
/// `wait_macos_for_test(use_os_sync = false)` seam — so the test's OWN
/// assertions never depend on the `OnceLock` caches.
///
/// Cache-poisoning defense (`#[serial]` IS needed here): the `=0` env window is
/// process-global, and the `os_sync_kill_switch()` `OnceLock` latches its
/// FIRST resolution forever — a parallel sibling whose first
/// `barrier_wake_tier()` / `os_sync_active()` read landed inside this window
/// would latch os_sync DISABLED process-wide, silently flipping every later
/// test (and the A/B's os_sync leg) to the fallback tier. Two layers:
/// 1. **LOAD-BEARING pre-resolution** — the first line below force-resolves
///    the kill-switch + backend caches via `barrier_wake_tier()` BEFORE the
///    guard sets the env. After that line the `OnceLock`s are FILLED with the
///    enabled state and nothing ever re-reads the env, so no sibling read —
///    whenever it lands — can observe the window. (Any sibling resolving
///    EARLIER than that line also reads an unset env: same result.)
/// 2. **`#[serial]` on the os_sync trio** (this test, the availability probe,
///    the wake-latency A/B) as defense-in-depth.
#[cfg(target_os = "macos")]
#[test]
#[serial]
fn os_sync_kill_switch_env_honored_and_fallback_opens() {
    // LOAD-BEARING (see the doc): pin the process-global kill-switch/backend
    // caches to the ENABLED state while the env is still unset — BEFORE the
    // guard opens the `=0` window. Removing this line re-opens the
    // latch-disabled-forever race for any cache-cold process.
    let _ = barrier_wake_tier();
    let _guard = EnvVarGuard::set("CERULION_BARRIER_OS_SYNC", "0");
    assert!(
        cerulion_core::barrier::os_sync_disabled_from_env_uncached(),
        "CERULION_BARRIER_OS_SYNC=0 must resolve to os_sync DISABLED (kill switch honored)"
    );

    // The route the kill switch selects (the recheck fallback) still opens.
    let b = Arc::new(BarrierShared::new(1));
    let opener = {
        let b = Arc::clone(&b);
        thread::spawn(move || {
            thread::sleep(Duration::from_millis(5));
            assert_eq!(
                b.arrive(0),
                ArriveOutcome::Opened(1),
                "the lone participant is the unique opener"
            );
        })
    };
    let outcome = b.wait_macos_for_test(0, Duration::from_secs(5), false);
    opener.join().expect("opener panicked");
    assert_eq!(
        outcome,
        WaitOutcome::Opened,
        "the kill-switch (fallback) route must still open on the opener's arrive"
    );
}

/// macOS: the FIRST evidence the os_sync primitive cuts the ~100µs
/// sleep-recheck quantization — a wake-latency A/B between the os_sync tier and
/// the recheck fallback IN ONE PROCESS via the `wait_macos_for_test` seam
/// (bypassing the cached routing). Each iteration: a waiter parks (after its
/// entry spin has expired — the waker sleeps 2ms, well past the ≤150µs boundary
/// spin), a waker records `t_wake` and `arrive`s (which `wake_waiters` the
/// parked waiter), the waiter records `t_return`; latency = `t_return −
/// t_wake`. PRINT-ONLY thresholds (numbers are hardware-dependent): the ONLY
/// assertion is that every round OPENED. Prints p50/p99 for both tiers so the
/// delta can be captured. `#[serial]` (the os_sync trio —
/// defense-in-depth): the seam's os_sync leg reads the backend
/// `OnceLock` (env-free) and its wake side (`arrive` → `wake_waiters`) DOES
/// consult the cached `os_sync_active()` — keeping the trio serialized keeps
/// every cache interaction outside the kill-switch test's `=0` env window.
#[cfg(target_os = "macos")]
#[test]
#[serial]
fn os_sync_vs_fallback_wake_latency_ab() {
    /// Rounds per tier (kept modest: each round is ~2ms + thread spawn).
    const ROUNDS: usize = 60;
    /// Waker delay — > the ≤150µs boundary spin so the waiter is genuinely
    /// PARKED (in the kernel / a sleep chunk) when woken, isolating the wake
    /// latency; < the 10ms os_sync slice so os_sync catches the wake in the
    /// FIRST block (not a re-spin).
    const WAKER_DELAY: Duration = Duration::from_millis(2);

    fn measure(use_os_sync: bool, rounds: usize, waker_delay: Duration) -> Vec<Duration> {
        let mut lat = Vec::with_capacity(rounds);
        for _ in 0..rounds {
            let b = Arc::new(BarrierShared::new(2));
            // Waiter: arrive (Pending), then park in the chosen tier; report the
            // instant it observed the open.
            let waiter = {
                let b = Arc::clone(&b);
                thread::spawn(move || {
                    assert_eq!(b.arrive(0), ArriveOutcome::Pending, "waiter arrives first");
                    let outcome = b.wait_macos_for_test(0, Duration::from_secs(5), use_os_sync);
                    (outcome, Instant::now())
                })
            };
            // Waker: let the waiter park, then record t_wake and open (the arrive
            // triggers the kernel/no-op wake in `wake_waiters`).
            let waker = {
                let b = Arc::clone(&b);
                thread::spawn(move || {
                    thread::sleep(waker_delay);
                    let t_wake = Instant::now();
                    assert_eq!(
                        b.arrive(0),
                        ArriveOutcome::Opened(1),
                        "the second arrival is the unique opener"
                    );
                    t_wake
                })
            };
            let (outcome, t_return) = waiter.join().expect("waiter panicked");
            let t_wake = waker.join().expect("waker panicked");
            assert_eq!(outcome, WaitOutcome::Opened, "every A/B round must open");
            lat.push(t_return.saturating_duration_since(t_wake));
        }
        lat
    }

    fn pct(sorted: &[Duration], p: f64) -> Duration {
        // Nearest-rank; `sorted` is non-empty (ROUNDS > 0).
        let idx = (((sorted.len() as f64) * p).ceil() as usize).clamp(1, sorted.len()) - 1;
        sorted[idx]
    }

    let mut os_sync = measure(true, ROUNDS, WAKER_DELAY);
    let mut fallback = measure(false, ROUNDS, WAKER_DELAY);
    os_sync.sort_unstable();
    fallback.sort_unstable();

    eprintln!(
        "barrier wake-latency A/B ({ROUNDS} rounds, tier={}):",
        barrier_wake_tier()
    );
    eprintln!(
        "  os_sync : p50={:?} p99={:?}",
        pct(&os_sync, 0.50),
        pct(&os_sync, 0.99)
    );
    eprintln!(
        "  fallback: p50={:?} p99={:?}",
        pct(&fallback, 0.50),
        pct(&fallback, 0.99)
    );
    // Behavioral guarantee only — every round opened (asserted in `measure`);
    // the numbers are informational (hardware-dependent), never a gate.
}

/// RAII guard that removes a process-global env var on drop, so cleanup runs
/// even if an assertion panics mid-test (cribbed from
/// `chunk_c_ffi_codes_3_4_test.rs`). Used ONLY by the macOS kill-switch test.
#[cfg(target_os = "macos")]
struct EnvVarGuard {
    name: &'static str,
}

#[cfg(target_os = "macos")]
impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

#[cfg(target_os = "macos")]
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

// ---- Tier-3 multi-process harness (Unix, `#[ignore]`'d except the stub-removal pin) ----
//
// Real cross-address-space MAP_SHARED rendezvous + the real per-OS wait shape
// (Linux: futex + CPU park; macOS: boundary spin + chunked sleep-recheck),
// which the in-process tests CANNOT prove. This module is all-Unix, not
// Linux-only: the macOS MappedBarrier arm is the SAME real
// POSIX shm_open + MAP_SHARED path (a by-name registry — separate
// processes = separate registries — could not serve it). Uses the
// `std::process::Command` self-re-exec pattern (NOT fork — fork-after-threads is
// unsafe): each test checks `CHILD_ENV` at the TOP; when set it acts
// as a CHILD participant and `std::process::exit`s, otherwise it is the PARENT
// supervisor that re-execs N children of THE SAME test binary.
//
// Tests: `two_subprocess_rendezvous_over_one_mapped_barrier` (the
// stub-removal regression pin — NOT ignored, runs in the normal suite on
// every Unix: two REAL child processes rendezvous over one MappedBarrier,
// the exact thing the registry stub could never do),
// `box_wide_cross_process_rendezvous` (8-child rendezvous + real park),
// `box_park_vs_spin_latency_ab` (park-vs-spin latency, print-only), and the
// munmap-during-park safety pins —
// `box_parked_waiter_survives_peer_crash_and_owner_unlink` (a parked wait()
// survives a peer SIGKILL + the owner's Drop (its own munmap + shm_unlink): exits
// TimedOut, NEVER SIGSEGV/SIGBUS — POSIX unlink removes the NAME only, the
// object persists until the last munmap) and
// `box_parked_waiter_still_wakes_after_owner_unlink` (the positive twin: after
// the unlink, a surviving pre-opened handle's arrive still OPENS the generation
// and wakes the parked waiter — unlink kills the name, not the mapping or the
// cross-process wake path). The safety pins' child roles ride CHILD_ENV VALUES
// ("parked_waiter" / "sleeper"); the original tests keep the value-agnostic
// `is_ok()` dispatch to `child_worker`.
//
// CI COUNT, because the list above reads like a suite: FIVE `#[test]`s live in
// this module and CI runs exactly ONE of them, the non-ignored
// `two_subprocess_rendezvous_over_one_mapped_barrier`. The other four are
// `#[ignore]`'d hardware-only, so the 8-child rendezvous, the park-vs-spin A/B and
// BOTH munmap-during-park safety pins reach no CI job — their coverage exists
// only on a manual run with `-- --ignored`. That is deliberate (each needs real
// multi-process MAP_SHARED + a real CPU park, and the peer-crash pin parks a
// 120 s `sleeper_child` the parent SIGKILLs), and it is worth stating beside the
// list so a reader does not count five pins where CI enforces one.
#[cfg(unix)]
mod box_harness {
    use super::{test_ns, ArriveOutcome, MappedBarrier, WaitOutcome};
    use std::io;
    use std::process::{Child, Command, ExitStatus};
    use std::time::{Duration, Instant};

    const CHILD_ENV: &str = "CERULION_BARRIER_CHILD";

    /// RAII wrapper around a spawned child process: `Drop` SIGKILLs then reaps it,
    /// so a parent unwind — a failed `assert_eq!`, or a mid-spawn `expect()`
    /// failing on child N while children `0..N` are already live — can never
    /// ORPHAN an already-spawned child (`std::process::Child` does NOT kill on
    /// drop by default). Ported from the `ChildGuard` pattern in
    /// `liveliness_crash_iox2_test.rs`, defined LOCALLY here to avoid a
    /// cross-test dependency.
    ///
    /// `kill()` is best-effort: on an already-exited child (the happy path waits
    /// it first) Rust's `Child::kill` is a no-op — once the child has been
    /// `wait`ed it returns `Ok` WITHOUT signalling, so there is no PID-reuse
    /// hazard — and the `wait()` in `Drop` then returns the cached status. Both
    /// results are ignored in `Drop`.
    struct ChildGuard {
        child: Child,
    }

    impl ChildGuard {
        fn new(child: Child) -> Self {
            Self { child }
        }

        /// Forward to the inner `Child::wait` for the happy-path join (reads the
        /// child's exit status). After this, `Drop` is a no-op: `kill` returns
        /// `Ok` without signalling and `wait` returns the cached status.
        fn wait(&mut self) -> io::Result<ExitStatus> {
            self.child.wait()
        }
    }

    impl Drop for ChildGuard {
        fn drop(&mut self) {
            // Best-effort kill + reap on any unwind / early return so a parent
            // panic never orphans a spawned child. `kill` is a no-op on an
            // already-exited child; `wait` reaps the zombie — both ignored.
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }

    /// `open_unowned` with a bounded retry — the parent creates the segment before
    /// spawning, but exec startup may race the create on a loaded machine.
    fn open_with_retry(ns: &str, id: &str) -> MappedBarrier {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match MappedBarrier::open_unowned(ns, id) {
                Ok(b) => return b,
                Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(5)),
                Err(e) => panic!("child could not open segment within 10s: {e}"),
            }
        }
    }

    /// A re-exec'd CHILD participant: open the segment, run GENS generations using
    /// the REAL parking `wait()`, exit 0 on success / 2 on a wait timeout. Never
    /// returns. Both `#[ignore]` tests re-exec this same worker; the ns/id/gens
    /// arrive via env from whichever parent spawned it.
    // P12 exemption, scoped to this fn rather than the file: this is the body of a
    // SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
    // code IS the channel the parent reads its verdict from. The ban stays armed for
    // every other line in this binary, which is the half a file-wide allow gave up.
    #[allow(clippy::disallowed_methods)]
    fn child_worker() -> ! {
        let ns = std::env::var("CERULION_BARRIER_NS").expect("CERULION_BARRIER_NS");
        let id = std::env::var("CERULION_BARRIER_ID").expect("CERULION_BARRIER_ID");
        let gens: u64 = std::env::var("CERULION_BARRIER_GENS")
            .expect("CERULION_BARRIER_GENS")
            .parse()
            .expect("GENS u64");
        let bar = open_with_retry(&ns, &id);
        for g in 0..gens {
            let _ = bar.arrive(g);
            match bar.wait(g, Duration::from_secs(30)) {
                WaitOutcome::Opened => {}
                WaitOutcome::TimedOut => std::process::exit(2),
            }
        }
        std::process::exit(0);
    }

    /// Stub-removal regression pin — NOT `#[ignore]`'d: runs in the
    /// normal suite on every Unix (Linux and macOS). TWO REAL
    /// child PROCESSES (self-re-exec'd `child_worker`s — separate address
    /// spaces) rendezvous over ONE `MappedBarrier` for several generations,
    /// with the parent as owner/observer only (`expected = 2` = the two
    /// children; the parent never arrives). This is EXACTLY the thing an
    /// in-process by-name registry can never do — separate processes have
    /// separate registries, so a child's `open_unowned` would `NotFound` and
    /// it would exit 3 (open failure) / the parent's generation watch would
    /// time out. A regression back to any in-process-only sharing fails this
    /// pin loudly on macOS while it keeps passing on Linux.
    ///
    /// Hang-safety discipline (same as every harness test): `ChildGuard` Drop
    /// SIGKILLs + reaps on any parent unwind; the parent's generation watch is
    /// HARD-bounded; `wait_bounded` joins with a deadline.
    ///
    /// Small GENS (5) + 2 children keeps it CI-cheap (lockstep rendezvous
    /// completes in ms; the bounded deadlines only pay out on failure).
    #[test]
    fn two_subprocess_rendezvous_over_one_mapped_barrier() {
        if std::env::var(CHILD_ENV).is_ok() {
            child_worker(); // re-exec'd participant — never returns
        }
        const N_CHILDREN: u32 = 2;
        const GENS: u64 = 5;
        let ns = test_ns("2proc");
        let id = "g";
        // The parent OWNS the segment (create + Drop-unlink) but does NOT
        // participate: expected = the two child processes only, so every
        // generation opens purely by CROSS-PROCESS arrivals.
        let owner = MappedBarrier::create_owned(&ns, id, N_CHILDREN).expect("owner create");
        let exe = std::env::current_exe().expect("current_exe");
        let mut kids: Vec<ChildGuard> = (0..N_CHILDREN)
            .map(|_| {
                ChildGuard::new(
                    Command::new(&exe)
                        .args([
                            "--exact",
                            "box_harness::two_subprocess_rendezvous_over_one_mapped_barrier",
                            "--nocapture",
                        ])
                        .env(CHILD_ENV, "1")
                        .env("CERULION_BARRIER_NS", &ns)
                        .env("CERULION_BARRIER_ID", id)
                        .env("CERULION_BARRIER_GENS", GENS.to_string())
                        .spawn()
                        .expect("spawn child"),
                )
            })
            .collect();
        // HARD-bounded generation watch: the two children must drive the
        // shared generation to GENS entirely on their own (the parent never
        // arrives — pure cross-process rendezvous).
        let deadline = Instant::now() + Duration::from_secs(30);
        while owner.current_generation() < GENS {
            assert!(
                Instant::now() < deadline,
                "two-subprocess rendezvous did not reach generation {GENS} within 30s \
                 (generation = {}); a child likely failed to open the shared segment — \
                 the stub-regression signature",
                owner.current_generation()
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        // Bounded join; exit 0 = the child saw every generation open (its
        // wait() returned Opened each time), 2 = a wait timeout, 101 = an
        // open/env panic.
        for (i, mut k) in kids.drain(..).enumerate() {
            let status = wait_bounded(&mut k, Duration::from_secs(10));
            assert!(
                status.success(),
                "child {i} exited non-zero: {:?}",
                status.code()
            );
        }
        assert_eq!(
            owner.current_generation(),
            GENS,
            "all {GENS} generations completed by the two child processes alone"
        );
        drop(owner);
    }

    /// THE real test: N child PROCESSES + the owner rendezvous over a real
    /// MAP_SHARED segment for many generations, each child parking on the real
    /// CPU monitor-wait. Separate page tables prove cross-address-space sharing.
    /// `#[ignore]`'d (needs multiple processes and a real CPU park; any Unix); run with
    /// `cargo test -p cerulion_core --test barrier_test -- --ignored --nocapture`
    /// on a machine with x86 WAITPKG or aarch64 WFE.
    #[test]
    #[ignore = "box-only: real cross-process MAP_SHARED rendezvous + CPU park (multi-process box run; any Unix)"]
    fn box_wide_cross_process_rendezvous() {
        if std::env::var(CHILD_ENV).is_ok() {
            child_worker(); // re-exec'd participant — never returns
        }
        const N_CHILDREN: u32 = 8;
        const GENS: u64 = 200;
        let ns = test_ns("box_xproc");
        let id = "g";
        let owner = MappedBarrier::create_owned(&ns, id, N_CHILDREN + 1).expect("owner create");
        let exe = std::env::current_exe().expect("current_exe");
        // Each spawned child is wrapped in a `ChildGuard` so a parent unwind —
        // a failed owner `assert_eq!`, or `spawn().expect()` failing on child N
        // while `0..N` are already live — kills the already-spawned children
        // instead of orphaning them (the partially-built `Vec` is dropped on the
        // `collect()` unwind, reaping each guard).
        let mut kids: Vec<ChildGuard> = (0..N_CHILDREN)
            .map(|_| {
                ChildGuard::new(
                    Command::new(&exe)
                        .args([
                            "--exact",
                            "box_harness::box_wide_cross_process_rendezvous",
                            "--ignored",
                            "--nocapture",
                        ])
                        .env(CHILD_ENV, "1")
                        .env("CERULION_BARRIER_NS", &ns)
                        .env("CERULION_BARRIER_ID", id)
                        .env("CERULION_BARRIER_GENS", GENS.to_string())
                        .spawn()
                        .expect("spawn child"),
                )
            })
            .collect();
        // Owner is the (N+1)th participant in every generation.
        for g in 0..GENS {
            let _ = owner.arrive(g);
            assert_eq!(
                owner.wait(g, Duration::from_secs(30)),
                WaitOutcome::Opened,
                "owner sees gen {g} open"
            );
        }
        for (i, mut k) in kids.drain(..).enumerate() {
            let status = k.wait().expect("child join");
            assert!(
                status.success(),
                "child {i} exited non-zero: {:?}",
                status.code()
            );
        }
        assert_eq!(
            owner.current_generation(),
            GENS,
            "all {GENS} generations completed"
        );
        drop(owner);
    }

    /// Hardware-only coarse latency A/B: 2-process ping-pong over `ROUNDS` generations,
    /// timing the owner's full per-generation arrive→Opened cycle under the REAL
    /// parking `wait()` vs a busy-spin baseline. PRINT-ONLY (no CI assert on the
    /// numbers — they are hardware-dependent); the headline is that the park
    /// matches the busy-spin latency WITHOUT burning a core. Analyze the printed
    /// p50/p99 on your hardware. Run with `-- --ignored --nocapture`.
    #[test]
    #[ignore = "box-only: cross-process park-vs-spin latency A/B (print-only; any Unix)"]
    fn box_park_vs_spin_latency_ab() {
        // Reuse the same CHILD_ENV switch: a child here just mirrors the owner with
        // the parking `wait()` (the busy-spin variant only changes the OWNER's wait).
        if std::env::var(CHILD_ENV).is_ok() {
            child_worker();
        }
        const ROUNDS: u64 = 2_000;
        let id = "g";

        // One 2-process session: spawn 1 child (re-exec'd `child_worker`, which uses
        // the REAL parking wait() in BOTH variants), then the owner pings each
        // generation. `use_park` selects the OWNER's wait — the parking `wait()` vs a
        // tight busy-spin on `is_open` — and we return the owner's per-round
        // arrive→Opened wall times (ns). Asserts each generation actually opened so a
        // broken session fails loudly (the timing numbers themselves are print-only).
        fn run_session(rounds: u64, id: &str, use_park: bool) -> Vec<u128> {
            let ns = test_ns(if use_park {
                "box_ab_park"
            } else {
                "box_ab_spin"
            });
            let owner = MappedBarrier::create_owned(&ns, id, 2).expect("owner create");
            let exe = std::env::current_exe().expect("current_exe");
            // Wrapped in a `ChildGuard` so a parent unwind (a failed owner
            // `assert_eq!` below, or the spin-deadline panic) reaps the child
            // instead of orphaning it.
            let mut child = ChildGuard::new(
                Command::new(&exe)
                    .args([
                        "--exact",
                        "box_harness::box_park_vs_spin_latency_ab",
                        "--ignored",
                        "--nocapture",
                    ])
                    .env(CHILD_ENV, "1")
                    .env("CERULION_BARRIER_NS", &ns)
                    .env("CERULION_BARRIER_ID", id)
                    .env("CERULION_BARRIER_GENS", rounds.to_string())
                    .spawn()
                    .expect("spawn child"),
            );

            let mut times = Vec::with_capacity(rounds as usize);
            for g in 0..rounds {
                let t0 = Instant::now();
                let _ = owner.arrive(g);
                if use_park {
                    assert_eq!(
                        owner.wait(g, Duration::from_secs(30)),
                        WaitOutcome::Opened,
                        "park session: owner sees gen {g} open"
                    );
                } else {
                    // Wall-clock-bounded busy-spin: like every other wait in this
                    // file (`wait_open_capped`), PANIC rather than hang if the
                    // generation never opens (e.g. the child died). The deadline is
                    // checked only every 1024th iteration so the hot-spin latency
                    // measurement is not perturbed by the `Instant::now()` cost.
                    let deadline = Instant::now() + Duration::from_secs(30);
                    let mut spins: u32 = 0;
                    while !owner.is_open(g) {
                        std::hint::spin_loop();
                        spins = spins.wrapping_add(1);
                        if spins.is_multiple_of(1024) && Instant::now() >= deadline {
                            panic!(
                                "spin baseline: gen {g} never opened within 30s — the child likely died"
                            );
                        }
                    }
                }
                times.push(t0.elapsed().as_nanos());
            }
            let status = child.wait().expect("child join");
            assert!(
                status.success(),
                "latency A/B child exited non-zero: {:?}",
                status.code()
            );
            assert_eq!(owner.current_generation(), rounds, "all rounds completed");
            drop(owner);
            times
        }

        // Nearest-rank percentile over an ascending-sorted slice (len >= 1 here).
        fn percentile(sorted: &[u128], p: f64) -> u128 {
            if sorted.is_empty() {
                return 0;
            }
            let idx = (((sorted.len() - 1) as f64) * p).round() as usize;
            sorted[idx]
        }

        let mut park = run_session(ROUNDS, id, true);
        let mut spin = run_session(ROUNDS, id, false);
        park.sort_unstable();
        spin.sort_unstable();

        eprintln!("box_park_vs_spin_latency_ab: owner arrive→Opened over {ROUNDS} rounds (ns)");
        eprintln!(
            "  park p50={} p99={}",
            percentile(&park, 0.50),
            percentile(&park, 0.99)
        );
        eprintln!(
            "  spin p50={} p99={}",
            percentile(&spin, 0.50),
            percentile(&spin, 0.99)
        );
    }

    // ---- Munmap-during-park safety pins ----
    //
    // POSIX contract under pin: `shm_unlink` removes the NAME only; the object
    // persists until the LAST `munmap`, so an existing mapping — and the CPU
    // monitor-wait/WFE park armed on it — stays valid across a peer crash and the
    // owner's teardown. Exit codes are the ORACLE (bare `ExitStatus`, never
    // through a pipe): 0 = the outcome named in `CERULION_BARRIER_EXPECT`
    // ("timeout" | "opened"), 4 = the OTHER wait outcome, 3 = setup failure.

    /// Read an env var for an oracle-exit-code child role, exiting 3 (setup
    /// failure) when absent — a panic's 101 would be indistinguishable noise to
    /// the parent's exit-code decode.
    // P12 exemption, scoped to this fn rather than the file: it is called ONLY from
    // the self-re-exec child roles below, where exit(3) is the setup-failure code the
    // parent decodes (a panic's 101 would be indistinguishable noise). The ban stays
    // armed for every other line in this binary.
    #[allow(clippy::disallowed_methods)]
    fn env_or_exit3(key: &str) -> String {
        match std::env::var(key) {
            Ok(v) => v,
            Err(_) => std::process::exit(3),
        }
    }

    /// `open_unowned` with a bounded retry that EXITs 3 on failure — the
    /// oracle-exit-code sibling of `open_with_retry` (which panics).
    // P12 exemption, scoped to this fn rather than the file: it is called ONLY from
    // the self-re-exec child roles below, where exit(3) is the setup-failure code the
    // parent decodes (a panic's 101 would be indistinguishable noise). The ban stays
    // armed for every other line in this binary.
    #[allow(clippy::disallowed_methods)]
    fn open_or_exit3(ns: &str, id: &str) -> MappedBarrier {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match MappedBarrier::open_unowned(ns, id) {
                Ok(b) => return b,
                Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(5)),
                Err(_) => std::process::exit(3),
            }
        }
    }

    /// A re-exec'd PARKED-WAITER child (`CHILD_ENV = "parked_waiter"`): opens the
    /// barrier, `arrive(0)` (1 of 2 — the cohort stays incomplete until the parent
    /// decides), writes the READY sentinel, then PARKS in
    /// `wait(0, CERULION_BARRIER_WAIT_MS)`. READY is written AFTER the arrive,
    /// immediately BEFORE the park, so the parent knows the waiter is heading into
    /// `wait` (its settle sleep covers the last few instructions). Exit code =
    /// oracle (see the section comment). Never returns.
    // P12 exemption, scoped to this fn rather than the file: this is the body of a
    // SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
    // code IS the channel the parent reads its verdict from. The ban stays armed for
    // every other line in this binary, which is the half a file-wide allow gave up.
    #[allow(clippy::disallowed_methods)]
    fn parked_waiter_child() -> ! {
        let ns = env_or_exit3("CERULION_BARRIER_NS");
        let id = env_or_exit3("CERULION_BARRIER_ID");
        let ready = env_or_exit3("CERULION_BARRIER_READY");
        let wait_ms: u64 = match env_or_exit3("CERULION_BARRIER_WAIT_MS").parse() {
            Ok(v) => v,
            Err(_) => std::process::exit(3),
        };
        let expected = match env_or_exit3("CERULION_BARRIER_EXPECT").as_str() {
            "timeout" => WaitOutcome::TimedOut,
            "opened" => WaitOutcome::Opened,
            _ => std::process::exit(3),
        };
        let bar = open_or_exit3(&ns, &id);
        let _ = bar.arrive(0);
        if std::fs::write(&ready, b"parked").is_err() {
            std::process::exit(3);
        }
        let outcome = bar.wait(0, Duration::from_millis(wait_ms));
        std::process::exit(if outcome == expected { 0 } else { 4 });
    }

    /// A re-exec'd SLEEPER child (`CHILD_ENV = "sleeper"`): opens (and HOLDS a
    /// mapping of) the barrier but never arrives — the "peer that never arrives",
    /// SIGKILLed by the parent to model a mid-run CRASH. The long sleep is a
    /// backstop only (the guard Drop reaps it on any parent unwind). Never
    /// returns normally in practice.
    // P12 exemption, scoped to this fn rather than the file: this is the body of a
    // SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
    // code IS the channel the parent reads its verdict from. The ban stays armed for
    // every other line in this binary, which is the half a file-wide allow gave up.
    #[allow(clippy::disallowed_methods)]
    fn sleeper_child() -> ! {
        let ns = env_or_exit3("CERULION_BARRIER_NS");
        let id = env_or_exit3("CERULION_BARRIER_ID");
        let bar = open_or_exit3(&ns, &id);
        std::thread::sleep(Duration::from_secs(120));
        drop(bar);
        std::process::exit(0);
    }

    /// Bounded poll for a sentinel file (the harness's READY style) — panics
    /// loudly (→ guard Drops reap the children) rather than hanging a manual run.
    fn wait_for_file(path: &std::path::Path, deadline: Duration) {
        let end = Instant::now() + deadline;
        while !path.exists() {
            assert!(
                Instant::now() < end,
                "READY sentinel {path:?} not written within {deadline:?}"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// Bounded join: poll `try_wait` until exit or `deadline` elapses. Panics on
    /// the deadline (→ the guard's Drop SIGKILLs + reaps) — a multi-process test must never
    /// hang on a wedged child. The exit status is read BARE via `try_wait` (never
    /// through a pipe).
    fn wait_bounded(guard: &mut ChildGuard, deadline: Duration) -> ExitStatus {
        let end = Instant::now() + deadline;
        loop {
            match guard.child.try_wait() {
                Ok(Some(status)) => return status,
                Ok(None) if Instant::now() < end => std::thread::sleep(Duration::from_millis(20)),
                Ok(None) => panic!("child did not exit within {deadline:?} (guard will SIGKILL)"),
                Err(e) => panic!("try_wait on child failed: {e}"),
            }
        }
    }

    /// Pins (a)+(b): a PARKED `wait()` survives BOTH a peer process
    /// crashing AND the owner tearing its `MappedBarrier` down (Drop =
    /// the owner's own `munmap` + `shm_unlink`) — the waiter runs its park to
    /// the full timeout and exits NORMALLY with the TimedOut oracle code, NEVER
    /// dies by SIGSEGV/SIGBUS. POSIX: unlink removes only the NAME; the object
    /// persists until the last munmap, so the waiter's mapping (and the
    /// WFE/monitor park armed on it) stays memory-backed until its own exit.
    ///
    /// Cohort: expected = 2. Waiter W arrives (1 of 2) then parks; sleeper C holds
    /// a mapping and never arrives. Parent: waits for W's READY (written just
    /// before the park) + a settle, SIGKILLs C (the crash), then DROPS the owner
    /// (unlink + parent munmap) while W is still parked. W's generation can never
    /// open → exit 0 = TimedOut after ~the full 8s window.
    #[test]
    #[ignore = "box-only: munmap/unlink-during-park safety — parked waiter survives peer crash + owner teardown (multi-process box run; any Unix)"]
    fn box_parked_waiter_survives_peer_crash_and_owner_unlink() {
        if let Ok(role) = std::env::var(CHILD_ENV) {
            match role.as_str() {
                "parked_waiter" => parked_waiter_child(),
                "sleeper" => sleeper_child(),
                other => panic!("unknown child role {other:?}"),
            }
        }
        const WAIT_MS: u64 = 8_000;
        let ns = test_ns("box_park_crash");
        let id = "g";
        let owner = MappedBarrier::create_owned(&ns, id, 2).expect("owner create");
        let exe = std::env::current_exe().expect("current_exe");
        let ready = std::env::temp_dir().join(format!("ch5_crash_ready_{}", std::process::id()));
        let _ = std::fs::remove_file(&ready);

        let mut w = ChildGuard::new(
            Command::new(&exe)
                .args([
                    "--exact",
                    "box_harness::box_parked_waiter_survives_peer_crash_and_owner_unlink",
                    "--ignored",
                    "--nocapture",
                ])
                .env(CHILD_ENV, "parked_waiter")
                .env("CERULION_BARRIER_NS", &ns)
                .env("CERULION_BARRIER_ID", id)
                .env("CERULION_BARRIER_READY", &ready)
                .env("CERULION_BARRIER_WAIT_MS", WAIT_MS.to_string())
                .env("CERULION_BARRIER_EXPECT", "timeout")
                .spawn()
                .expect("spawn parked-waiter child"),
        );
        let mut c = ChildGuard::new(
            Command::new(&exe)
                .args([
                    "--exact",
                    "box_harness::box_parked_waiter_survives_peer_crash_and_owner_unlink",
                    "--ignored",
                    "--nocapture",
                ])
                .env(CHILD_ENV, "sleeper")
                .env("CERULION_BARRIER_NS", &ns)
                .env("CERULION_BARRIER_ID", id)
                .spawn()
                .expect("spawn sleeper child"),
        );

        // W is provably heading into the park: READY lands just before wait();
        // the settle sleep covers the few instructions between the write and the
        // actual park arming.
        wait_for_file(&ready, Duration::from_secs(15));
        let parked_at = Instant::now();
        std::thread::sleep(Duration::from_millis(300));

        // (a) the CRASH: SIGKILL the sleeping peer (no cleanup runs), reap it.
        let _ = c.child.kill();
        let _ = c.child.wait();

        // (b) the TEARDOWN: drop the owner while W is parked — the parent's
        // own munmap + shm_unlink (the name dies). W's mapping must stay valid.
        drop(owner);

        // W must exit NORMALLY: code 0 = TimedOut (4 = unexpectedly Opened,
        // 3 = setup failure), and specifically NOT killed by a signal — a
        // SIGSEGV/SIGBUS here would mean the park's mapping died with the unlink.
        let status = wait_bounded(&mut w, Duration::from_secs(25));
        use std::os::unix::process::ExitStatusExt;
        assert_eq!(
            status.signal(),
            None,
            "the parked waiter was KILLED BY A SIGNAL (SIGSEGV/SIGBUS would mean \
             the unlink/munmap invalidated the parked mapping): {status:?}"
        );
        assert_eq!(
            status.code(),
            Some(0),
            "waiter oracle: 0 = TimedOut expected (4 = unexpectedly Opened, \
             3 = setup failure); got {status:?}"
        );
        // ~ the full timeout: the park genuinely survived to its deadline rather
        // than aborting early (parked_at is the parent's READY observation, so
        // this lower bound is conservative).
        let waited = parked_at.elapsed();
        assert!(
            waited >= Duration::from_millis(WAIT_MS - 500),
            "waiter exited after only {waited:?} — it should have parked ~the \
             full {WAIT_MS}ms window"
        );
        let _ = std::fs::remove_file(&ready);
    }

    /// The POSITIVE twin: after the owner's teardown unlinks the
    /// NAME, a SECOND peer handle opened BEFORE the drop still opens the
    /// generation and WAKES the parked waiter — proving `shm_unlink` kills the
    /// name only, not the live mapping or the cross-process wake path (the
    /// opener's `generation` store through ITS mapping wakes the monitor/WFE
    /// armed on the waiter's mapping of the same physical page).
    ///
    /// Cohort: expected = 2. W arrives (1 of 2) then parks with a 10s timeout.
    /// Parent: keeps a surviving `open_unowned` handle from BEFORE the owner
    /// drop; drops the owner (fresh opens now ENOENT — asserted); then arrives
    /// through the surviving handle (2 of 2 → Opened). W must exit 0 = Opened,
    /// not signaled, well BEFORE its timeout (within ~5s of the arrive).
    #[test]
    #[ignore = "box-only: unlink kills the NAME, not the mapping/wake path — parked waiter still wakes (multi-process box run; any Unix)"]
    fn box_parked_waiter_still_wakes_after_owner_unlink() {
        if let Ok(role) = std::env::var(CHILD_ENV) {
            match role.as_str() {
                "parked_waiter" => parked_waiter_child(),
                other => panic!("unknown child role {other:?}"),
            }
        }
        const WAIT_MS: u64 = 10_000;
        let ns = test_ns("box_park_wake");
        let id = "g";
        let owner = MappedBarrier::create_owned(&ns, id, 2).expect("owner create");
        // The SURVIVING peer handle — opened while the name still exists; its
        // mapping (and the wake path through it) must outlive the name.
        let survivor = MappedBarrier::open_unowned(&ns, id).expect("survivor open");
        let exe = std::env::current_exe().expect("current_exe");
        let ready = std::env::temp_dir().join(format!("ch5_wake_ready_{}", std::process::id()));
        let _ = std::fs::remove_file(&ready);

        let mut w = ChildGuard::new(
            Command::new(&exe)
                .args([
                    "--exact",
                    "box_harness::box_parked_waiter_still_wakes_after_owner_unlink",
                    "--ignored",
                    "--nocapture",
                ])
                .env(CHILD_ENV, "parked_waiter")
                .env("CERULION_BARRIER_NS", &ns)
                .env("CERULION_BARRIER_ID", id)
                .env("CERULION_BARRIER_READY", &ready)
                .env("CERULION_BARRIER_WAIT_MS", WAIT_MS.to_string())
                .env("CERULION_BARRIER_EXPECT", "opened")
                .spawn()
                .expect("spawn parked-waiter child"),
        );

        wait_for_file(&ready, Duration::from_secs(15));
        std::thread::sleep(Duration::from_millis(300));

        // Tear the OWNER down FIRST, while W is parked: the name dies here.
        drop(owner);
        // The unlink provably landed BEFORE the wake below — a fresh open must
        // ENOENT (proving the owner drop is not a no-op).
        assert!(
            MappedBarrier::open_unowned(&ns, id).is_err(),
            "the name must be unlinked after the owner drop (fresh opens ENOENT)"
        );

        // Complete the cohort THROUGH THE SURVIVING HANDLE: W already arrived
        // (1 of 2, before READY), so this arrive is 2 of 2 → the unique opener.
        let arrive_at = Instant::now();
        assert_eq!(
            survivor.arrive(0),
            ArriveOutcome::Opened(1),
            "the surviving handle's arrive completes the cohort and opens gen 0"
        );

        let status = wait_bounded(&mut w, Duration::from_secs(25));
        let woke_after = arrive_at.elapsed();
        use std::os::unix::process::ExitStatusExt;
        assert_eq!(
            status.signal(),
            None,
            "the parked waiter was KILLED BY A SIGNAL (SIGSEGV/SIGBUS would mean \
             the unlink invalidated the parked mapping): {status:?}"
        );
        assert_eq!(
            status.code(),
            Some(0),
            "waiter oracle: 0 = Opened expected (4 = TimedOut — the wake path \
             died with the name; 3 = setup failure); got {status:?}"
        );
        // The wake must be prompt — well before the 10s timeout (the ~100µs
        // monitor recheck bounds it; 5s is a generous loaded-machine margin).
        assert!(
            woke_after < Duration::from_secs(5),
            "waiter took {woke_after:?} to exit after the surviving arrive — the \
             wake path must fire well before the {WAIT_MS}ms timeout"
        );
        let _ = std::fs::remove_file(&ready);
        drop(survivor);
    }
}

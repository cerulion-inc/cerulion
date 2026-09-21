// SPDX-License-Identifier: AGPL-3.0-only
//! The child BREADCRUMB over a REAL `fork(2)`, and the ONE property
//! no in-process test can see — that a child's progress stamps reach the PARENT.
//!
//! The in-module oracles in `state_carrier::breadcrumb` drive the page from one
//! process, where `MAP_SHARED` and `MAP_PRIVATE` behave identically, so they are
//! structurally blind to the mapping flag. Get it wrong and the whole watchdog
//! inverts: every stamp the child makes lands in the CHILD's copy-on-write copy, the
//! parent watches a frozen counter, and EVERY child — healthy giants included — is
//! SIGKILLed at the first `STATE_STALL_TIMEOUT_NS`. That is the cost-derived refusal
//! the liveness watchdog exists to remove, re-created by one `mmap` flag.
//!
//! So the headline arm forks a real child, has it stamp through the production API,
//! and reads the page back in the parent — with a hand-built `MAP_PRIVATE` page beside
//! it in the SAME body as the mutation control. Without that control "the parent sees
//! the stamps" is satisfiable by a test that never really forked.
//!
//! Every child here follows the fork child's discipline (`_exit`, never `exit`): a libtest child
//! that unwound or ran destructors would tear down the harness's own state in a
//! process that owns none of it — the unwinding hazard the panic hook exists for, reachable
//! from a test as easily as from production.
//!
//! `#![cfg(unix)]` (`fork`/`waitpid`/`mmap`); parallel-safe — the page is anonymous, so
//! it has no name to collide on, and every wait is bounded and targeted at this test's
//! OWN pid (never `waitpid(-1)`, which would steal a sibling's child).

#![cfg(unix)]

use cerulion_core::state_carrier::{ChildPhase, MappedBreadcrumb, BREADCRUMB_BYTES};

/// A generous liveness ceiling for a child that does microseconds of work. Load can
/// delay it; nothing can make a correct child exceed it.
const CHILD_WAIT_CEILING: std::time::Duration = std::time::Duration::from_secs(30);

/// Bounded, TARGETED reap — never `waitpid(-1)`.
///
/// The reason: this process already owns other children in production
/// (workers, `bagd`, the gateway), so a wildcard wait would steal their exit status or
/// theirs would steal ours. Under libtest the same rule keeps a parallel sibling's
/// child out of this reap.
fn reap(pid: libc::pid_t) -> libc::c_int {
    let deadline = std::time::Instant::now() + CHILD_WAIT_CEILING;
    loop {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(
            r >= 0,
            "waitpid({pid}) failed: {}",
            std::io::Error::last_os_error()
        );
        assert!(
            std::time::Instant::now() < deadline,
            "child {pid} did not exit within {CHILD_WAIT_CEILING:?}"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

fn exited_with(status: libc::c_int) -> Option<libc::c_int> {
    if libc::WIFEXITED(status) {
        Some(libc::WEXITSTATUS(status))
    } else {
        None
    }
}

#[test]
fn a_forked_childs_stamps_are_visible_to_the_parent_and_a_private_page_proves_it() {
    const BUMPS: u64 = 5_000;
    const NODE: u32 = 11;
    const FIELD: u32 = 4;

    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");

    // The CONTROL: an identical page mapped MAP_PRIVATE. It is written by the same
    // child, through the same stores, in the same order — the ONLY difference is the
    // flag. Without it, the assertion below is satisfied by any test that forgot to
    // fork at all.
    let private = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            BREADCRUMB_BYTES,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
            -1,
            0,
        )
    };
    assert_ne!(private, libc::MAP_FAILED, "control mmap failed");
    let private_counter = private as *mut u64;
    unsafe { private_counter.write(0) };

    // SAFETY: everything the child touches below is async-signal-safe or a relaxed
    // atomic store into memory mapped before the fork; it never allocates, never takes
    // a lock, and leaves via `_exit`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());

    if pid == 0 {
        // ---- CHILD ----
        crumb.enter_node(NODE);
        crumb.enter_field(FIELD);
        for i in 0..BUMPS {
            crumb.bump();
            // Same count, same loop, into the private page.
            unsafe { private_counter.write(i + 1) };
        }
        crumb.set_phase(ChildPhase::Finishing);
        // `_exit`, never `exit`: no destructors, no atexit, nothing of the harness's
        // torn down by a process that owns none of it.
        unsafe { libc::_exit(0) };
    }

    // ---- PARENT ----
    let status = reap(pid);
    assert_eq!(exited_with(status), Some(0), "child must exit cleanly");

    assert_eq!(
        crumb.progress(),
        BUMPS,
        "a MAP_SHARED breadcrumb must carry the child's stamps to the parent; \
         a MAP_PRIVATE one would leave the watchdog watching a frozen counter and \
         SIGKILL every healthy child at the first stall timeout"
    );
    assert_eq!(crumb.node_idx(), NODE);
    assert_eq!(crumb.field_idx(), FIELD);
    assert_eq!(crumb.phase(), ChildPhase::Finishing);

    // The control, which is what makes the assertion above mean anything: the child
    // really ran and really wrote, and a PRIVATE page still shows the parent nothing.
    let seen_private = unsafe { private_counter.read() };
    unsafe { libc::munmap(private, BREADCRUMB_BYTES) };
    assert_eq!(
        seen_private, 0,
        "the MAP_PRIVATE control must NOT show the child's writes — if it does, the \
         child did not run in a separate address space and the headline assertion is \
         vacuous"
    );
}

#[test]
fn a_rearmed_page_gives_the_next_child_a_clean_baseline() {
    // Two children in sequence over ONE mapping — the production shape, since the page
    // is mapped at arm time and re-armed per anchor rather than re-mapped. Carrying
    // the first child's high-water mark forward would make the second child's stamps
    // read as no advance at all, so the watchdog would kill a healthy child.
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");

    for (round, bumps) in [(0u32, 900u64), (1, 3u64)] {
        crumb.rearm();
        assert_eq!(
            crumb.progress(),
            0,
            "round {round} must start from a clean page"
        );

        // SAFETY: as above — the child only stamps and `_exit`s.
        let pid = unsafe { libc::fork() };
        assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
        if pid == 0 {
            crumb.enter_node(round);
            for _ in 0..bumps {
                crumb.bump();
            }
            unsafe { libc::_exit(0) };
        }
        let status = reap(pid);
        assert_eq!(exited_with(status), Some(0), "round {round} child exit");
        assert_eq!(
            crumb.progress(),
            bumps,
            "round {round} must see EXACTLY its own child's stamps"
        );
        assert_eq!(crumb.node_idx(), round);
    }
}

#[test]
fn a_child_killed_mid_stamp_leaves_the_page_readable_and_the_parent_running() {
    // The watchdog's own path: the parent SIGKILLs a child that will not stop. The
    // page must survive (it is the parent's mapping — the child only shared it), the
    // last stamps must still be readable so the report can NAME the node and phase,
    // and the parent must not be taken down with it.
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");
    const NODE: u32 = 3;

    // The readiness signal is a PIPE, deliberately NOT the breadcrumb.
    //
    // Gating this arm's precondition on `progress() > 0` would make it wait on the very
    // mechanism under test: a mapping-flag regression would hang here for the whole
    // liveness ceiling and then fail on a PRECONDITION, reporting "the child never
    // stamped" for a bug that is really "the parent cannot see stamps" — and leaving a
    // paused child behind while it did so. A pipe answers "did the child run?"
    // independently, so a mapping-flag regression fails fast, on the assertion that names it.
    let mut fds = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
    let (read_fd, write_fd) = (fds[0], fds[1]);

    // SAFETY: the child stamps once, reports through the pipe, then blocks forever in
    // `pause(2)` — async-signal-safe throughout, and the closest thing to the "wedged
    // encoder" shape the watchdog exists to bound.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        crumb.enter_node(NODE);
        crumb.bump();
        crumb.set_phase(ChildPhase::RingFull);
        let byte = [1u8];
        unsafe { libc::write(write_fd, byte.as_ptr() as *const libc::c_void, 1) };
        loop {
            unsafe { libc::pause() };
        }
    }

    // A guard, because every assertion below can panic and a paused child would
    // otherwise outlive the test.
    struct ChildGuard(libc::pid_t);
    impl Drop for ChildGuard {
        fn drop(&mut self) {
            unsafe {
                libc::kill(self.0, libc::SIGKILL);
                let mut st: libc::c_int = 0;
                libc::waitpid(self.0, &mut st, 0);
            }
        }
    }

    unsafe { libc::close(write_fd) };
    let mut pfd = libc::pollfd {
        fd: read_fd,
        events: libc::POLLIN,
        revents: 0,
    };
    let ready = unsafe { libc::poll(&mut pfd, 1, CHILD_WAIT_CEILING.as_millis() as libc::c_int) };
    if ready != 1 {
        drop(ChildGuard(pid));
        unsafe { libc::close(read_fd) };
        panic!("child did not report readiness within {CHILD_WAIT_CEILING:?}");
    }
    unsafe { libc::close(read_fd) };

    assert_eq!(
        unsafe { libc::kill(pid, libc::SIGKILL) },
        0,
        "SIGKILL the child"
    );
    let status = reap(pid);
    assert_eq!(
        exited_with(status),
        None,
        "a SIGKILLed child has no exit code"
    );
    assert!(libc::WIFSIGNALED(status));
    assert_eq!(libc::WTERMSIG(status), libc::SIGKILL);

    // The diagnosis survives the kill — this is what makes `ChildStalled{node, phase}`
    // a diagnosis rather than a shrug.
    assert_eq!(crumb.progress(), 1);
    assert_eq!(crumb.node_idx(), NODE);
    assert_eq!(crumb.phase(), ChildPhase::RingFull);
    assert!(
        crumb.is_initialised(),
        "the page outlives the child that shared it"
    );
}

// ===========================================================================
// The phase the PRODUCTION sink publishes
// ===========================================================================

/// A `BumpingSink` write onto a FULL ring publishes `RingFull` for the whole
/// blocking push, and restores `Encoding` after it.
///
/// This is the arm that proves `push_with_backpressure_phase` HAS a production
/// caller. The classifier
/// (`StallWatch` → `Backpressured` → `SkipCause::RecorderBehind`) and its unit tests
/// pass whether or not anything outside a test ever sets `RingFull`. With no
/// production caller, a recorder that stops draining freezes the child at
/// `Encoding`, is classified `Stalled` → `ChildTimeout`, and `blames_the_node()`
/// names the one component that is working.
///
/// The oracle is the phase OBSERVED while the push is genuinely blocked — read from
/// another thread through the shared mapping, exactly as the reaper reads it — not a
/// phase read after the fact, which a wrapper that stamped and restored around a
/// non-blocking write would also satisfy.
#[test]
fn a_blocked_ring_write_publishes_ring_full_while_it_is_blocked() {
    use cerulion_core::state::StateSink;
    use cerulion_core::state_carrier::BumpingSink;
    use cerulion_core::state_ring::StateRingOwner;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    const CAP: u32 = 4;
    /// Long enough that the observer below has a real window, short enough that a
    /// wedge reports itself quickly. Nothing here asserts on elapsed time.
    const WAIT: Duration = Duration::from_secs(2);
    const DEADLINE: Duration = Duration::from_secs(10);

    let tag = format!("a8_{}", std::process::id());
    let mut owner = StateRingOwner::create(&tag, CAP, 0, 0xA8, &["n0"]).expect("ring");
    let mut producer = owner.producer().expect("producer");
    producer.set_backpressure_wait_timeout(WAIT);

    let crumb = Arc::new(MappedBreadcrumb::create().expect("map breadcrumb"));
    // The phase a node's encode runs under, set by `enter_node` in production.
    crumb.enter_node(0);
    assert_eq!(crumb.phase(), ChildPhase::Encoding, "the encoding baseline");

    // Fill the ring. No consumer exists, so its read cursor never moves and the
    // NEXT write must block — which is the whole premise of the arm.
    //
    // MORE than a whole record per write, deliberately: `StateChunker` emits a
    // record only once one is full AND more bytes follow, so a write of exactly
    // `STATE_RECORD_PAYLOAD` bytes buffers and pushes NOTHING. `CAP + 1` payloads
    // emit exactly `CAP` records and leave the last buffered — the ring is full and
    // no push has waited yet.
    let payload = cerulion_core::state_ring::STATE_RECORD_PAYLOAD;
    {
        let mut sink = producer.sink(0, 0);
        sink.write(&vec![0u8; payload * (CAP as usize + 1)])
            .expect("a ring with room accepts");
        // DROPPED, never finished: `finish` would emit the buffered tail, and this
        // sink exists only to fill the ring.
        drop(sink);
    }
    assert_eq!(
        producer.free_records(),
        Some(0),
        "the fixture must have genuinely filled the ring, or the arm below is vacuous"
    );

    let writing = Arc::new(AtomicBool::new(false));
    let payload_for_thread = payload;
    let c2 = Arc::clone(&crumb);
    let w2 = Arc::clone(&writing);
    let writer = std::thread::spawn(move || {
        let mut sink = producer.sink(1, 0);
        let mut bumping = BumpingSink::new(&mut sink, &c2);
        w2.store(true, Ordering::Release);
        // Blocks: the ring is full and nothing drains it. It returns when the
        // producer's wait ceiling expires (and laps) — that is the ring's contract,
        // not this test's subject.
        let _ = bumping.write(&vec![1u8; payload_for_thread * 2]);
        drop(sink);
    });

    // THE ARM: catch the phase WHILE the push is blocked.
    let start = Instant::now();
    let mut seen_ring_full = false;
    while start.elapsed() < DEADLINE {
        if writing.load(Ordering::Acquire) && crumb.phase() == ChildPhase::RingFull {
            seen_ring_full = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    writer.join().expect("writer thread");

    assert!(
        seen_ring_full,
        "a ring write that is BLOCKING must publish ChildPhase::RingFull — without it \
         the reaper classifies a recorder that stopped draining as a node that \
         wedged, and blames the node"
    );
    assert_eq!(
        crumb.phase(),
        ChildPhase::Encoding,
        "and the phase must be RESTORED, so a stall starting just after a push does \
         not inherit it and blame a healthy recorder"
    );
    drop(owner);
}

/// `sink.finish()` publishes `RingFull` too — the guard is per WRITE, not per node.
///
/// Guarding `BumpingSink::write` and the header write is not enough, because
/// `finish()` is neither: it EMITS the node's final record, so on a full ring it
/// blocks exactly as a body write does. Unguarded, a child that encoded a whole node and then
/// stalled at its LAST push would still publish `Encoding`, and the watchdog
/// would blame the node for a recorder that had stopped draining — the same defect,
/// one call site further along.
///
/// Driven through the PRODUCTION encoder (`ForkSetEncoder`, the `NodeEncoder`
/// `child_main` runs), not by calling `finish()` from the test: the guard lives in the
/// carrier, so a test that wrapped the call itself would be pinning its own code.
///
/// The fixture puts the block precisely at `finish()`. `StateChunker` emits a record
/// only once one is full AND more bytes follow, so a capture writing `CAP` whole
/// payloads plus a remainder emits exactly `CAP` records — filling the ring, none of
/// them waiting — and leaves the remainder buffered for `finish()` to emit into a ring
/// with no room. The header is 16 bytes and rides the same first record.
///
/// The observer only accepts a `RingFull` seen AFTER the capture returns, so the
/// transient stamps the body writes (correctly, already) publish cannot satisfy it.
#[test]
fn a_blocked_finish_publishes_ring_full_not_just_a_blocked_body_write() {
    use cerulion_core::state::{StateError, StateSink};
    use cerulion_core::state_carrier::{ForkCaptureTarget, ForkSetEncoder, NodeEncoder};
    use cerulion_core::state_ring::StateRingOwner;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    const CAP: u32 = 4;
    const WAIT: Duration = Duration::from_secs(2);
    const DEADLINE: Duration = Duration::from_secs(10);

    /// A target whose capture writes enough to fill the ring EXACTLY and leave a
    /// remainder buffered, then flags that the only ring write still to come is the
    /// final record.
    struct FillThenFinish {
        bytes: usize,
        captured: Arc<AtomicBool>,
    }
    impl ForkCaptureTarget for FillThenFinish {
        fn node_idx(&self) -> u32 {
            0
        }
        fn state_shape(&self) -> Option<u64> {
            Some(0xF1)
        }
        fn capture(&self, sink: &mut dyn StateSink) -> Result<(), StateError> {
            sink.write(&vec![7u8; self.bytes])?;
            self.captured.store(true, Ordering::Release);
            Ok(())
        }
    }

    let tag = format!("a8fin_{}", std::process::id());
    let mut owner = StateRingOwner::create(&tag, CAP, 0, 0xA8F, &["n0"]).expect("ring");
    let mut producer = owner.producer().expect("producer");
    producer.set_backpressure_wait_timeout(WAIT);

    let crumb = Arc::new(MappedBreadcrumb::create().expect("breadcrumb"));
    let payload = cerulion_core::state_ring::STATE_RECORD_PAYLOAD;
    // The 16-byte v1 header rides the first record, so the body writes one header
    // less than CAP whole payloads, plus a remainder for `finish()` to emit.
    let body = payload * (CAP as usize) - 16 + 16;
    let captured = Arc::new(AtomicBool::new(false));

    let c2 = Arc::clone(&crumb);
    let cap2 = Arc::clone(&captured);
    let writer = std::thread::spawn(move || {
        let targets = [FillThenFinish {
            bytes: body,
            captured: cap2,
        }];
        let mut enc = ForkSetEncoder::new(&targets, &mut producer, 1);
        enc.encode(0, &c2)
    });

    let start = Instant::now();
    let mut seen = false;
    while start.elapsed() < DEADLINE {
        if captured.load(Ordering::Acquire) && crumb.phase() == ChildPhase::RingFull {
            seen = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    let outcome = writer.join().expect("writer thread");

    assert!(
        seen,
        "a BLOCKED `finish()` must publish ChildPhase::RingFull — otherwise a child \
         that stalls on its LAST push is classified ChildTimeout and the operator's \
         line names the node instead of the recorder (outcome was {outcome:?})"
    );
    assert_eq!(
        crumb.phase(),
        ChildPhase::Encoding,
        "and the phase must be restored afterwards"
    );
    drop(owner);
}

// ===========================================================================
// PRECEDENCE: a published Complete beats a post-mortem Skip
// ===========================================================================

/// One node of a fork set, capturing a fixed payload.
struct FixedTarget {
    node_idx: u32,
    payload: Vec<u8>,
}

impl cerulion_core::state_carrier::ForkCaptureTarget for FixedTarget {
    fn node_idx(&self) -> u32 {
        self.node_idx
    }
    fn state_shape(&self) -> Option<u64> {
        Some(0x5EED)
    }
    fn capture(
        &self,
        sink: &mut dyn cerulion_core::state::StateSink,
    ) -> Result<(), cerulion_core::state::StateError> {
        Ok(sink.write(&self.payload)?)
    }
}

/// Read a ring back through the PRODUCTION assembler.
fn assemble_ring(name: &str) -> (Vec<cerulion_core::state_ring::StateAnchorEvent>, u64) {
    let mut consumer = cerulion_core::state_ring::StateRingConsumer::open(name).expect("open ring");
    let mut asm = cerulion_core::state_ring::StateAssembler::passthrough();
    let mut events = Vec::new();
    consumer.drain(&mut asm, &mut events).expect("drain");
    events.extend(asm.finish());
    (events, asm.skips_after_complete())
}

/// A SKIP that names an anchor the stream ALREADY COMPLETED is a diagnostic, and the
/// COMPLETE wins.
///
/// The capture child publishes a node's final record and THEN bumps its accounting
/// word. Those are two statements, so a kill between them leaves a complete anchor in
/// the ring while the parent's post-mortem still believes that node uncovered — and
/// the parent duly pushes a SKIP for the same `(run, step, node)`. A reader that
/// emitted both let a restore apply state and then be told the very same anchor was
/// void.
///
/// No parent-side ordering fixes that (reversing the two trades a loud contradiction
/// for a SILENT ABSENCE, and the parent cannot read its own ring — it is the
/// producer), so the resolution is a READER rule: the Complete is data the writer
/// really published; the Skip is the parent's guess about accounting it could not
/// observe.
///
/// Both halves of the stream are written by PRODUCTION code — the anchor through
/// `ForkSetEncoder` (the encoder `child_main` runs) and the skip through
/// `push_skip` with the cause and detail the post-mortem uses — so this is the real
/// byte stream, not a hand-built imitation of one.
#[test]
fn a_skip_naming_an_already_completed_anchor_is_a_note_and_the_anchor_wins() {
    use cerulion_core::state::SkipCause;
    use cerulion_core::state_carrier::{ForkSetEncoder, NodeEncoder};
    use cerulion_core::state_ring::{StateAnchorEvent, StateRingOwner};
    use std::sync::Arc;

    const STEP: u64 = 41;
    const RUN: u64 = 0x7E3;

    let tag = format!("prec_{}", std::process::id());
    let mut owner = StateRingOwner::create(&tag, 64, 0, RUN, &["n0"]).expect("ring");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let crumb = Arc::new(MappedBreadcrumb::create().expect("breadcrumb"));

    let oracle_payload = vec![0xC0u8; 300];
    {
        let targets = [FixedTarget {
            node_idx: 0,
            payload: oracle_payload.clone(),
        }];
        let mut enc = ForkSetEncoder::new(&targets, &mut producer, STEP);
        // The child's real output for this node: header, body, final record.
        enc.encode(0, &crumb);
    }
    // ...and then the child dies before its accounting word moves. The parent's
    // post-mortem reads `nodes_accounted` and skips what it believes uncovered —
    // this exact call, with this exact cause and detail.
    producer.push_skip(
        STEP,
        0,
        SkipCause::ChildCrashed,
        "the capture child ended before this node was encoded",
    );

    let (events, contradictions) = assemble_ring(&name);

    // THE ANCHOR IS APPLIED, with its bytes intact.
    let complete: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Complete {
                step,
                node_idx,
                bytes,
                ..
            } => Some((*step, *node_idx, bytes.clone())),
            _ => None,
        })
        .collect();
    assert_eq!(complete.len(), 1, "exactly one anchor: {events:?}");
    assert_eq!(complete[0].0, STEP);
    assert_eq!(complete[0].1, 0);
    assert!(
        complete[0].2.ends_with(&oracle_payload),
        "the anchor must carry the payload the encoder wrote, after its framing"
    );

    // THE SKIP IS A NOTE, NOT AN OUTCOME.
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, StateAnchorEvent::Skipped { .. })),
        "a skip naming an anchor this stream completed must NOT be reported as a \
         refusal — that is what lets a restore apply state and then be told it was \
         void: {events:?}"
    );
    let notes: Vec<_> = events
        .iter()
        .filter(|e| matches!(e, StateAnchorEvent::SkipAfterComplete { .. }))
        .collect();
    assert_eq!(notes.len(), 1, "the contradiction is REPORTED: {events:?}");
    assert!(
        matches!(
            notes[0],
            StateAnchorEvent::SkipAfterComplete {
                step: STEP,
                node_idx: 0,
                cause: SkipCause::ChildCrashed,
                ..
            }
        ),
        "and it keeps the writer's own key and cause: {:?}",
        notes[0]
    );
    assert_eq!(
        contradictions, 1,
        "and it is COUNTED, so a bag can say it happened without anyone reading logs"
    );
    drop(owner);
}

/// The rule must not swallow a REAL refusal.
///
/// Precedence is scoped to one `(run, step, node)`: a skip for a step the node never
/// completed, and a skip for a node with no anchor at all, are ordinary voided anchors
/// and must still read as `Skipped`. Without this arm the rule could be "suppress every
/// skip that follows any complete", which would hide exactly the refusals a skip record exists
/// to surface.
#[test]
fn a_skip_for_a_step_or_node_with_no_completed_anchor_is_still_a_refusal() {
    use cerulion_core::state::SkipCause;
    use cerulion_core::state_carrier::{ForkSetEncoder, NodeEncoder};
    use cerulion_core::state_ring::{StateAnchorEvent, StateRingOwner};
    use std::sync::Arc;

    const RUN: u64 = 0x7E4;

    let tag = format!("prec2_{}", std::process::id());
    let mut owner = StateRingOwner::create(&tag, 64, 0, RUN, &["n0", "n1"]).expect("ring");
    let name = owner.name().to_string();
    let mut producer = owner.producer().expect("producer");
    let crumb = Arc::new(MappedBreadcrumb::create().expect("breadcrumb"));

    // Node 0 completes at step 10 — the only completion in this stream.
    {
        let targets = [FixedTarget {
            node_idx: 0,
            payload: vec![1u8; 64],
        }];
        let mut enc = ForkSetEncoder::new(&targets, &mut producer, 10);
        enc.encode(0, &crumb);
    }
    // A LATER step for the same node: a real refusal.
    producer.push_skip(11, 0, SkipCause::Contended, "later step");
    // An EARLIER step for the same node: also a real refusal.
    producer.push_skip(9, 0, SkipCause::Contended, "earlier step");
    // A different node entirely, at the completed step: still a real refusal.
    producer.push_skip(10, 1, SkipCause::RecorderBehind, "other node");

    let (events, contradictions) = assemble_ring(&name);

    let skipped: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Skipped { step, node_idx, .. } => Some((*step, *node_idx)),
            _ => None,
        })
        .collect();
    assert_eq!(
        skipped,
        vec![(11, 0), (9, 0), (10, 1)],
        "every skip that names something this stream did NOT complete stays a refusal: \
         {events:?}"
    );
    assert_eq!(
        contradictions, 0,
        "and none of them is a contradiction — the rule is scoped to one (step, node)"
    );
    drop(owner);
}

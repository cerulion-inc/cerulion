// SPDX-License-Identifier: AGPL-3.0-only
//! The checkpoint ARM WORD over REAL POSIX SHM.
//!
//! The PURE halves (the cadence decision at both sides of every boundary, the claim
//! table, the stale-claim sweep against an INJECTED liveness predicate) are
//! in-module unit tests in `state_arm.rs`; this file is the behavioural half — the
//! seams a pure test structurally cannot see:
//!
//! * a recorder ARMING the word and a graph process reading `due()` through its OWN
//!   mapping, which is the whole mechanism (`bag record --run` is a MID-RUN
//!   attach with no spawn to configure);
//! * an unowned open REFUSING a segment that is missing, too small, or not a state
//!   arm word — so a graph process can never read a half-initialised or foreign word
//!   as an arm;
//! * the owner's `shm_unlink` removing the NAME only, so a peer that already mapped
//!   the word keeps it (the [`cerulion_core::barrier`] contract, inherited);
//! * a real `fork(2)` child's claim being visible to the parent — the `MAP_SHARED`
//!   property the fork gate's reservation words rest on, which a thread cannot model;
//! * the mapping being its OWN region of a known size, which is what the
//!   `MADV_DONTFORK` exclusion needs (one call, no collateral).
//!
//! Every wait is bounded by a GENEROUS liveness deadline. Per-test tags (name + pid)
//! ⇒ parallel-safe; no `#[serial]`, no iceoryx2.

#![cfg(unix)]

use std::time::{Duration, Instant};

use cerulion_core::state_arm::{
    state_arm_shm_name, MappedStateArm, STATE_ARM_BYTES, STATE_CLAIM_SLOTS,
};

/// Unique tag per test (name + pid) so parallel tests / re-runs never collide.
fn tag(name: &str) -> String {
    format!("arm_{name}_{}", std::process::id())
}

/// GENEROUS liveness ceiling. Never a wall stated in units of the thing under test.
const DEADLINE: Duration = Duration::from_secs(30);

/// BOUNDED `waitpid`, so a wedged child is an attributable red rather than a hang.
fn reap_bounded(pid: libc::pid_t) -> libc::c_int {
    let start = Instant::now();
    loop {
        let mut status: libc::c_int = 0;
        // SAFETY: FFI waitpid on a child this test forked; WNOHANG never blocks.
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(
            r >= 0,
            "waitpid({pid}): {}",
            std::io::Error::last_os_error()
        );
        if start.elapsed() >= DEADLINE {
            // SAFETY: kill + blocking reap of our own child; no orphan is left.
            unsafe {
                libc::kill(pid, libc::SIGKILL);
                let mut s: libc::c_int = 0;
                libc::waitpid(pid, &mut s, 0);
            }
            panic!("the fork child did not exit within {DEADLINE:?} — it was killed and reaped");
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

// ===========================================================================
// 1. The mechanism: a recorder arms, a graph process reads
// ===========================================================================

/// THE arm mechanism, end to end across two mappings of one page: the recorder
/// (owner) arms a cadence, and the graph process (peer) — which has its OWN mapping
/// and never saw the arm call — answers `due()` for exactly the steps the cadence
/// names, then goes quiet again when the recorder detaches.
///
/// The peer is opened BEFORE the arm, so the assertion covers the publish edge (arm
/// stores the cadence values, THEN the armed flag with `Release`) rather than a peer
/// that happened to map an already-armed word.
#[test]
fn a_recorder_arms_the_word_and_a_graph_process_sees_it_through_its_own_mapping() {
    let t = tag("arm_across");
    let owner = MappedStateArm::create_owned(&t).expect("create");
    let peer = MappedStateArm::open_unowned(&t).expect("open");
    assert_eq!(owner.name(), state_arm_shm_name(&t));
    assert_eq!(peer.name(), owner.name());

    // Detached: the boundary check is a null test and nothing is due, ever.
    assert!(!peer.is_armed());
    for step in [0u64, 1, 1_000, u64::MAX] {
        assert!(
            !peer.due(step),
            "a detached word anchors nothing (step {step})"
        );
    }

    owner.arm(30_000, 1_000);
    assert!(
        peer.is_armed(),
        "the peer sees the arm through the shared page"
    );
    assert_eq!(peer.cadence_steps(), 30_000);
    assert_eq!(peer.first_anchor_step(), 1_000);
    assert!(!peer.due(999));
    assert!(peer.due(1_000), "the attach's own boundary is due");
    assert!(!peer.due(1_001));
    assert!(peer.due(31_000));

    owner.disarm();
    assert!(!peer.is_armed());
    assert!(!peer.due(31_000), "a detached word anchors nothing");
}

/// A ONE-SHOT arm — `cadence_steps == 0` — is the mid-run attach's own shape
/// (an immediate checkpoint at the next boundary), and the reason periodic anchors
/// are OFF by default for `graph run --record`. It must fire once and never again.
#[test]
fn a_one_shot_arm_fires_at_its_step_and_never_again() {
    let t = tag("oneshot");
    let owner = MappedStateArm::create_owned(&t).expect("create");
    let peer = MappedStateArm::open_unowned(&t).expect("open");
    owner.arm(0, 55);
    assert!(peer.due(55));
    for step in [0u64, 54, 56, 110, 1_000_000] {
        assert!(!peer.due(step), "one-shot must not repeat (step {step})");
    }
}

// ===========================================================================
// 2. An opener never reads a word it cannot trust
// ===========================================================================

#[test]
fn open_unowned_refuses_a_segment_that_was_never_created() {
    let err = MappedStateArm::open_unowned(&tag("missing")).expect_err("must not create silently");
    assert_eq!(
        err.kind(),
        std::io::ErrorKind::NotFound,
        "a missing arm word is ENOENT, never a silently-created word nobody armed: {err}"
    );
}

/// A segment of the right NAME that is not a state arm word — the shape a stale
/// object of another kind, or a creator that died mid-`init`, presents — is refused
/// on its magic rather than read as a detached-but-valid word.
///
/// This matters because a zero-filled region decodes as "detached, cadence 0, empty
/// table", which is a perfectly plausible-looking word: without the magic check an
/// opener could not tell "nobody has armed yet" from "this is not an arm word".
#[test]
fn open_unowned_refuses_a_segment_whose_magic_is_not_set() {
    let name = state_arm_shm_name(&tag("nomagic"));
    let cname = std::ffi::CString::new(name.clone()).unwrap();
    // SAFETY: FFI create of a name this test derived; unlinked below.
    let fd = unsafe {
        libc::shm_open(
            cname.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
            0o600 as libc::c_uint,
        )
    };
    assert!(fd >= 0, "shm_open: {}", std::io::Error::last_os_error());
    // SAFETY: size the fresh object; zero-filled by POSIX.
    assert_eq!(
        unsafe { libc::ftruncate(fd, STATE_ARM_BYTES as libc::off_t) },
        0
    );
    // SAFETY: the mapping is not needed; close our descriptor.
    unsafe { libc::close(fd) };

    let err = MappedStateArm::open_unowned(&tag("nomagic")).expect_err("magic is unset");
    assert!(
        err.to_string().contains("magic"),
        "the refusal must name the magic: {err}"
    );
    // SAFETY: clean up the object this test created.
    unsafe { libc::shm_unlink(cname.as_ptr()) };
}

/// A segment SHORTER than the word is refused before it is mapped — otherwise the
/// claim table would be indexed past the object and every access would be a
/// SIGBUS-on-touch rather than an error.
///
/// SCOPE, because the harness cannot reach that guard on every platform:
/// macOS rounds a POSIX SHM object's size UP to a page, so `ftruncate(fd, 1)` here
/// yields an object of at least [`STATE_ARM_BYTES`] and the refusal comes from the
/// MAGIC instead (MEASURED on macOS: a test asserting
/// the size wording fails with the magic wording). Linux does not round, so
/// there the size arm is the one that fires. The assertion is therefore on the
/// property that holds everywhere — an untrustworthy segment is REFUSED, never
/// mapped and read — and it names which guard caught it rather than pretending only
/// one can.
#[test]
fn open_unowned_refuses_a_segment_too_short_to_hold_the_word() {
    let name = state_arm_shm_name(&tag("short"));
    let cname = std::ffi::CString::new(name).unwrap();
    // SAFETY: FFI create of a name this test derived; unlinked below.
    let fd = unsafe {
        libc::shm_open(
            cname.as_ptr(),
            libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
            0o600 as libc::c_uint,
        )
    };
    assert!(fd >= 0, "shm_open: {}", std::io::Error::last_os_error());
    // SAFETY: size it to one byte — far short of the word.
    assert_eq!(unsafe { libc::ftruncate(fd, 1) }, 0);
    // SAFETY: close our descriptor.
    unsafe { libc::close(fd) };

    let err = MappedStateArm::open_unowned(&tag("short")).expect_err("too short");
    let msg = err.to_string();
    assert!(
        msg.contains("short of") || msg.contains("magic"),
        "an untrustworthy segment must be refused by the SIZE guard (Linux) or the \
         MAGIC guard (macOS, which rounds the object up to a page) — never mapped \
         and read: {msg}"
    );
    // SAFETY: clean up the object this test created.
    unsafe { libc::shm_unlink(cname.as_ptr()) };
}

/// The owner's `Drop` removes the NAME, not the object: a peer that already mapped
/// the word keeps a live, usable mapping, while a FRESH open fails.
///
/// Inherited from [`cerulion_core::barrier`]'s contract, and load-bearing for the
/// same reason: a recorder detaching mid-run must not invalidate a graph process's
/// mapping under it.
#[test]
fn the_owner_unlinks_the_name_on_drop_but_a_live_peer_mapping_survives() {
    let t = tag("unlink");
    let owner = MappedStateArm::create_owned(&t).expect("create");
    let peer = MappedStateArm::open_unowned(&t).expect("open");
    owner.arm(10, 0);
    drop(owner);

    assert!(
        MappedStateArm::open_unowned(&t).is_err(),
        "a FRESH open must fail once the name is gone"
    );
    // The already-mapped peer is untouched.
    assert!(peer.is_armed());
    assert!(peer.due(20));
    let slot = peer.claim(std::process::id() as i32, 1_234).expect("slot");
    assert_eq!(peer.busy_workers(), 1);
    assert_eq!(peer.release(slot, std::process::id() as i32), Some(1_234));
}

// ===========================================================================
// 3. The reservation words really are SHARED
// ===========================================================================

/// A claim taken through one mapping is visible through another, with its reserved
/// bytes — the property the fork memory gate rests on. Uncoordinated per-process
/// `MemAvailable` reads all pass at once because they are taken simultaneously; a
/// SHARED reservation word is what makes the gates observe each other, and this is
/// the proof it is shared rather than per-mapping.
#[test]
fn a_claim_taken_in_one_mapping_is_visible_and_releasable_in_another() {
    let t = tag("claims");
    let owner = MappedStateArm::create_owned(&t).expect("create");
    let peer = MappedStateArm::open_unowned(&t).expect("open");

    let a = owner.claim(1001, 700_000_000).expect("slot");
    assert_eq!(peer.busy_workers(), 1, "the gate's one relaxed load");
    assert_eq!(peer.rss_reserved(), 700_000_000);
    assert_eq!(peer.claim_pid(a), Some(1001));

    let b = peer.claim(1002, 300_000_000).expect("slot");
    assert_ne!(a, b);
    assert_eq!(owner.busy_workers(), 2);
    assert_eq!(owner.rss_reserved(), 1_000_000_000);

    // A release through EITHER mapping frees the same slot.
    assert_eq!(
        peer.release(a, std::process::id() as i32),
        Some(700_000_000)
    );
    assert_eq!(owner.busy_workers(), 1);
    assert_eq!(owner.rss_reserved(), 300_000_000);
    assert_eq!(
        owner.release(b, std::process::id() as i32),
        Some(300_000_000)
    );
    assert_eq!(peer.busy_workers(), 0);
    assert_eq!(peer.rss_reserved(), 0);
    assert_eq!(peer.slot_count(), STATE_CLAIM_SLOTS);
}

/// A real `fork(2)` child's claim is visible to the parent — the shape a capture fork
/// produces (the parent claims, forks, and reaps; the stale-claim sweep then has to
/// tell a live claimant from a dead one).
///
/// A thread cannot model this: the point is that the page is `MAP_SHARED` and
/// therefore NOT copy-on-write, so the child's atomic write lands in the parent's
/// view. The child's body is two atomic ops and `_exit` — no allocation, no lock, no
/// destructor — and `_exit` (not `exit`) runs no `atexit` handler and no `Drop`, so
/// the child never `shm_unlink`s the parent's word.
///
/// # This is also the arm that answers "the claim tests use invented pids"
///
/// The in-module claim-table oracles — hand-chosen pids, hand-chosen
/// reservation sizes, an injected liveness predicate — can read as Principle #13 fake data.
/// They are the opposite on both halves. Hand-built oracle VALUES are the pattern this
/// repo mandates (`shm_ring_test`'s "round-trip vs a hand-built record vector"; the
/// testing section's "compare against an oracle vector"), and Principle #13 is about
/// never fabricating data that STANDS IN for a real measurement — a benchmark number,
/// a robot's frames — which no test input does. The injected predicate is not a
/// simulation of `kill(2)` either: it is what keeps a syscall out of a decision that
/// gets mutated, per the repo's own mutate-PURE-decision-fns-only rule, and the rule
/// under test ("which claims are stale, and who wins a race with a release") is
/// independent of any pid's numeric value.
///
/// The one thing an invented pid genuinely cannot show is that the rule works against
/// a process that REALLY died — so THIS arm does that: a real `fork(2)` child claims
/// with its own `getpid()`, is reaped, and is then swept as stale. Testing only that
/// way would be strictly worse (pid reuse makes it flaky, and no adversarial
/// interleaving can be enumerated against a live process), which is why both exist.
#[test]
fn a_forked_childs_claim_lands_in_the_parents_view_of_the_word() {
    let t = tag("fork_claim");
    let arm = MappedStateArm::create_owned(&t).expect("create");
    let parent_slot = arm.claim(std::process::id() as i32, 111).expect("slot");
    assert_eq!(arm.busy_workers(), 1);

    // The child OPENS THE WORD BY NAME rather than using an inherited mapping, and
    // that is a requirement rather than a style choice: the `StateArmWord` is
    // excluded from `fork` inheritance at BIRTH, so a
    // capture child's accidental touch of the live claim table FAULTS in a disposable
    // process instead of corrupting the word every worker's fork gate reads. Touching
    // the inherited address here SIGSEGVs.
    //
    // Opening by name is also what a real peer does: workers are spawned `fork`+`exec`,
    // which replaces the image, so every production peer maps the word
    // afresh from its name. The properties this test pins are untouched — the page is
    // still MAP_SHARED, the child's claim still lands in the parent's view, and the
    // dead-pid sweep still composes.
    //
    // SAFETY: the child branch below maps the word by name, does two atomic ops and
    // `_exit`s.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork: {}", std::io::Error::last_os_error());
    if pid == 0 {
        // SAFETY: getpid is async-signal-safe; the claim is two atomic ops on the
        // child's OWN MAP_SHARED mapping of the same object.
        let mypid = unsafe { libc::getpid() };
        let ok = match MappedStateArm::open_unowned(&t) {
            Ok(peer) => {
                let claimed = peer.claim(mypid, 222).is_some();
                // Leak the peer mapping: its `Drop` would `munmap` in a process that is
                // about to `_exit` anyway, and running destructors here is exactly what
                // the discipline forbids.
                std::mem::forget(peer);
                claimed
            }
            Err(_) => false,
        };
        // SAFETY: leave immediately — no unwinding into the parent's frames, no
        // `Drop` for the inherited owner (which would unlink the parent's word).
        unsafe { libc::_exit(if ok { 0 } else { 1 }) };
    }

    let status = reap_bounded(pid);
    assert!(
        libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
        "the fork child must claim and exit 0 (status {status})"
    );

    assert_eq!(
        arm.busy_workers(),
        2,
        "the child's claim landed in the parent's view — the page is MAP_SHARED, \
         not copy-on-write"
    );
    assert_eq!(arm.rss_reserved(), 333);

    // AMENDMENT 7 composed with a REAL dead pid: the child is reaped, so its claim is
    // stale and must be cleared — otherwise every survivor's cadence freezes as
    // `StillEncoding` forever under `--peer-loss continue`.
    let me = std::process::id() as i32;
    let sweep = arm.sweep_stale_claims(|claimant| claimant == me);
    assert_eq!(sweep.cleared, 1, "the reaped child's claim is stale");
    assert_eq!(sweep.freed_bytes, 222);
    assert_eq!(sweep.live, 1);
    assert!(!sweep.is_empty(), "the caller logs this LOUDLY");
    assert_eq!(arm.busy_workers(), 1, "counters re-derived from the table");
    assert_eq!(arm.rss_reserved(), 111);
    assert_eq!(arm.claim_pid(parent_slot), Some(me), "ours survived");
}

// ===========================================================================
// 4. The MADV_DONTFORK constraint
// ===========================================================================

/// The mapping is its OWN page-aligned region of exactly [`STATE_ARM_BYTES`], holding
/// nothing else.
///
/// That is the LAYOUT half of the `MADV_DONTFORK` constraint, and it is the arm's to hold: the
/// carrier's `MADV_DONTFORK` sweep (Linux-only) is then one
/// `madvise(ptr, len, MADV_DONTFORK)` that covers the arm word exactly and no other
/// object. If the word ever shared a mapping with something else, that sweep would
/// either miss it or take the neighbour with it.
#[test]
fn the_mapping_is_exactly_what_the_madv_dontfork_sweep_must_cover() {
    let arm = MappedStateArm::create_owned(&tag("dontfork")).expect("create");
    let (ptr, len) = arm.mapping();
    assert_eq!(len, STATE_ARM_BYTES);
    assert!(!ptr.is_null());
    // SAFETY: reading the platform page size; no memory is touched.
    let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as usize;
    assert!(page > 0);
    assert_eq!(
        (ptr as usize) % page,
        0,
        "mmap returns page-aligned memory, which is the granularity madvise works at"
    );
    // The word is genuinely usable through that pointer (the region is not a stub).
    arm.arm(1, 0);
    assert!(arm.due(0));
}

// ===========================================================================
// 5. The publication edge: ONE read path, structurally
// ===========================================================================

/// Strip Rust comments so a walk cannot be satisfied — or defeated — by prose.
///
/// Block comments NEST in Rust, so the depth is tracked; an unterminated block fails
/// CLOSED (everything after it is treated as comment) rather than leaving a partial
/// view that would make every negative assertion vacuous over a prefix. String
/// literals are deliberately NOT modelled: the module under test names these field
/// accesses in its own DOC COMMENTS — which is exactly what this strips — and holds
/// no string literal containing them.
fn code_only(src: &str) -> String {
    let b: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth, mut line_comment) = (0usize, 0usize, false);
    while i < b.len() {
        if line_comment {
            if b[i] == '\n' {
                line_comment = false;
                out.push('\n');
            }
            i += 1;
            continue;
        }
        if depth > 0 {
            if b[i] == '*' && i + 1 < b.len() && b[i + 1] == '/' {
                depth -= 1;
                i += 2;
                continue;
            }
            if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
                depth += 1;
                i += 2;
                continue;
            }
            if b[i] == '\n' {
                out.push('\n');
            }
            i += 1;
            continue;
        }
        if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '/' {
            line_comment = true;
            i += 2;
            continue;
        }
        if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
            depth += 1;
            i += 2;
            continue;
        }
        out.push(b[i]);
        i += 1;
    }
    out
}

fn state_arm_source() -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/state_arm.rs");
    code_only(&std::fs::read_to_string(&path).expect("read state_arm.rs"))
}

/// The body of `fn <name>`, brace-matched over the comment-stripped source.
///
/// A whole-file `find` cannot express "THIS function does X" — it is satisfied the
/// moment ANY function does — and the two rules below are both per-function. Scope
/// stated because it is not general: string literals are not modelled, so this is used
/// only on bodies that contain none (asserted by each caller's marker check).
fn fn_body<'a>(src: &'a str, name: &str) -> &'a str {
    let at = src
        .find(name)
        .unwrap_or_else(|| panic!("`{name}` must exist in the stripped source"));
    let open = src[at..]
        .find('{')
        .unwrap_or_else(|| panic!("`{name}` must have a body"))
        + at;
    let mut depth = 0usize;
    for (i, c) in src[open..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[open..open + i + 1];
                }
            }
            _ => {}
        }
    }
    panic!("`{name}`'s body is unterminated in the stripped source");
}

/// THE structural guard for the slot protocol's two ordering rules.
///
/// A behavioural test CANNOT pin either. A missing happens-before edge is not an
/// observable event — it is the ABSENCE of a guarantee — and on x86's TSO the ordered
/// and unordered forms are indistinguishable at runtime, so a green test here would
/// prove nothing about the aarch64 robots this ships to. The ABA rule is worse: its
/// window is a few instructions inside a function a test cannot preempt. What IS
/// checkable is the SHAPE of the code, which is what this asserts.
///
/// **Rule 1 — identity is not a separate field.** The state tag and the holder's pid
/// ride ONE atomic, so "read the pid before its publication edge" is not a mistake
/// this module can make: there is no `pid` field to read. That is why this walk
/// does not count `.pid.load(` sites and check each against its own
/// `Acquire` load of `state`: the ordering question is removed by the
/// protocol, not patched in the test. `reserved_bytes` IS still a separate word, so
/// its one read must follow the `owner` `Acquire` load, and `claim_view` being the
/// sole read path is what makes that checkable at all.
///
/// **Rule 2 — the sweep exchanges on the word it OBSERVED.** Re-loading before the
/// compare-exchange makes the exchange succeed against a DIFFERENT claim that recycled
/// the slot, tearing down a live peer's reservation. Exactly one load in the body is
/// how that is enforced structurally.
#[test]
fn the_slot_protocol_keeps_identity_in_the_ownership_word_and_exchanges_on_it() {
    let src = state_arm_source();

    // RULE 1a: no separate identity field, in either direction.
    for probe in [".pid.load(", ".pid.store(", ".state.load(", ".state.store("] {
        let n = src.matches(probe).count();
        assert_eq!(
            n, 0,
            "found {n} `{probe}` site(s) on a claim slot. A slot's lifecycle tag and \
             its holder's pid must stay in the ONE `owner` atomic: split across two \
             words, a process killed between them leaves a held slot nobody can name, \
             which no sweep can liveness-test and therefore no sweep can reclaim — a \
             finite table, so it ends as a permanent leak."
        );
    }

    // RULE 1b: `reserved_bytes` is read in exactly one place, and after the edge.
    let bytes_reads = src.matches(".reserved_bytes.load(").count();
    assert_eq!(
        bytes_reads, 1,
        "expected EXACTLY 1 read of a slot's `reserved_bytes` (in `claim_view`, the \
         one public read path) but found {bytes_reads}. A new read must take the \
         publication edge FIRST: route it through `claim_view`, or Acquire-load \
         `owner` and read the bytes only after observing a state that published \
         them. The write is Relaxed and is published by the Release exchange of \
         `owner`, so an Acquire load of `reserved_bytes` ITSELF pairs with nothing \
         and orders nothing. (`finish_release` uses `.swap(..)` under the ownership \
         exchange, which is a write, not a read.)"
    );

    // Every ordered read of the ownership word really is Acquire — a Relaxed one
    // re-opens the byte hazard rule 1b closes.
    let owner_loads = src.matches(".owner.load(").count();
    let acquire_loads = src.matches(".owner.load(Ordering::Acquire)").count();
    assert_eq!(
        owner_loads, acquire_loads,
        "{owner_loads} `.owner.load(` site(s) but only {acquire_loads} of them are \
         `Ordering::Acquire`. The owner load is the publication edge for \
         `reserved_bytes`; a Relaxed one orders nothing."
    );
    assert!(
        owner_loads >= 3,
        "expected at least the three ordered readers (`claim_view`, `release`, \
         `sweep_inner`) but found {owner_loads} — the walk is measuring nothing"
    );

    // RULE 1b, ORDER: the one read path takes the edge FIRST. Asserted inside
    // `claim_view`'s own body, so a same-named load elsewhere cannot satisfy it.
    let view = fn_body(&src, "pub fn claim_view");
    let owner_at = view
        .find("s.owner.load(Ordering::Acquire)")
        .expect("claim_view must Acquire-load the ownership word");
    let bytes_at = view
        .find("s.reserved_bytes.load(")
        .expect("claim_view must read the reserved bytes");
    assert!(
        owner_at < bytes_at,
        "the Acquire load of `owner` must come BEFORE the `reserved_bytes` read — it \
         is what makes that read ordered at all"
    );

    // RULE 2: one observation, and the exchange uses it.
    let sweep = fn_body(&src, "fn sweep_inner");
    let sweep_loads = sweep.matches(".owner.load(").count();
    assert_eq!(
        sweep_loads, 1,
        "`sweep_inner` must load a slot's ownership word EXACTLY once per slot and \
         compare-exchange on THAT value, but found {sweep_loads} load site(s). A \
         re-load between the judgement and the exchange lets the exchange succeed \
         against a claim that took the slot in between — the sweep then tears down a \
         LIVE peer's reservation on a stale liveness verdict."
    );
    assert!(
        sweep.contains("compare_exchange(\n                    owner,")
            || sweep.contains("compare_exchange(owner,"),
        "`sweep_inner`'s ownership exchange must compare against the `owner` word it \
         observed, not a freshly-read or hand-built one — that identity check IS the \
         ABA defence"
    );

    // ANTI-TAUTOLOGY: the stripped view (and the extractor) must still contain the
    // code being asserted over. A stripper that ate everything, or an extractor that
    // returned an empty body, would make every count above read 0 and pass.
    assert!(
        src.contains("pub fn claim_view") && src.contains("fn sweep_inner"),
        "the comment-stripped view lost the code under test — every assertion above \
         would be vacuous"
    );
    assert!(
        view.contains("ClaimView") && sweep.contains("StaleClaimSweep"),
        "the function-body extractor returned something that is not the body under \
         test — the per-function assertions above would be vacuous"
    );
    assert!(
        !view.contains("fn sweep_inner") && !sweep.contains("pub fn claim_view"),
        "the extractor ran past the end of its function, so the per-function rules \
         above are really whole-file rules"
    );
}

/// The anti-tautology twin for the walk above: the stripper really strips, and really
/// keeps code. A `code_only` returning the input UNCHANGED would let a commented-out
/// mention inflate a count silently, which is the failure mode that matters here.
#[test]
fn the_source_stripper_removes_comments_and_keeps_code() {
    assert_eq!(code_only("let a = 1; // x.pid.load(\n"), "let a = 1; \n");
    assert_eq!(code_only("a/* x.pid.load( */b"), "ab");
    assert_eq!(
        code_only("a/* /* nested */ */b"),
        "ab",
        "blocks NEST in Rust"
    );
    assert_eq!(
        code_only("a// /* not a block\nb"),
        "a\nb",
        "a block opener inside a line comment is not an opener"
    );
    assert_eq!(
        code_only("a/* unterminated b"),
        "a",
        "an unterminated block fails CLOSED"
    );
    assert_eq!(code_only("keep.me()"), "keep.me()");
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Audit **amendment 10**: an excluded mapping is genuinely ABSENT
//! from a `fork` child, driven over a real child on BOTH shipping platforms.
//!
//! # Why a behavioural arm, and why it is portable
//!
//! The in-module oracles cover the tally and the return values. None of them can see
//! the property the control exists for — that the child cannot READ the region — and
//! that is the whole point: an encoder's accidental read of a live `MAP_SHARED`
//! iceoryx2 segment must be a SIGSEGV in a disposable child rather than torn bytes
//! written into the bag as a point-in-time image.
//!
//! macOS was originally thought to simply lose this control, which would have made
//! the arm Linux-only and left the dev platform weaker than the robot in a way a node
//! author discovers in production. The audit's fact-check addendum verified
//! `minherit(2)`/`VM_INHERIT_NONE` instead, so the SAME arm runs on both: Linux via
//! `MADV_DONTFORK`, macOS via `minherit`. The constants were re-measured first-party
//! for this chunk (`VM_INHERIT_NONE == 2`; `int minherit(void *, size_t, int)`).
//!
//! # The control is in the same body, and it is what makes the arm mean anything
//!
//! Two identical `MAP_SHARED` pages, written with the same sentinel, differing only in
//! whether they were excluded. The child reads BOTH and reports through a pipe. If the
//! excluded page were still inherited the child would report its sentinel; if the
//! control page were NOT inherited, the test would be proving something about `mmap`
//! rather than about the exclusion. Both halves are asserted.
//!
//! The child reads the excluded page LAST and reports the control's answer FIRST, so
//! its report survives its own death: touching an excluded region is a fault, and the
//! child is expected to die on it.
//!
//! `#![cfg(unix)]`; parallel-safe (anonymous pages, own pipes, targeted reaps).

#![cfg(unix)]

use cerulion_core::state_carrier::{exclude_from_fork, ExclusionOutcome};

const PAGE: usize = 4096;
const SENTINEL: u64 = 0x0BAD_F00D_1053_C0DE;
const CEILING: std::time::Duration = std::time::Duration::from_secs(30);

fn map_shared_page() -> *mut std::ffi::c_void {
    let p = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            PAGE,
            libc::PROT_READ | libc::PROT_WRITE,
            // MAP_SHARED, because that is the class the control is FOR: a private
            // anonymous page is frozen by the fork anyway, so excluding one would
            // prove nothing about the iceoryx2 segments that matter here.
            libc::MAP_SHARED | libc::MAP_ANONYMOUS,
            -1,
            0,
        )
    };
    assert_ne!(p, libc::MAP_FAILED, "mmap failed");
    unsafe { (p as *mut u64).write(SENTINEL) };
    p
}

fn reap(pid: libc::pid_t) -> libc::c_int {
    let deadline = std::time::Instant::now() + CEILING;
    loop {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(r >= 0, "waitpid: {}", std::io::Error::last_os_error());
        if std::time::Instant::now() >= deadline {
            unsafe { libc::kill(pid, libc::SIGKILL) };
            let mut st: libc::c_int = 0;
            unsafe { libc::waitpid(pid, &mut st, 0) };
            panic!("child did not exit within {CEILING:?}");
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

#[test]
fn amendment_10_an_excluded_mapping_is_absent_from_the_child_while_its_twin_is_not() {
    let control = map_shared_page();
    let excluded = map_shared_page();

    // SAFETY: `excluded` is a live mapping this process owns.
    let outcome = unsafe { exclude_from_fork(excluded, PAGE) };
    assert_eq!(
        outcome,
        ExclusionOutcome::Excluded,
        "both shipping platforms have a primitive for this: MADV_DONTFORK on Linux, \
         minherit(VM_INHERIT_NONE) on macOS. An Unsupported here means the dev platform \
         is silently weaker than the robot."
    );

    let mut fds = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
    let (read_fd, write_fd) = (fds[0], fds[1]);

    // SAFETY: the child does two volatile reads and one `write(2)`, then `_exit`s. It is
    // EXPECTED to die on the second read; the first is already reported by then.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        // The CONTROL first, and reported before the fault can happen.
        let seen_control = unsafe { (control as *const u64).read_volatile() };
        let ok = [u8::from(seen_control == SENTINEL)];
        unsafe { libc::write(write_fd, ok.as_ptr() as *const libc::c_void, 1) };

        // Now the excluded page. On both platforms this address is not mapped in this
        // process, so this is a fault and the `_exit` below is unreachable — which is
        // the property under test.
        let seen_excluded = unsafe { (excluded as *const u64).read_volatile() };
        let leaked = [b'L', u8::from(seen_excluded == SENTINEL)];
        unsafe { libc::write(write_fd, leaked.as_ptr() as *const libc::c_void, 2) };
        unsafe { libc::_exit(0) };
    }

    unsafe { libc::close(write_fd) };
    let status = reap(pid);

    // Drain whatever the child managed to report.
    let mut buf = [0u8; 8];
    let n = unsafe { libc::read(read_fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    unsafe { libc::close(read_fd) };
    unsafe {
        libc::munmap(control, PAGE);
        libc::munmap(excluded, PAGE);
    }

    assert!(n >= 1, "the child reported nothing at all; it never ran");
    assert_eq!(
        buf[0], 1,
        "the CONTROL page must be inherited — without that this test proves something \
         about mmap rather than about the exclusion"
    );
    assert_eq!(
        n, 1,
        "the child read the EXCLUDED page successfully and reported it: the mapping was \
         inherited, so an encoder's stray read of a live iceoryx2 segment would be torn \
         bytes in the bag (§0.4) rather than a fault in a disposable child"
    );
    assert!(
        libc::WIFSIGNALED(status),
        "touching an excluded region must FAULT the child; it exited normally instead \
         (status {status:#x})"
    );
    let sig = libc::WTERMSIG(status);
    assert!(
        sig == libc::SIGSEGV || sig == libc::SIGBUS,
        "expected SIGSEGV/SIGBUS on the excluded region, got signal {sig}"
    );
}

#[test]
fn excluding_a_region_leaves_the_parents_own_access_untouched() {
    // The control is about INHERITANCE, not about protection: the parent must keep full
    // access to a region it excluded, or arming the sweep would break the very data
    // plane it is protecting. (Reading the sentinel back is the whole assertion; if the
    // exclusion revoked the parent's own mapping this would fault here.)
    let p = map_shared_page();
    // SAFETY: a live mapping this process owns.
    let outcome = unsafe { exclude_from_fork(p, PAGE) };
    assert_eq!(outcome, ExclusionOutcome::Excluded);

    let read_back = unsafe { (p as *const u64).read_volatile() };
    unsafe { (p as *mut u64).write(SENTINEL ^ 0xFFFF) };
    let rewritten = unsafe { (p as *const u64).read_volatile() };
    unsafe { libc::munmap(p, PAGE) };

    assert_eq!(
        read_back, SENTINEL,
        "the parent must still READ an excluded region"
    );
    assert_eq!(
        rewritten,
        SENTINEL ^ 0xFFFF,
        "the parent must still WRITE an excluded region"
    );
}

/// Probe a region in a real `fork` child and report whether the read SUCCEEDED.
///
/// Returns `(read_ok, exited_normally)`. An excluded region faults, so the child never
/// reaches its report — which is why the report is written BEFORE the fault can happen
/// and the exit status is returned alongside it.
fn child_can_read(region: *mut std::ffi::c_void) -> (bool, bool) {
    let mut fds = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
    let (read_fd, write_fd) = (fds[0], fds[1]);

    // SAFETY: the child does one volatile read, one `write(2)`, then `_exit`s. It is
    // EXPECTED to die on the read when the region is excluded.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        // The VALUE is irrelevant — reaching the report at all is the answer, since an
        // excluded region faults on the read above.
        let seen = unsafe { (region as *const u64).read_volatile() };
        let ok = [1u8, (seen & 0xff) as u8];
        unsafe { libc::write(write_fd, ok.as_ptr() as *const libc::c_void, 2) };
        unsafe { libc::_exit(0) };
    }
    unsafe { libc::close(write_fd) };
    let status = reap(pid);
    let mut buf = [0u8; 4];
    let n = unsafe { libc::read(read_fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    unsafe { libc::close(read_fd) };
    (n >= 1, libc::WIFEXITED(status))
}

#[test]
fn amendment_10_every_mapping_the_sweep_set_names_is_excluded_at_birth() {
    // The primitive's own arms prove it WORKS; this proves it is CALLED — the
    // no-inert-shipping rule, and the review's own finding (the primitive had no
    // production caller at all). Each mapping is built through its REAL production
    // constructor, so a constructor that stopped excluding fails here.
    //
    // The sweep set is amendment 10's table, and the COMPLEMENT is asserted in the same
    // body: the breadcrumb and the state ring are the child's only channels, so
    // excluding either would mean no anchor, or every child killed at the first stall
    // timeout. A test that only checked the excluded half would pass a sweep that
    // excluded everything.
    let tag = format!("a10_{}_{}", std::process::id(), line!());

    // --- the EXCLUDED set -------------------------------------------------------
    let trace = cerulion_core::trace_ring::TraceRingOwner::create(&tag, 8, 0, &["n0"])
        .expect("create trace ring");
    let (trace_ptr, _) = trace.mapping();

    let arm =
        cerulion_core::state_arm::MappedStateArm::create_owned(&tag).expect("create arm word");
    let (arm_ptr, _) = arm.mapping();

    let barrier = cerulion_core::barrier::MappedBarrier::create_owned(&tag, "a10", 1)
        .expect("create barrier");
    let (barrier_ptr, _) = barrier.mapping();

    // The cross-process `block`-edge CREDIT word. It belongs
    // in THIS body rather than in a sibling arm because the property is the
    // TABLE's, not any one mapping's: a case that lives elsewhere can be
    // deleted, or never written, without the inventory noticing. Its stakes are
    // the same class as the trace ring's and the direction is worse — the word
    // is the producer's live DEFER GATE, so a capture child that inherited it
    // and strayed would write into a page a running graph reads every tick
    // entry, and the cheapest wrong value (`outstanding` raised) wedges that
    // edge's producer rather than merely corrupting a record.
    let credit_id = cerulion_core::credit::credit_edge_id("/a10/scan", "planner", "scan_in");
    let credit = cerulion_core::credit::MappedCredit::create_owned(&tag, &credit_id, 1)
        .expect("create credit word");
    let credit_ptr = credit.addr() as *mut std::ffi::c_void;

    // The PEER mapping of that same word, opened exactly as a worker opens it.
    // It is a SEPARATE case rather than a duplicate: `open_unowned` mmaps the
    // shared object a SECOND time, at its own address, so it carries its OWN
    // `exclude_at_birth` call — the exclusion is per-MAPPING (`madvise` /
    // `minherit` take an address), not per shared object. Without a peer here
    // the owner's call alone satisfies the table and the worker's could be
    // deleted with this inventory still green, while on the shipping shape it
    // is the WORKER that forks: a supervisor creates the word and every
    // `graph run-worker` opens it, so an unexcluded peer mapping is exactly the
    // live defer gate a capture child would inherit.
    let credit_peer = cerulion_core::credit::MappedCredit::open_unowned(&tag, &credit_id)
        .expect("peer opens the credit word");
    let credit_peer_ptr = credit_peer.addr() as *mut std::ffi::c_void;
    assert_ne!(
        credit_ptr, credit_peer_ptr,
        "the peer must be a DISTINCT mapping of the same page, or it probes nothing new"
    );

    // The per-rank WEDGE page. Same reasoning as the credit
    // word above, and the same OWNER/PEER pair for the same per-MAPPING reason:
    // the supervisor `create_owned`s the page and the worker that rank belongs
    // to `open_unowned`s it, so the WORKER holds a second mapping of the same
    // object at its own address — and it is the worker that forks. The page is
    // the fire path's live in-tick seq pairs, written by every node entry and
    // exit, so a capture child that inherited it and strayed would write into
    // words a running graph's supervisor reads every alarm sweep.
    let wedge = cerulion_core::wedge_page::MappedWedgePage::create_owned(&tag, 0, 4)
        .expect("create wedge page");
    let (wedge_ptr, _) = wedge.mapping();

    let wedge_peer = cerulion_core::wedge_page::MappedWedgePage::open_unowned(&tag, 0)
        .expect("peer opens the wedge page");
    let (wedge_peer_ptr, _) = wedge_peer.mapping();
    assert_ne!(
        wedge_ptr, wedge_peer_ptr,
        "the peer must be a DISTINCT mapping of the same page, or it probes nothing new"
    );

    for (what, ptr) in [
        ("trace ring", trace_ptr),
        ("state arm word", arm_ptr),
        ("barrier page", barrier_ptr),
        ("credit page (owner mapping)", credit_ptr),
        ("credit page (peer mapping)", credit_peer_ptr),
        ("wedge page (owner mapping)", wedge_ptr),
        ("wedge page (peer mapping)", wedge_peer_ptr),
    ] {
        let (read_ok, exited) = child_can_read(ptr);
        assert!(
            !read_ok,
            "a capture child could READ the {what}: the mapping was inherited, so a \
             stray access is torn live bytes (§0.4) rather than a fault in a \
             disposable child — and for the trace ring that is pointed at the LIVE bag"
        );
        assert!(
            !exited,
            "the {what} probe must FAULT the child, not exit normally"
        );
    }

    // --- the COMPLEMENT: BOTH of the child's own channels stay inherited ---------
    //
    // The complement is a PAIR, not an example: exactly two mappings the
    // sweep must leave alone, and they fail differently: excluding the breadcrumb kills
    // every child at the first stall timeout, while excluding the STATE RING silently
    // costs every anchor its output — the child writes its records there, so a fault or
    // a private copy means the bag gets nothing and the reader reports PartialAnchor
    // for a capture that ran perfectly.
    let crumb = cerulion_core::state_carrier::MappedBreadcrumb::create().expect("breadcrumb");
    let (crumb_ptr, _) = crumb.mapping();
    let (read_ok, exited) = child_can_read(crumb_ptr);
    assert!(
        read_ok && exited,
        "the BREADCRUMB must stay inherited — it is the child's only liveness channel, \
         and excluding it would kill every child at the first stall timeout"
    );

    // The STATE RING, driven through its REAL production shape rather than a raw read:
    // the producer role is handed to the CHILD for the child's lifetime (the SPSC
    // invariant), so the child PUSHES and a consumer opened afterwards must see it.
    // That proves the mapping is not merely readable but genuinely SHARED — a private
    // copy would take the push and show the consumer nothing.
    let ring_tag = format!("{tag}_ring");
    let mut ring = cerulion_core::state_ring::StateRingOwner::create(&ring_tag, 8, 0, 7, &["n0"])
        .expect("create state ring");
    let ring_name = ring.name().to_string();
    let mut producer = ring.producer().expect("take the producer");

    // SAFETY: the child pushes one record through the inherited producer and `_exit`s.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        producer.push_skip(
            1,
            0,
            cerulion_core::state_ring::SkipCause::CaptureFailed,
            "complement probe",
        );
        unsafe { libc::_exit(0) };
    }
    let status = reap(pid);
    assert!(
        libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
        "the child must PUSH into the state ring and exit cleanly; a faulting child means the ring \
         was excluded, and then no anchor this design produces can reach the bag (status \
         {status:#x})"
    );

    let consumer = cerulion_core::state_ring::StateRingConsumer::open(&ring_name)
        .expect("open the state ring");
    assert!(
        consumer.available() >= 1,
        "the child's record did not reach the SHARED segment: the state ring must stay inherited \
         AND shared, or every capture writes into a private copy and the reader reports \
         PartialAnchor for a capture that ran perfectly"
    );
}

/// The Linux-specific evidence: the kernel's OWN view of the mapping says `dc`.
///
/// The behavioural arm above proves the OUTCOME on both platforms. This one proves the
/// MECHANISM on the robot's platform — that `MADV_DONTFORK` really set `VM_DONTCOPY` on
/// this exact VMA, which is what an operator would check by hand on a live robot, and
/// what `shm_guard`'s sibling `MADV_NOHUGEPAGE` pin checks for its own advice.
///
/// Not `#[ignore]`d, unlike that sibling: it needs no iceoryx2 pool and no box, only an
/// anonymous page and `/proc/self/smaps`.
#[cfg(target_os = "linux")]
#[test]
fn on_linux_the_excluded_vma_carries_the_kernels_own_dontcopy_flag() {
    let p = map_shared_page();
    // SAFETY: a live mapping this process owns.
    assert_eq!(
        unsafe { exclude_from_fork(p, PAGE) },
        ExclusionOutcome::Excluded
    );

    let addr = p as usize;
    let smaps = std::fs::read_to_string("/proc/self/smaps").expect("read smaps");
    let mut flags: Option<String> = None;
    let mut in_vma = false;
    for line in smaps.lines() {
        if let Some((range, _)) = line.split_once(' ') {
            if let Some((lo, hi)) = range.split_once('-') {
                if let (Ok(lo), Ok(hi)) =
                    (usize::from_str_radix(lo, 16), usize::from_str_radix(hi, 16))
                {
                    in_vma = (lo..hi).contains(&addr);
                    continue;
                }
            }
        }
        if in_vma {
            if let Some(rest) = line.strip_prefix("VmFlags:") {
                flags = Some(rest.trim().to_string());
                break;
            }
        }
    }
    unsafe { libc::munmap(p, PAGE) };

    let flags = flags.unwrap_or_else(|| panic!("no VmFlags line for the mapping at {addr:#x}"));
    assert!(
        flags.split_whitespace().any(|f| f == "dc"),
        "the excluded VMA must carry the kernel's VM_DONTCOPY flag (`dc`); VmFlags were: {flags}"
    );
}

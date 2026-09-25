// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(all(target_os = "linux", target_env = "gnu"))]
//! Adopt-take, the REAL-`LD_PRELOAD` arms
//! (release e2e, and fork): the adopted plain take driven
//! against the GENUINE `libcerulion_heaphook.so`, where the app's ordinary
//! libc `free(msg.data)` really is the interposed one — the true
//! classify-by-range → release-callback → `Arc` drop → SHM-borrow release
//! chain, plus the atfork child fold (an inherited forged-range free in a
//! fork child must be a counted quarantine NO-OP, never a callback, never
//! glibc on an SHM address). This is exactly what the fake-hook file
//! (`rmw_adopt_take_test.rs`) cannot prove.
//!
//! # Shape: self-re-exec with `LD_PRELOAD`
//!
//! The crib of `rmw_borrow_window_linux_test.rs`: `rmw_cerulion`
//! deliberately does not link the hook crate, so a re-exec of THIS test
//! binary with `LD_PRELOAD=libcerulion_heaphook.so` is a clean consumer —
//! the `cerulion_heaphook_*` symbols reach it only through the preload.
//!
//! # Where this runs
//!
//! `#[ignore]`d, Linux/GNU only — a Linux host or the ros2-bench container
//! (iceoryx2 SHM must work; macOS cannot load the hook at all). These arms
//! cannot run on macOS: run them on Linux to validate a change to this
//! path. Build the hook cdylib under the SAME profile first:
//!
//! ```bash
//! cargo build -p cerulion_heaphook
//! cargo test -p rmw_cerulion --test rmw_adopt_take_linux_test -- --ignored --test-threads=1 --nocapture
//! ```

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime};

use rmw_cerulion::adopt_take::{ADOPT_TAKE_BUDGET_ENV, ADOPT_TAKE_ENV};
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::heaphook::{active_hook, process_verdict, HandshakeVerdict};
use rmw_cerulion::runtime::SubscriptionData;
use rmw_cerulion::*;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_STRING: u8 = 16;
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);
/// Guard env: the child arms run ONLY under the parent's re-exec (a bare
/// `--ignored` sweep must not run them un-preloaded and fail confusingly).
const CHILD_ENV: &str = "CER_RMW_ADOPT_TAKE_LINUX_CHILD";

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

// ── artifact location + freshness gate (the repo's stale-cdylib
//    phantom-pass discipline — crib of rmw_borrow_window_linux_test) ──────

fn profile_dir() -> PathBuf {
    let exe = std::env::current_exe().expect("current_exe");
    exe.parent()
        .and_then(|p| p.parent())
        .expect("profile dir")
        .to_path_buf()
}

fn hook_so() -> PathBuf {
    let p = profile_dir().join("libcerulion_heaphook.so");
    assert!(
        p.exists(),
        "libcerulion_heaphook.so not found at {} — build it under THIS profile \
         first: cargo build -p cerulion_heaphook",
        p.display()
    );
    let so_m = p
        .metadata()
        .and_then(|m| m.modified())
        .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", p.display()));
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .join("cerulion_heaphook");
    let mut newest = newest_rs_mtime(&root.join("src"))
        .unwrap_or_else(|e| panic!("cannot establish hook .so freshness: {e}"));
    let manifest = root.join("Cargo.toml");
    newest = newest.max(
        manifest
            .metadata()
            .and_then(|m| m.modified())
            .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", manifest.display())),
    );
    assert!(
        so_m >= newest,
        "STALE libcerulion_heaphook.so at {} — its sources changed after it was \
         built; rebuild under THIS profile: cargo build -p cerulion_heaphook",
        p.display()
    );
    p
}

/// Newest `.rs` mtime under `dir`, recursive, fail-closed.
fn newest_rs_mtime(dir: &Path) -> Result<SystemTime, String> {
    fn walk(dir: &Path) -> Result<Option<SystemTime>, String> {
        let read = std::fs::read_dir(dir)
            .map_err(|e| format!("read_dir({}) failed: {e}", dir.display()))?;
        let mut newest: Option<SystemTime> = None;
        for entry in read {
            let entry =
                entry.map_err(|e| format!("dir entry in {} unreadable: {e}", dir.display()))?;
            let path = entry.path();
            let meta = entry
                .metadata()
                .map_err(|e| format!("metadata of {} unreadable: {e}", path.display()))?;
            let m = if meta.is_dir() {
                match walk(&path)? {
                    Some(m) => m,
                    None => continue,
                }
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                meta.modified()
                    .map_err(|e| format!("mtime of {} unreadable: {e}", path.display()))?
            } else {
                continue;
            };
            newest = Some(newest.map_or(m, |n: SystemTime| n.max(m)));
        }
        Ok(newest)
    }
    walk(dir)?.ok_or_else(|| format!("no .rs files under {}", dir.display()))
}

// ── C fixture: one forgeable f32 sequence + a string ─────────────────────

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

#[repr(C)]
struct CF32Seq {
    data: *mut f32,
    size: usize,
    capacity: usize,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

#[repr(C)]
struct CScan {
    angle_min: f32,
    ranges: CF32Seq,
    frame_id: CRosString,
}

unsafe extern "C" fn scan_init(
    msg: *mut c_void,
    _init: ffi::rosidl_runtime_c__message_initialization,
) {
    let m = &mut *(msg as *mut CScan);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

unsafe extern "C" fn scan_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CScan);
    for p in [m.frame_id.data as *mut c_void, m.ranges.data as *mut c_void] {
        if !p.is_null() {
            // Under the preload this IS the interposed free — a still-forged
            // `ranges.data` routes to a release, exactly like a real app's
            // fini. That routing is the feature under test.
            free(p);
        }
    }
    m.frame_id.data = std::ptr::null_mut();
    m.ranges.data = std::ptr::null_mut();
}

fn scan_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let mut ranges = ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr("ranges"),
        type_id_: ROS_TYPE_FLOAT,
        offset_: std::mem::offset_of!(CScan, ranges) as u32,
        ..Default::default()
    };
    ranges.is_array_ = true;
    let members = Box::leak(
        vec![
            ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("angle_min"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: 0,
                ..Default::default()
            },
            ranges,
            ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("frame_id"),
                type_id_: ROS_TYPE_STRING,
                offset_: std::mem::offset_of!(CScan, frame_id) as u32,
                ..Default::default()
            },
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_adopt_lx__msg"),
            message_name_: cstr(unique),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScan>(),
            members_: members.as_ptr(),
            init_function: Some(scan_init),
            fini_function: Some(scan_fini),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

// ── Shared scaffolding ────────────────────────────────────────────────────

unsafe fn setup_pair(
    tag: &str,
) -> (
    *mut ffi::rmw_node_t,
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
) {
    let suffix = std::process::id() as u64;
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(&format!("{tag}_{suffix}")), cstr("/"));
    assert!(!node.is_null());
    let ts = scan_ts(&format!("Lx{tag}{suffix}"));
    let topic = CString::new(format!("/rmw_adopt_lx/{tag}/{suffix}")).expect("topic");
    let qos = ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 8,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    };
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null());
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null());
    (node, publisher, subscription)
}

unsafe fn publish_scan(publisher: *const ffi::rmw_publisher_t, n: usize, seed: f32) -> Vec<f32> {
    let want: Vec<f32> = (0..n).map(|i| seed + i as f32 * 0.25).collect();
    let rdata = calloc(n.max(1), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(want.as_ptr(), rdata, n);
    let sdata = calloc(6, 1) as *mut u8;
    std::ptr::copy_nonoverlapping("laser".as_ptr(), sdata, 5);
    let msg = CScan {
        angle_min: seed,
        ranges: CF32Seq {
            data: rdata,
            size: n,
            capacity: n,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        frame_id: CRosString {
            data: sdata,
            size: 5,
            capacity: 6,
        },
    };
    assert_eq!(
        rmw_publish(
            publisher,
            &msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    free(rdata as *mut c_void);
    free(sdata as *mut c_void);
    want
}

unsafe fn take_scan(subscription: *const ffi::rmw_subscription_t) -> (bool, Box<CScan>) {
    let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
    scan_init(&mut *msg as *mut _ as *mut c_void, 0);
    let mut taken = false;
    assert_eq!(
        rmw_take(
            subscription,
            &mut *msg as *mut _ as *mut c_void,
            &mut taken,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    (taken, msg)
}

unsafe fn stats(
    subscription: *const ffi::rmw_subscription_t,
) -> &'static rmw_cerulion::adopt_take::AdoptStats {
    let data = &*((*subscription).data as *const SubscriptionData);
    &data.adopt.as_ref().expect("adopt-armed").stats
}

// =====================================================================
// Child arms (run ONLY under the parent's preloaded re-exec)
// =====================================================================

/// Release e2e: adopted take → REAL interposed `free` → borrow slot
/// observably freed (a past-budget refusal recovers), release accounting
/// exact.
#[test]
#[ignore]
fn child_adopt_release_e2e() {
    if std::env::var(CHILD_ENV).is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    assert_eq!(
        process_verdict(),
        HandshakeVerdict::Active,
        "the preload must resolve Active in the child"
    );
    unsafe {
        let (_node, publisher, subscription) = setup_pair("rel");
        let s = stats(subscription);

        // One adopted take, released by the app's ordinary free().
        let want = publish_scan(publisher, 64, 1.0);
        let (taken, mut msg) = take_scan(subscription);
        assert!(taken);
        let seen = std::slice::from_raw_parts(msg.ranges.data, msg.ranges.size);
        assert_eq!(seen.len(), want.len());
        for (a, b) in seen.iter().zip(want.iter()) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
        assert_eq!(s.adopted_takes.load(Ordering::Relaxed), 1);
        assert_eq!(s.outstanding.load(Ordering::Relaxed), 1);
        free(msg.ranges.data as *mut c_void); // THE interposed free
        msg.ranges.data = std::ptr::null_mut();
        assert_eq!(
            s.releases.load(Ordering::Relaxed),
            1,
            "the interposed free must fire the release callback"
        );
        assert_eq!(s.outstanding.load(Ordering::Relaxed), 0);
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // Budget (env=4): hold 4, the 5th refuses taken=false, a real free
        // self-heals it.
        for i in 0..5 {
            publish_scan(publisher, 16, 10.0 + i as f32);
        }
        let mut held = Vec::new();
        for _ in 0..4 {
            let (taken, msg) = take_scan(subscription);
            assert!(taken);
            held.push(msg);
        }
        let (taken, mut refused) = take_scan(subscription);
        assert!(!taken, "past the budget: taken=false, never an error");
        assert!(s.budget_refusals.load(Ordering::Relaxed) >= 1);
        scan_fini(&mut *refused as *mut _ as *mut c_void);
        let mut first = held.remove(0);
        free(first.ranges.data as *mut c_void);
        first.ranges.data = std::ptr::null_mut();
        scan_fini(&mut *first as *mut _ as *mut c_void);
        let (taken, msg) = take_scan(subscription);
        assert!(taken, "freeing one adopted message recovers the take");
        held.push(msg);
        for mut msg in held {
            free(msg.ranges.data as *mut c_void);
            msg.ranges.data = std::ptr::null_mut();
            scan_fini(&mut *msg as *mut _ as *mut c_void);
        }
        assert_eq!(s.outstanding.load(Ordering::Relaxed), 0);
        println!("CHILD_RELEASE_OK");
    }
}

/// Fork: adopt → fork → the FORK CHILD's inherited free is a
/// counted quarantine NO-OP (atfork fold + cleared callback), never a
/// release; the parent's own free afterwards releases normally.
#[test]
#[ignore]
fn child_adopt_fork_quarantine() {
    if std::env::var(CHILD_ENV).is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    assert_eq!(process_verdict(), HandshakeVerdict::Active);
    unsafe {
        let (_node, publisher, subscription) = setup_pair("fork");
        let s = stats(subscription);
        publish_scan(publisher, 32, 2.0);
        let (taken, mut msg) = take_scan(subscription);
        assert!(taken);
        assert_eq!(s.outstanding.load(Ordering::Relaxed), 1);
        let api = active_hook().expect("Active");
        let quarantine_noops_before = (api.counter)(3);

        let pid = libc::fork();
        assert!(pid >= 0, "fork failed");
        if pid == 0 {
            // FORK CHILD: the inherited forged-vector free must be a
            // quarantine no-op — counter kind 3 bumps, releases stays 0.
            // _exit codes carry the verdict (no libtest teardown in here).
            free(msg.ranges.data as *mut c_void);
            if s.releases.load(Ordering::Relaxed) != 0 {
                libc::_exit(41);
            }
            if (api.counter)(3) != quarantine_noops_before + 1 {
                libc::_exit(42);
            }
            libc::_exit(0);
        }
        let mut status: i32 = 0;
        let waited = libc::waitpid(pid, &mut status, 0);
        assert_eq!(waited, pid, "waitpid");
        assert!(libc::WIFEXITED(status), "fork child must exit cleanly");
        assert_eq!(
            libc::WEXITSTATUS(status),
            0,
            "fork-child verdict (41 = release fired in the child, 42 = no quarantine no-op)"
        );

        // The PARENT's own free still releases normally.
        free(msg.ranges.data as *mut c_void);
        msg.ranges.data = std::ptr::null_mut();
        assert_eq!(s.releases.load(Ordering::Relaxed), 1);
        assert_eq!(s.outstanding.load(Ordering::Relaxed), 0);
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        println!("CHILD_FORK_OK");
    }
}

// =====================================================================
// The parents
// =====================================================================

fn run_child(name: &str) -> String {
    run_child_with_budget(name, "4")
}

/// The same harness with the adopt budget as a parameter — the
/// reuse-wedge arm needs `BUDGET=1`, the shape a gate-first ordering wedges.
fn run_child_with_budget(name: &str, budget: &str) -> String {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([
        "--exact",
        name,
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(CHILD_ENV, "1")
    .env(ADOPT_TAKE_ENV, "1")
    .env(ADOPT_TAKE_BUDGET_ENV, budget)
    .env("LD_PRELOAD", hook_so())
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    let mut child = cmd.spawn().expect("spawn child");
    // BOTH pipes are drained on their own threads for the WHOLE
    // wait. Draining only after the loop (via `wait_with_output`) deadlocks
    // a child that fills a pipe buffer — 64 KiB on Linux: it blocks in
    // `write`, never exits, and the harness reports a spurious timeout
    // instead of the child's own verdict. The sibling harness in
    // `rmw_adopt_take_test.rs` nulls stdout and reads stderr on a thread
    // for exactly this reason; this one NEEDS stdout (the `CHILD_*_OK`
    // markers are on it), so it drains both. Not hypothetical: the child
    // runs libtest with `--nocapture` over real iceoryx2 and INHERITS the
    // operator's ambient `RUST_LOG`/`IOX2_LOG_LEVEL` (this harness sets
    // neither, so the fallback is warn-level), and the preloaded hook
    // writes on its own account — either pipe can fill without anything
    // in this file raising a log level.
    let mut child_stdout = child.stdout.take().expect("stdout piped");
    let mut child_stderr = child.stderr.take().expect("stderr piped");
    // Each reader returns its read RESULT alongside the bytes: swallowing
    // an I/O error would truncate the buffer silently, and the caller
    // would then blame the child for a missing marker that a harness fault
    // ate.
    let out_reader = std::thread::spawn(move || {
        use std::io::Read;
        let mut buf = Vec::new();
        let outcome = child_stdout.read_to_end(&mut buf);
        (buf, outcome)
    });
    let err_reader = std::thread::spawn(move || {
        use std::io::Read;
        let mut buf = Vec::new();
        let outcome = child_stderr.read_to_end(&mut buf);
        (buf, outcome)
    });
    // `join` is a reader-thread handle: resume the panic rather than
    // rendering an opaque `Box<dyn Any>`, so a fault in the drain is
    // reported as itself. NOTE the joins sit OUTSIDE `CHILD_TIMEOUT` —
    // they end at EOF, which a killed or exited child guarantees
    // (the forking arm `waitpid`s its fork child before printing its
    // marker, so no grandchild inherits a write end). An arm that
    // lets a descendant outlive the child would turn a bounded failure
    // into a hang here.
    let drain = |handle: std::thread::JoinHandle<(Vec<u8>, std::io::Result<usize>)>, what: &str| {
        let (buf, outcome) = handle
            .join()
            .unwrap_or_else(|e| std::panic::resume_unwind(e));
        if let Err(e) = outcome {
            panic!("reading child {name}'s {what} failed: {e}");
        }
        String::from_utf8_lossy(&buf).into_owned()
    };
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if Instant::now() > deadline => {
                // Killing the child closes both write ends, so the readers
                // reach EOF at once and the diagnostic they hold — the
                // whole point of draining them — reaches the failure
                // message instead of being discarded with the handles.
                let _ = child.kill();
                let _ = child.wait();
                let stdout = drain(out_reader, "stdout");
                let stderr = drain(err_reader, "stderr");
                panic!(
                    "child {name} timed out after {CHILD_TIMEOUT:?}\n--- stdout ---\n{stdout}\
                     \n--- stderr ---\n{stderr}"
                );
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    };
    let stdout = drain(out_reader, "stdout");
    let stderr = drain(err_reader, "stderr");
    assert!(
        status.success(),
        "child {name} failed\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    );
    stdout
}

/// A caller reusing one message
/// buffer must not wedge at the retention budget.
///
/// `release_forgeable_members` — the call that frees a reused buffer's
/// previously-forged pointers and so drops the `Arc` that decrements
/// `outstanding` — runs inside the take. With the retention gate
/// BEFORE it, once `outstanding` reaches the budget the gate refuses
/// first, the release never runs, `outstanding` never falls, and every later
/// take is refused too. At `BUDGET=1` with one reused buffer that is the
/// second take onward, forever.
///
/// Linux-only like its siblings: the reuse path frees a forged SHM pointer,
/// which is safe only because the REAL hook interposes it.
#[test]
#[ignore]
fn child_adopt_reused_buffer_does_not_wedge_at_the_budget() {
    if std::env::var(CHILD_ENV).is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    assert_eq!(
        process_verdict(),
        HandshakeVerdict::Active,
        "the preload must resolve Active in the child"
    );
    unsafe {
        let (_node, publisher, subscription) = setup_pair("reuse");
        let s = stats(subscription);
        // ONE buffer, reused across takes, never `fini`d between them — the
        // sharpest shape, and the one a gate-first ordering wedges.
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        for round in 0..6 {
            publish_scan(publisher, 32, round as f32);
            let mut taken = false;
            let ret = rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            );
            assert_eq!(ret, RMW_RET_OK, "round {round}");
            assert!(
                taken,
                "round {round}: a reused buffer must keep being served — its own release is \
                 what frees the slot, so a gate that refuses first wedges forever"
            );
            assert!(
                s.outstanding.load(Ordering::Relaxed) <= 1,
                "round {round}: never more retained than the configured budget"
            );
            // `taken` + `outstanding <= 1` is
            // satisfied by `outstanding == 0` — i.e. by every take silently
            // taking the COPY path, which is exactly the shape this
            // test exists to forbid. The reuse wedge is a claim about
            // ADOPTION and RELEASE, so assert both.
            assert_eq!(
                s.adopted_takes.load(Ordering::Relaxed),
                round as u64 + 1,
                "round {round}: every reuse round must ADOPT — a copy-path fallback would \
                 satisfy `taken` and `outstanding <= 1` while proving nothing about the \
                 retention/release cycle this test pins"
            );
            assert_eq!(
                s.outstanding.load(Ordering::Relaxed),
                1,
                "round {round}: …and the adopted sample is RETAINED, so the next round's \
                 own release is what frees the slot"
            );
            assert_eq!(
                s.releases.load(Ordering::Relaxed),
                round as u64,
                "round {round}: each round after the first releases the PREVIOUS round's \
                 forged range — through `release_forgeable_members`, inside the take"
            );
        }
        assert_eq!(
            s.fallbacks.load(Ordering::Relaxed),
            0,
            "no take in the reuse loop fell back to copying"
        );
        free(msg.ranges.data as *mut c_void);
        msg.ranges.data = std::ptr::null_mut();
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        println!("CHILD_REUSE_OK");
    }
}

#[test]
#[ignore]
#[serial]
fn real_ld_preload_adopted_take_releases_on_free() {
    let out = run_child("child_adopt_release_e2e");
    assert!(
        out.contains("CHILD_RELEASE_OK"),
        "release child must reach its marker:\n{out}"
    );
}

#[test]
#[ignore]
#[serial]
fn a_reused_buffer_does_not_wedge_at_the_retention_budget() {
    let out = run_child_with_budget(
        "child_adopt_reused_buffer_does_not_wedge_at_the_budget",
        "1",
    );
    assert!(
        out.contains("CHILD_REUSE_OK"),
        "reuse child must reach its marker:\n{out}"
    );
}

#[test]
#[ignore]
#[serial]
fn adopted_ranges_survive_fork_as_quarantine_noops() {
    let out = run_child("child_adopt_fork_quarantine");
    assert!(
        out.contains("CHILD_FORK_OK"),
        "fork child must reach its marker:\n{out}"
    );
}

/// Every real arm in this binary is `#[ignore]` (Linux-only: the genuine
/// `libcerulion_heaphook.so` must be built and preloadable). This one is
/// NOT, and its NAME is the whole message: libtest prints every test's
/// `test <name> ... ok` line whether or not output is captured, so the name
/// is the one channel a plain `cargo test` always shows.
///
/// The six ignored arms are 3 PARENT arms
/// (`real_ld_preload_adopted_take_releases_on_free`,
/// `a_reused_buffer_does_not_wedge_at_the_retention_budget`,
/// `adopted_ranges_survive_fork_as_quarantine_noops`) plus the 3
/// `child_*` entrypoints each parent re-execs under the preload — so a
/// plain `cargo test` on this binary reports `1 passed; 6 ignored`, and
/// that is this fn plus all of them skipped.
///
/// Why the name carries the count at all: a bare `0 passed; N ignored`
/// line is easily mistaken for green, which makes a run of this binary
/// vacuous. Keep this name in step with
/// what libtest actually prints. No `eprintln!`: a captured line that only
/// appears under `--nocapture`, where the operator is already looking, is
/// dead weight. It asserts nothing; the real arms run with
/// `-- --ignored --test-threads=1` after `cargo build -p cerulion_heaphook`.
#[test]
fn box_only_six_ignored_arms_three_parents_and_three_children_run_with_dash_dash_ignored_test_threads_1(
) {
}

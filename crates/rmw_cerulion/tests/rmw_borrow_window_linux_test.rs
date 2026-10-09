// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(all(target_os = "linux", target_env = "gnu"))]
//! The REAL-`LD_PRELOAD` arm: the windowed borrow driven
//! against the GENUINE `libcerulion_heaphook.so` — the true per-symbol
//! dlsym handshake, the real interposed `malloc` bumping a fill into the
//! armed window, the real quarantine behind `fini`. This is exactly what
//! the fake-hook file (`rmw_borrow_publish_test.rs`) cannot prove.
//!
//! # Shape: self-re-exec with `LD_PRELOAD`
//!
//! `rmw_cerulion` deliberately does NOT link the hook crate (see
//! `src/heaphook.rs` — linking its rlib would compile a second malloc
//! interposer into every rmw binary), so a re-exec of THIS test binary
//! with `LD_PRELOAD=libcerulion_heaphook.so` is a clean consumer: the
//! `cerulion_heaphook_*` symbols reach it only through the preload, and
//! the no-preload control genuinely resolves Absent. (Contrast the hook
//! crate's own e2e, which needs a separate probe fixture precisely
//! because it links the rlib.)
//!
//! # Where this runs
//!
//! `#[ignore]`d, Linux/GNU only — a Linux host or the ros2-bench container
//! (iceoryx2 SHM must work; macOS cannot load the hook at all). Build
//! the hook cdylib under the SAME profile first:
//!
//! ```bash
//! cargo build -p cerulion_heaphook
//! cargo test -p rmw_cerulion --test rmw_borrow_window_linux_test -- --ignored --test-threads=1 --nocapture
//! ```

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::wire::WireHeader;
use rmw_cerulion::ffi::{self, RMW_RET_OK, RMW_RET_UNSUPPORTED};
use rmw_cerulion::heaphook::{process_verdict, HandshakeVerdict};
use rmw_cerulion::runtime::PublisherData;
use rmw_cerulion::*;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_STRING: u8 = 16;
const CHILD_TIMEOUT: Duration = Duration::from_secs(60);
/// Guard env: the child arms run ONLY under the parent's re-exec (a bare
/// `--ignored` sweep must not run them un-preloaded and fail confusingly).
const CHILD_ENV: &str = "CER_ROUTE_S_STAGE3_CHILD";

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    fn malloc(size: usize) -> *mut c_void;
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

// ── artifact location (crib of the heaphook e2e — no sibling-profile
//    fallback: a stale other-profile .so is the phantom-pass class) ────────

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
    // Freshness gate (the repo's stale-cdylib phantom-pass class — the crib
    // of `heaphook_e2e_test::assert_fresh`): `cargo test` does not rebuild
    // the hook's `.so`, so an old artifact left from before a source edit
    // would run this whole arm against yesterday's interposer. Fail CLOSED:
    // any input that cannot be timed is an input whose freshness against
    // the `.so` cannot be proven.
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
    let manifest_m = manifest
        .metadata()
        .and_then(|m| m.modified())
        .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", manifest.display()));
    newest = newest.max(manifest_m);
    // build.rs is OPTIONAL — folded in only when it exists; anything but a
    // genuine NotFound fails loudly (an unreadable build.rs must not
    // silently weaken the gate).
    let build_rs = root.join("build.rs");
    match build_rs.metadata() {
        Ok(m) => {
            let build_m = m
                .modified()
                .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", build_rs.display()));
            newest = newest.max(build_m);
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => panic!("cannot stat {}: {e}", build_rs.display()),
    }
    assert!(
        so_m >= newest,
        "STALE libcerulion_heaphook.so at {} — its sources changed after it was \
         built; rebuild under THIS profile: cargo build -p cerulion_heaphook",
        p.display()
    );
    p
}

/// Newest mtime among the `.rs` files under `dir`, RECURSIVELY, fail-closed
/// (`Err` on an unreadable tree / entry / mtime, or a tree with no `.rs` at
/// all) — the `heaphook_e2e_test::newest_rs_mtime` discipline.
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
            newest = Some(newest.map_or(m, |cur| cur.max(m)));
        }
        Ok(newest)
    }
    walk(dir)?.ok_or_else(|| format!("no `.rs` sources found under {}", dir.display()))
}

// ── the unbounded fixture (Scanish — crib of rmw_borrow_publish_test) ─────

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
struct CScanish {
    angle_min: f32,
    ranges: CF32Seq,
    frame_id: CRosString,
}

unsafe extern "C" fn scanish_init(
    msg: *mut c_void,
    _init: ffi::rosidl_runtime_c__message_initialization,
) {
    let m = &mut *(msg as *mut CScanish);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

unsafe extern "C" fn scanish_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CScanish);
    if !m.frame_id.data.is_null() {
        free(m.frame_id.data as *mut c_void);
        m.frame_id.data = std::ptr::null_mut();
    }
    if !m.ranges.data.is_null() {
        free(m.ranges.data as *mut c_void);
        m.ranges.data = std::ptr::null_mut();
    }
}

fn scanish_ts(type_name: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = vec![
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr("angle_min"),
            type_id_: ROS_TYPE_FLOAT,
            offset_: 0,
            ..Default::default()
        },
        {
            let mut m = ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("ranges"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: std::mem::offset_of!(CScanish, ranges) as u32,
                ..Default::default()
            };
            m.is_array_ = true;
            m.array_size_ = 0;
            m.is_upper_bound_ = false;
            m
        },
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr("frame_id"),
            type_id_: ROS_TYPE_STRING,
            offset_: std::mem::offset_of!(CScanish, frame_id) as u32,
            ..Default::default()
        },
    ];
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("borrow_ld__msg"),
            message_name_: cstr(type_name),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScanish>(),
            members_: members.as_ptr(),
            init_function: Some(scanish_init),
            fini_function: Some(scanish_fini),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

unsafe fn setup_pair(
    tag: &str,
) -> (
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
    *const PublisherData,
    *const ffi::rosidl_message_type_support_t,
) {
    let suffix = std::process::id() as u64 ^ 0x5157;
    let ts = scanish_ts(&format!("ScanLd{tag}{suffix}"));
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(&format!("ld_{tag}_{suffix}")), cstr("/"));
    assert!(!node.is_null());
    let topic = CString::new(format!("/borrow_ld/{tag}/{suffix}")).expect("topic");
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
    let pdata = (*publisher).data as *const PublisherData;
    (publisher, subscription, pdata, ts)
}

unsafe extern "C" fn alloc_fn(size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(1, size)
}
unsafe extern "C" fn dealloc_fn(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}

unsafe fn take_raw(subscription: *const ffi::rmw_subscription_t) -> (WireHeader, Vec<u8>) {
    let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
    serialized.allocator.allocate = Some(alloc_fn);
    serialized.allocator.deallocate = Some(dealloc_fn);
    let mut taken = false;
    assert_eq!(
        rmw_take_serialized_message(
            subscription,
            &mut serialized,
            &mut taken,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    assert!(taken, "the published frame must be takeable");
    let frame = std::slice::from_raw_parts(serialized.buffer, serialized.buffer_length).to_vec();
    free(serialized.buffer as *mut c_void);
    let header = WireHeader::read_from_buf(&frame).expect("header");
    (header, frame[WireHeader::SIZE..].to_vec())
}

// =====================================================================
// Child arms (run only under the parent's re-exec)
// =====================================================================

/// PRELOADED child: the genuine handshake is Active and a REAL interposed
/// `malloc` fill is adopted zero-copy.
#[test]
#[ignore]
#[serial]
fn child_real_hook_adopts() {
    if std::env::var(CHILD_ENV).is_err() {
        eprintln!("child arm skipped (run through the parent re-exec)");
        return;
    }
    unsafe {
        assert_eq!(
            process_verdict(),
            HandshakeVerdict::Active,
            "the preloaded hook must resolve through the real per-symbol handshake"
        );
        let (publisher, subscription, pdata, ts) = setup_pair("adopt");
        assert!((*publisher).can_loan_messages);

        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut msg),
            RMW_RET_OK
        );
        assert!(!msg.is_null());

        // THE REAL FILL: this malloc goes through the PRELOADED interposer,
        // and — with the window armed by the borrow — must bump into the
        // slot tail. No addresses are simulated anywhere in this arm.
        let ranges = [1.5f32, -2.25, 3.0, 1.0e-3];
        let dst = malloc(ranges.len() * 4) as *mut f32;
        assert!(!dst.is_null());
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), dst, ranges.len());
        let m = &mut *(msg as *mut CScanish);
        m.angle_min = -0.75;
        m.ranges = CF32Seq {
            data: dst,
            size: ranges.len(),
            capacity: ranges.len(),
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        };
        // The string too: a real interposed calloc (also lands in-window;
        // the seal copies it above the cursor and empties the in-slot
        // header before fini — the hookless-defense walk, here exercised
        // WITH the hook underneath).
        let s = calloc(6, 1) as *mut u8;
        std::ptr::copy_nonoverlapping(b"lidar".as_ptr(), s, 5);
        free(m.frame_id.data as *mut c_void);
        m.frame_id = CRosString {
            data: s,
            size: 5,
            capacity: 6,
        };

        assert_eq!(
            rmw_publish_loaned_message(publisher, msg, std::ptr::null_mut()),
            RMW_RET_OK
        );
        // The Principle-#3 counters are the adoption proof.
        assert_eq!(
            (*pdata).borrow_adopted_count(),
            1,
            "a real bump-backed fill must ADOPT (zero-copy)"
        );
        assert_eq!((*pdata).borrow_copied_count(), 0);
        assert_eq!((*pdata).borrow_degrade_count(), 0);

        // And the frame: the adopted entry sits in the TAIL (past the
        // struct region — the gap-frame placement), bytes exact.
        let (header, payload) = take_raw(subscription);
        assert_eq!(header.sequence, 0);
        let fixed = 4usize;
        let e0_off = u32::from_le_bytes(payload[fixed..fixed + 4].try_into().unwrap()) as usize;
        let e0_len = u32::from_le_bytes(payload[fixed + 4..fixed + 8].try_into().unwrap()) as usize;
        assert_eq!(e0_len, 16);
        assert!(
            e0_off >= std::mem::size_of::<CScanish>(),
            "the adopted entry lies in the fill tail, past the struct region \
             (off {e0_off})"
        );
        for (i, r) in ranges.iter().enumerate() {
            assert_eq!(
                &payload[e0_off + 4 * i..e0_off + 4 * i + 4],
                &r.to_le_bytes(),
                "adopted range {i}"
            );
        }
        let e1_off =
            u32::from_le_bytes(payload[fixed + 8..fixed + 12].try_into().unwrap()) as usize;
        let e1_len =
            u32::from_le_bytes(payload[fixed + 12..fixed + 16].try_into().unwrap()) as usize;
        assert_eq!(&payload[e1_off..e1_off + e1_len], b"lidar");
        println!("CHILD_ADOPT_OK");
    }
}

/// NO-PRELOAD control child: the same binary, the same type — the
/// handshake resolves Absent and the surface is exactly the copy path.
#[test]
#[ignore]
#[serial]
fn child_no_hook_degrades() {
    if std::env::var(CHILD_ENV).is_err() {
        eprintln!("child arm skipped (run through the parent re-exec)");
        return;
    }
    unsafe {
        assert_eq!(process_verdict(), HandshakeVerdict::DegradeAbsent);
        let (publisher, _subscription, _pdata, ts) = setup_pair("nohook");
        assert!(!(*publisher).can_loan_messages);
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut msg),
            RMW_RET_UNSUPPORTED
        );
        println!("CHILD_DEGRADE_OK");
    }
}

// =====================================================================
// The parent: re-exec the two children, with and without the preload
// =====================================================================

fn run_child(name: &str, preload: bool) -> String {
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
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    if preload {
        cmd.env("LD_PRELOAD", hook_so());
    } else {
        cmd.env_remove("LD_PRELOAD");
    }
    let mut child = cmd.spawn().expect("spawn child");
    let deadline = Instant::now() + CHILD_TIMEOUT;
    loop {
        match child.try_wait().expect("try_wait") {
            Some(_) => break,
            None if Instant::now() > deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("child {name} timed out after {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    let out = child.wait_with_output().expect("output");
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    assert!(
        out.status.success(),
        "child {name} (preload={preload}) failed\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    );
    stdout
}

#[test]
#[ignore]
#[serial]
fn real_ld_preload_borrow_adopts_and_no_preload_degrades() {
    let adopted = run_child("child_real_hook_adopts", true);
    assert!(
        adopted.contains("CHILD_ADOPT_OK"),
        "adopt child must reach its marker:\n{adopted}"
    );
    let degraded = run_child("child_no_hook_degrades", false);
    assert!(
        degraded.contains("CHILD_DEGRADE_OK"),
        "degrade child must reach its marker:\n{degraded}"
    );
}

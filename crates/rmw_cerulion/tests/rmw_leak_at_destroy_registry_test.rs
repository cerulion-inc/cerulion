// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! A publisher destroyed while it must LEAK a windowed slot keeps
//! its iceoryx2 port REGISTERED — so the dead-node sweep of the NEXT process
//! can reclaim this process's node.
//!
//! The bug, traced in the
//! iceoryx2 0.9.1 source: an `rmw_destroy_publisher` that `mem::forget`s the
//! leaked loans and then DROPS the `PublisherData` orphans the tag. iceoryx2's
//! `Publisher::drop` only deregisters the port (`src/port/publisher.rs:
//! 380-395`); the port's on-disk `.port_tag` is the LAST field of the
//! `Arc`-shared `PublisherSharedState` (`:194`) every forgotten sample keeps
//! alive. So the process exits with a DEREGISTERED port whose tag still sits
//! in `<root>/nodes/<node_id>/`. The dead-node sweep's service-tags pass
//! removes tags ONLY for ports still registered (`src/service/mod.rs:795`),
//! its port-tags pass never deletes the tag, and `remove_node`'s `rmdir`
//! fails ENOTEMPTY (`src/node/mod.rs:821-855`) — `failed_cleanups == 1` on
//! EVERY sweep, forever, so `cerulion clean` never converges.
//!
//! The pin is HERMETIC and CROSS-PROCESS, in the self-re-exec shape of
//! `cerulion_core/tests/cdylib_iox2_log_level_test.rs`: the PARENT mints a
//! unique iceoryx2 root + prefix, spawns THIS binary as a child on that
//! config, and the child runs the REAL C-ABI cross-thread leak sequence
//! (crib of `rmw_borrow_publish_test`'s
//! `destroying_with_a_cross_thread_armed_window_leaks_the_slot_never_releases_it`)
//! and then `process::exit(0)`s WITHOUT dropping the transport singleton —
//! which is exactly what makes its node DEAD for the parent. The parent
//! asserts the child really ran the arm (its node directory holds a
//! `<prefix>*.port_tag`), then runs the sweep the way `cerulion clean` does
//! (`Node::try_cleanup_dead_nodes`, `cerulion_cli_engine/src/ipc_cleanup.rs`)
//! and requires `cleanups == 1 && failed_cleanups == 0` with the node
//! directory gone.
//!
//! Two independent oracles test the same failure: dropping the
//! `PublisherData` at destroy instead of tracking the leak: the child's
//! in-process port count after destroy (`topic_publisher_count_checked ==
//! Some(1)`; that bug reads `Some(0)`), and the parent's sweep verdict
//! (that bug reads `failed_cleanups == 1` with the orphan tag still on
//! disk). The child REPORTS its count instead of asserting it, so one run
//! shows both oracles flip in the parent's single violation list.
//!
//! The child's rmw runtime MUST land on the isolated root: it initialises
//! the transport singleton itself via `TransportManager::init_with_config`
//! (the multi-process worker seam — the singleton path `runtime::runtime()`
//! then adopts), and asserts `Arc::ptr_eq` against what `runtime()` hands
//! back. Without that the leak would land on the machine's REAL `/tmp/iceoryx2`
//! — and the test itself would mint the orphan it exists to kill.
//!
//! ⚠️ Run with `--test-threads=1` (the child is a full rmw runtime on
//! iceoryx2 shared memory; the parent's root is unique per test but the
//! process-global fake hook + the rmw singleton are not):
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_leak_at_destroy_registry_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::collections::HashMap;
use std::ffi::CString;
use std::io::{Read, Write};
use std::os::raw::{c_char, c_void};
use std::path::{Path as StdPath, PathBuf};
use std::sync::{Mutex, OnceLock};
use std::thread::ThreadId;
use std::time::{Duration, Instant};

use cerulion_core::transport::{TransportConfig, TransportManager};
use iceoryx2::config::Config;
use iceoryx2::node::Node;
use iceoryx2::prelude::{FileName, Path as IoxPath, SemanticString};
use iceoryx2::service::ipc_threadsafe::Service as CerService;
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::heaphook::{
    HookApi, TestHookGuard, RC_ERR_ALREADY_ARMED, RC_ERR_NOT_ARMED, RC_OK,
};
use rmw_cerulion::*;

/// Set to "1" ONLY on the spawned child, so a bare `-- --ignored` run no-ops.
const ENV_CHILD: &str = "CER_LEAK_REGISTRY_CHILD";
/// The isolated iceoryx2 root directory (absolute, trailing slash).
const ENV_ROOT: &str = "CER_LEAK_REGISTRY_IOX2_ROOT";
/// The isolated iceoryx2 file prefix.
const ENV_PREFIX: &str = "CER_LEAK_REGISTRY_IOX2_PREFIX";
const CHILD_TEST: &str = "subprocess_child_leak_at_destroy";
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);
/// Child → parent probes (stdout, one per line).
const PROBE_LEAK_DELTA: &str = "LEAK_COUNT_DELTA=";
/// The child's `topic_publisher_count_checked` after destroy; `-1` = `None`.
const PROBE_COUNT_AFTER_DESTROY: &str = "PUBLISHER_COUNT_AFTER_DESTROY=";

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

// =====================================================================
// The isolated iceoryx2 config: root + prefix travel by env var so parent
// and child rebuild byte-identical `Config`s.
// =====================================================================

fn isolated_config(root: &str, prefix: &str) -> Config {
    let mut cfg = Config::default();
    cfg.global
        .set_root_path(&IoxPath::new(root.as_bytes()).expect("iceoryx2 root path"));
    cfg.global.prefix = FileName::new(prefix.as_bytes()).expect("iceoryx2 prefix");
    cfg
}

/// The parent's unique root; `Drop` removes it so a FAILING run leaves
/// nothing behind (a passing run's sweep has already emptied `nodes/`).
struct IsolatedRoot {
    dir: PathBuf,
    root: String,
    prefix: String,
}

impl IsolatedRoot {
    fn mint() -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .expect("clock")
            .as_nanos();
        let dir = PathBuf::from(format!("/tmp/iceoryx2/{}_{}", std::process::id(), nanos));
        std::fs::create_dir_all(&dir).expect("create the isolated iceoryx2 root");
        let root = format!("{}/", dir.display());
        let prefix = format!("{}_", std::process::id());
        Self { dir, root, prefix }
    }

    fn nodes_dir(&self) -> PathBuf {
        self.dir.join("nodes")
    }
}

impl Drop for IsolatedRoot {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

// =====================================================================
// The FAKE heap hook (minimal crib of `rmw_borrow_publish_test`): the
// window contract as plain per-thread state, enough to ARM a window on a
// spawned thread so the destroy takes its cross-thread leak arm.
// =====================================================================

/// A thread's armed window: `[base, cursor)` is what the range test adopts.
/// (The crib's `limit` + bump allocator are not needed here — nothing fills
/// the borrowed message; the window only has to be ARMED on another thread.)
#[derive(Clone, Copy)]
struct FakeWindow {
    base: usize,
    cursor: usize,
}

#[derive(Default)]
struct FakeState {
    windows: HashMap<ThreadId, FakeWindow>,
}

fn fake() -> &'static Mutex<FakeState> {
    static FAKE: OnceLock<Mutex<FakeState>> = OnceLock::new();
    FAKE.get_or_init(|| Mutex::new(FakeState::default()))
}

unsafe extern "C" fn f_arm(base: *mut c_void, _limit: *mut c_void) -> i32 {
    let me = std::thread::current().id();
    let mut s = fake().lock().expect("fake");
    if s.windows.contains_key(&me) {
        return RC_ERR_ALREADY_ARMED;
    }
    s.windows.insert(
        me,
        FakeWindow {
            base: base as usize,
            cursor: base as usize,
        },
    );
    RC_OK
}

unsafe extern "C" fn f_disarm() -> i32 {
    let me = std::thread::current().id();
    match fake().lock().expect("fake").windows.remove(&me) {
        Some(_) => 0,
        None => RC_ERR_NOT_ARMED,
    }
}

unsafe extern "C" fn f_escape() -> i32 {
    let me = std::thread::current().id();
    if fake().lock().expect("fake").windows.contains_key(&me) {
        0
    } else {
        RC_ERR_NOT_ARMED
    }
}

unsafe extern "C" fn f_range(ptr: *const c_void, len: usize) -> i32 {
    let me = std::thread::current().id();
    match fake().lock().expect("fake").windows.get(&me) {
        Some(w) => {
            let p = ptr as usize;
            i32::from(p >= w.base && p + len <= w.cursor)
        }
        None => RC_ERR_NOT_ARMED,
    }
}

/// The fake hook: the window entries are the real fake; the segment
/// registry entries spread from
/// `HookApi::inert()`'s inert `RC_OK` stubs.
fn fake_hook_api() -> HookApi {
    HookApi {
        arm_window: f_arm,
        disarm_window: f_disarm,
        window_escape: f_escape,
        window_range_test: f_range,
        ..HookApi::inert()
    }
}

// =====================================================================
// The unbounded fixture — LaserScan-shaped (crib of the borrow tests);
// an UNBOUNDED primitive sequence is what qualifies the type for the
// windowed borrow.
// =====================================================================

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
        {
            let mut m = ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("angle_min"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: 0,
                ..Default::default()
            };
            m.is_array_ = false;
            m
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
            message_namespace_: cstr("leak_registry__msg"),
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

fn default_qos() -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 8,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

// =====================================================================
// THE CHILD: the real C-ABI cross-thread leak sequence on the isolated
// root, then exit WITHOUT dropping the transport singleton.
// =====================================================================

/// `#[ignore]`d: only the parent spawns it (with `ENV_CHILD=1`); a bare
/// `-- --ignored` run returns at the env check.
// P12 exemption, scoped to this fn rather than the file (the `barrier_test.rs`
// `child_worker` precedent): this is the body of a SELF-RE-EXEC CHILD process —
// a process entrypoint by construction — and exiting WITHOUT dropping the
// transport singleton is the whole point of the pin (that is what leaves the
// node DEAD with its ports live for the parent's sweep). The ban stays armed
// for every other line in this binary.
#[allow(clippy::disallowed_methods)]
#[test]
#[ignore]
fn subprocess_child_leak_at_destroy() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let root = std::env::var(ENV_ROOT).expect("the parent sets the isolated root");
    let prefix = std::env::var(ENV_PREFIX).expect("the parent sets the isolated prefix");

    // The singleton FIRST, on the isolated config — `runtime::runtime()`'s
    // own `TransportManager::init` then adopts it (the config is honored
    // only on the first init; the `.or_else(get)` arm is the same handle).
    let manager = TransportManager::init_with_config(
        TransportConfig {
            node_name: format!("rmw_cerulion_{}", std::process::id()),
            ..Default::default()
        },
        isolated_config(&root, &prefix),
    )
    .expect("the child initialises the transport singleton on the isolated root");

    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        // rmw_init + node (crib of `setup_node`).
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr("leak_child"), cstr("/"));
        assert!(!node.is_null(), "node creation failed");

        // HERMETICITY: the rmw runtime must be on OUR isolated manager, not
        // a default-root singleton — else the leak lands on the real
        // `/tmp/iceoryx2` (and this test would mint the very orphan it
        // exists to kill).
        let rt = rmw_cerulion::runtime::runtime().expect("rmw runtime");
        assert!(
            std::sync::Arc::ptr_eq(&rt.transport, &manager),
            "the rmw runtime did not adopt the isolated transport singleton"
        );

        // One pub + sub pair on a unique topic (crib of `setup_pair`).
        let ts = scanish_ts("ScanLeak1579");
        let ros_topic = format!("/leak_registry/{}", std::process::id());
        let topic = CString::new(ros_topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let cerulion_topic = rmw_cerulion::runtime::ros_topic_to_cerulion(&ros_topic)
            .expect("a fully-qualified ROS name maps verbatim");
        assert_eq!(
            manager.topic_publisher_count_checked(&cerulion_topic),
            Some(1),
            "precondition: the live publisher's port is registered"
        );

        // Borrow on a SPAWNED thread T: T's window arms over the slot.
        let ts_addr = ts as usize;
        let pub_addr = publisher as usize;
        let msg_addr = std::thread::spawn(move || {
            let mut msg: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(
                    pub_addr as *const ffi::rmw_publisher_t,
                    ts_addr as *const ffi::rosidl_message_type_support_t,
                    &mut msg,
                ),
                RMW_RET_OK
            );
            msg as usize
        })
        .join()
        .expect("borrow thread");
        assert_eq!(
            fake().lock().expect("fake").windows.len(),
            1,
            "T's window is armed"
        );
        // Wrong-thread RETURN from main ⇒ the slot is orphan-HELD (T's
        // window cannot be disarmed from here).
        assert_eq!(
            rmw_return_loaned_message_from_publisher(publisher, msg_addr as *mut c_void),
            RMW_RET_OK
        );
        // DESTROY on main: the cross-thread-armed slot is LEAKED (counted).
        let before = rmw_cerulion::borrow_destroy_leak_count();
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        let delta = rmw_cerulion::borrow_destroy_leak_count() - before;
        // PRECONDITION (holds under that bug too): the leak arm really ran.
        assert_eq!(
            delta, 1,
            "the cross-thread-armed slot is leaked, never released"
        );

        // ORACLE 1, REPORTED not asserted (so the parent lists every flipped
        // oracle from one run at once): the leaked publisher's port must
        // STAY registered — `Some(1)`. Dropping the
        // `PublisherData` reads `Some(0)`.
        let count = manager
            .topic_publisher_count_checked(&cerulion_topic)
            .map_or(-1i64, i64::from);
        let mut out = std::io::stdout().lock();
        // A leading newline: libtest has already printed `test <name> ... `
        // on this line without a newline.
        writeln!(out).expect("probe");
        writeln!(out, "{PROBE_LEAK_DELTA}{delta}").expect("probe");
        writeln!(out, "{PROBE_COUNT_AFTER_DESTROY}{count}").expect("probe");
        out.flush().expect("flush probes");
    }
    // Exit WITHOUT dropping the singleton, the hook guard, the node or the
    // subscription: the process simply dies with its ports live, which is
    // the ordinary shape every other rmw test binary leaves behind — and
    // what makes this node DEAD for the parent's sweep.
    std::process::exit(0);
}

// =====================================================================
// THE PARENT
// =====================================================================

struct ChildRun {
    status: std::process::ExitStatus,
    stdout: String,
    stderr: String,
}

fn run_child(root: &IsolatedRoot) -> ChildRun {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = std::process::Command::new(exe);
    cmd.args([
        "--exact",
        CHILD_TEST,
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(ENV_CHILD, "1")
    .env(ENV_ROOT, &root.root)
    .env(ENV_PREFIX, &root.prefix)
    .stdin(std::process::Stdio::null())
    .stdout(std::process::Stdio::piped())
    .stderr(std::process::Stdio::piped());
    let mut child = cmd.spawn().expect("spawn child");
    let mut out = child.stdout.take().expect("child stdout");
    let mut err = child.stderr.take().expect("child stderr");
    let drain_out = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = out.read_to_string(&mut buf);
        buf
    });
    let drain_err = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = err.read_to_string(&mut buf);
        buf
    });
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if Instant::now() > deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("leak child did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    };
    ChildRun {
        status,
        stdout: drain_out.join().expect("stdout drain thread"),
        stderr: drain_err.join().expect("stderr drain thread"),
    }
}

/// Find `<prefix><value>` ANYWHERE in a stdout line: libtest prints
/// `test <name> ... ` WITHOUT a newline before the test body runs, so the
/// child's first probe lands on that same line (measured — the first
/// anchored-at-line-start version of this parser missed it).
fn probe<T: std::str::FromStr>(stdout: &str, prefix: &str) -> Option<T> {
    stdout
        .lines()
        .find_map(|l| l.find(prefix).map(|at| &l[at + prefix.len()..]))
        .and_then(|v| v.split_whitespace().next()?.parse::<T>().ok())
}

fn list_dir(dir: &StdPath) -> Vec<PathBuf> {
    match std::fs::read_dir(dir) {
        Ok(entries) => entries.filter_map(|e| e.ok().map(|e| e.path())).collect(),
        Err(_) => Vec::new(),
    }
}

fn port_tags(node_dir: &StdPath, prefix: &str) -> Vec<PathBuf> {
    let mut tags: Vec<PathBuf> = list_dir(node_dir)
        .into_iter()
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with(prefix) && n.ends_with(".port_tag"))
        })
        .collect();
    tags.sort();
    tags
}

/// THE PIN: reverting the `leaked_slots > 0`
/// branch in `rmw_destroy_publisher` to `drop(data)` makes the child report
/// `PUBLISHER_COUNT_AFTER_DESTROY=0`, the sweep report
/// `failed_cleanups == 1`, and the orphan `.port_tag` plus the node
/// directory survive the sweep — all four listed by ONE failure.
#[test]
#[serial]
fn a_leaking_destroy_keeps_the_port_registered_so_the_dead_node_sweep_converges() {
    let root = IsolatedRoot::mint();
    let run = run_child(&root);
    assert!(
        run.status.success(),
        "the leak child failed ({:?});\n--- stdout ---\n{}\n--- stderr ---\n{}",
        run.status,
        run.stdout,
        run.stderr
    );
    let leak_delta: u64 = probe(&run.stdout, PROBE_LEAK_DELTA).unwrap_or_else(|| {
        panic!(
            "the child never printed its leak probe;\n--- stdout ---\n{}\n--- stderr ---\n{}",
            run.stdout, run.stderr
        )
    });
    assert_eq!(
        leak_delta, 1,
        "precondition: the child ran the cross-thread leak arm"
    );
    let count_after_destroy: i64 = probe(&run.stdout, PROBE_COUNT_AFTER_DESTROY)
        .unwrap_or_else(|| panic!("the child never printed its port-count probe;\n--- stdout ---\n{}\n--- stderr ---\n{}", run.stdout, run.stderr));

    // PRECONDITION: the child's node directory exists under OUR root and
    // holds at least one `<prefix>*.port_tag` — the child ran the arm on
    // the isolated config, and its ports outlived it. (This holds under
    // that bug too; what differs is whether the sweep can remove them.)
    let node_dirs: Vec<PathBuf> = list_dir(&root.nodes_dir())
        .into_iter()
        .filter(|p| p.is_dir())
        .collect();
    assert_eq!(
        node_dirs.len(),
        1,
        "exactly one node directory under the isolated root, found {node_dirs:?};\n--- child stderr ---\n{}",
        run.stderr
    );
    let node_dir = node_dirs[0].clone();
    let tags_before = port_tags(&node_dir, &root.prefix);
    assert!(
        !tags_before.is_empty(),
        "the dead node's directory carries no `{}*.port_tag` — the child's ports did not \
         outlive it, so this run pins nothing; contents: {:?}",
        root.prefix,
        list_dir(&node_dir)
    );

    // THE SWEEP — exactly what `cerulion clean` and the exit
    // hygiene run (`ipc_cleanup.rs`), on the isolated config.
    let cfg = isolated_config(&root.root, &root.prefix);
    let state = Node::<CerService>::try_cleanup_dead_nodes(&cfg);

    let tags_after = port_tags(&node_dir, &root.prefix);
    let mut violations: Vec<String> = Vec::new();
    if count_after_destroy != 1 {
        violations.push(format!(
            "ORACLE 1 (in-process): the leaked publisher's port count after destroy is \
             {count_after_destroy}, expected 1 — the port was DEREGISTERED while its \
             leaked samples keep its tag alive"
        ));
    }
    if (state.cleanups, state.failed_cleanups) != (1, 0) {
        violations.push(format!(
            "ORACLE 2 (cross-process): the dead-node sweep reported cleanups={} \
             failed_cleanups={}, expected (1, 0)",
            state.cleanups, state.failed_cleanups
        ));
    }
    if !tags_after.is_empty() {
        violations.push(format!(
            "ORACLE 2: orphan port tag(s) survived the sweep: {tags_after:?}"
        ));
    }
    if node_dir.exists() {
        violations.push(format!(
            "ORACLE 2: the dead node's directory survived the sweep: {}",
            node_dir.display()
        ));
    }
    assert!(
        violations.is_empty(),
        "leak-at-destroy contract violated ({} oracle(s)):\n  - {}\n--- child stderr ---\n{}",
        violations.len(),
        violations.join("\n  - "),
        run.stderr
    );
}

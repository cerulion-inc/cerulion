// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! `rmw_get_topic_names_and_types` lists the topics of other processes on the
//! same host, with their exact types, through the rmw graph registry.
//!
//! Every arm runs its rmw processes as children of this test binary (the
//! `#[ignore]`d `subprocess_child_rmw_process`, inert without its environment),
//! all on one isolated iceoryx2 root that the arm owns and removes at its end.
//! The test process itself opens no transport. Each child obeys one command per
//! stdin line (create or destroy an endpoint, list, gather, exit) and answers
//! each with one `XPROC_REPLY` line, so the arms run in lockstep with no sleep.
//! Every listing is compared with a hand-written string.
//!
//! The children share the machine's iceoryx2 and spawn processes, so run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_graph_xproc_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::io::{BufRead, BufReader, Write};
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use cerulion_core::testing::{
    child_iceoryx_config, IsolatedRoot, CHILD_IOX2_PREFIX_ENV, CHILD_IOX2_ROOT_ENV,
};
use cerulion_core::transport::rmw_graph_registry::GatherCompleteness;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

/// Set to "1" only on a spawned child, so a bare `-- --ignored` run does nothing.
const ENV_CHILD: &str = "CER_RMW_GRAPH_XPROC_CHILD";
const CHILD_TEST: &str = "subprocess_child_rmw_process";
/// The longest a child may take to answer one command before the parent kills
/// it and fails. It turns a wedged child into a failed test, not a hung one.
const REPLY_DEADLINE: Duration = Duration::from_secs(60);
const REPLY: &str = "XPROC_REPLY ";
/// The window of the child's `gather` command: long enough that a writer the
/// gather still waits for shows as `Incomplete` rather than a lucky `Settled`.
const GATHER_WINDOW: Duration = Duration::from_secs(5);

const ROS_TYPE_DOUBLE: u8 = 2;
const RMW_RET_INVALID_ARGUMENT: i32 = 11;
const RMW_RET_BAD_ALLOC: i32 = 10;

// =====================================================================
// Type support: `rmw_gx__msg/<Name>` is the ROS graph type `rmw_gx/msg/<Name>`.
// =====================================================================

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn point_ts(name: &str) -> *const ffi::rosidl_message_type_support_t {
    let members: Vec<_> = ["x", "y", "z"]
        .iter()
        .zip([0u32, 8, 16])
        .map(
            |(field, offset)| ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr(field),
                type_id_: ROS_TYPE_DOUBLE,
                offset_: offset,
                ..Default::default()
            },
        )
        .collect();
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_gx__msg"),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: 3 * std::mem::size_of::<f64>(),
            members_: members.as_ptr(),
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
// Allocators: a plain malloc one, and a countdown one that fails after a
// budget and keeps a ledger of every pointer it handed out and freed.
// =====================================================================

extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

unsafe extern "C" fn malloc_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    malloc(size)
}

unsafe extern "C" fn malloc_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}

fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(malloc_allocate);
    a.deallocate = Some(malloc_deallocate);
    a
}

/// The callbacks never panic (an unwind out of an `extern "C"` fn aborts);
/// the child reads the ledger after the call.
#[derive(Default)]
struct FaultAlloc {
    remaining: AtomicUsize,
    allocated: Mutex<Vec<usize>>,
    freed: Mutex<Vec<usize>>,
}

impl FaultAlloc {
    fn with_budget(n: usize) -> Self {
        Self {
            remaining: AtomicUsize::new(n),
            ..Default::default()
        }
    }

    /// Every pointer handed out was freed exactly once, and nothing else was.
    fn balanced(&self) -> bool {
        let mut allocated = self.allocated.lock().expect("ledger").clone();
        let mut freed = self.freed.lock().expect("ledger").clone();
        allocated.sort_unstable();
        freed.sort_unstable();
        allocated == freed
    }
}

unsafe extern "C" fn fault_allocate(size: usize, state: *mut c_void) -> *mut c_void {
    let st = &*(state as *const FaultAlloc);
    if st
        .remaining
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_sub(1))
        .is_err()
    {
        return std::ptr::null_mut();
    }
    let p = malloc(size);
    if !p.is_null() {
        st.allocated
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(p as usize);
    }
    p
}

unsafe extern "C" fn fault_deallocate(ptr: *mut c_void, state: *mut c_void) {
    let st = &*(state as *const FaultAlloc);
    let mut freed = st.freed.lock().unwrap_or_else(|e| e.into_inner());
    // Record first and free at most once, so a double free shows in the
    // ledger instead of corrupting the heap.
    let again = freed.contains(&(ptr as usize));
    freed.push(ptr as usize);
    if !again {
        free(ptr);
    }
}

fn fault_allocator(state: &FaultAlloc) -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fault_allocate);
    a.deallocate = Some(fault_deallocate);
    a.state = state as *const FaultAlloc as *mut c_void;
    a
}

/// Read a filled `rmw_names_and_types_t` as `(name, types)` rows in the order
/// the C ABI returned them, then free every allocation through `allocator`.
unsafe fn take_names_and_types(
    nat: &mut ffi::rmw_names_and_types_t,
    allocator: &ffi::rcutils_allocator_t,
) -> Vec<(String, Vec<String>)> {
    let dealloc = allocator.deallocate.expect("deallocate");
    let mut rows = Vec::with_capacity(nat.names.size);
    for i in 0..nat.names.size {
        let name = *nat.names.data.add(i);
        let types = &mut *nat.types.add(i);
        let mut type_names = Vec::with_capacity(types.size);
        for k in 0..types.size {
            let t = *types.data.add(k);
            type_names.push(CStr::from_ptr(t).to_str().expect("utf-8").to_string());
            dealloc(t as *mut c_void, allocator.state);
        }
        if !types.data.is_null() {
            dealloc(types.data as *mut c_void, allocator.state);
        }
        rows.push((
            CStr::from_ptr(name).to_str().expect("utf-8").to_string(),
            type_names,
        ));
        dealloc(name as *mut c_void, allocator.state);
    }
    if !nat.types.is_null() {
        dealloc(nat.types as *mut c_void, allocator.state);
    }
    if !nat.names.data.is_null() {
        dealloc(nat.names.data as *mut c_void, allocator.state);
    }
    rows
}

/// `topic=type,type;topic=type` over the rows whose name starts with `prefix`.
fn render(rows: &[(String, Vec<String>)], prefix: &str) -> String {
    rows.iter()
        .filter(|(name, _)| name.starts_with(prefix))
        .map(|(name, types)| format!("{name}={}", types.join(",")))
        .collect::<Vec<_>>()
        .join(";")
}

// =====================================================================
// THE CHILD: one rmw process on the arm's isolated root.
// =====================================================================

struct ChildState {
    node: *mut ffi::rmw_node_t,
    type_supports: HashMap<String, usize>,
    publishers: HashMap<String, *mut ffi::rmw_publisher_t>,
    subscriptions: HashMap<String, *mut ffi::rmw_subscription_t>,
    natives: Vec<cerulion_core::transport::publisher::CerulionPublisher>,
}

impl ChildState {
    fn ts(&mut self, name: &str) -> *const ffi::rosidl_message_type_support_t {
        *self
            .type_supports
            .entry(name.to_string())
            .or_insert_with(|| point_ts(name) as usize) as *const _
    }

    unsafe fn list(&self) -> Vec<(String, Vec<String>)> {
        let mut allocator = malloc_allocator();
        let mut nat: ffi::rmw_names_and_types_t = std::mem::zeroed();
        let ret = rmw_get_topic_names_and_types(self.node, &mut allocator, false, &mut nat);
        assert_eq!(ret, RMW_RET_OK, "rmw_get_topic_names_and_types");
        take_names_and_types(&mut nat, &allocator)
    }

    /// One command line in, one reply payload out.
    unsafe fn run(&mut self, line: &str) -> String {
        let words: Vec<&str> = line.split_whitespace().collect();
        let qos = default_qos();
        match words.as_slice() {
            ["pub", topic, type_name] => {
                let ts = self.ts(type_name);
                let topic_c = CString::new(*topic).expect("topic");
                let opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
                let p = rmw_create_publisher(self.node, ts, topic_c.as_ptr(), &qos, &opts);
                assert!(!p.is_null(), "create publisher on {topic}");
                self.publishers.insert(topic.to_string(), p);
                "ok".into()
            }
            ["sub", topic, type_name] => {
                let ts = self.ts(type_name);
                let topic_c = CString::new(*topic).expect("topic");
                let opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
                let s = rmw_create_subscription(self.node, ts, topic_c.as_ptr(), &qos, &opts);
                assert!(!s.is_null(), "create subscription on {topic}");
                self.subscriptions.insert(topic.to_string(), s);
                "ok".into()
            }
            ["unpub", topic] => {
                let p = self
                    .publishers
                    .remove(*topic)
                    .expect("a publisher to destroy");
                assert_eq!(rmw_destroy_publisher(self.node, p), RMW_RET_OK);
                "ok".into()
            }
            ["unsub", topic] => {
                let s = self
                    .subscriptions
                    .remove(*topic)
                    .expect("a subscription to destroy");
                assert_eq!(rmw_destroy_subscription(self.node, s), RMW_RET_OK);
                "ok".into()
            }
            ["native", topic] => {
                let rt = rmw_cerulion::runtime::runtime().expect("runtime");
                let p = rt
                    .transport
                    .create_publisher(topic, MaxSliceLen::const_new(4096), 0)
                    .expect("native publisher");
                self.natives.push(p);
                "ok".into()
            }
            ["list", prefix] => render(&self.list(), prefix),
            ["gather", prefix] => {
                let rt = rmw_cerulion::runtime::runtime().expect("runtime");
                let own = rt.graph_registry.as_ref().and_then(|w| w.writer_id());
                let gather = rt
                    .transport
                    .gather_rmw_graph(own, GATHER_WINDOW)
                    .expect("gather");
                let verdict = match gather.completeness {
                    GatherCompleteness::Settled => "settled".to_string(),
                    GatherCompleteness::Incomplete {
                        live_writers,
                        writers_heard,
                    } => format!("incomplete:{live_writers}:{writers_heard}"),
                };
                let rows: Vec<(String, Vec<String>)> = gather
                    .topics
                    .into_iter()
                    .map(|(t, types)| (t, types.into_iter().collect()))
                    .collect();
                format!("{verdict} {}", render(&rows, prefix))
            }
            ["nulls"] => {
                let mut allocator = malloc_allocator();
                let mut nat: ffi::rmw_names_and_types_t = std::mem::zeroed();
                let codes = [
                    rmw_get_topic_names_and_types(
                        std::ptr::null(),
                        &mut allocator,
                        false,
                        &mut nat,
                    ),
                    rmw_get_topic_names_and_types(self.node, std::ptr::null_mut(), false, &mut nat),
                    rmw_get_topic_names_and_types(
                        self.node,
                        &mut allocator,
                        false,
                        std::ptr::null_mut(),
                    ),
                ];
                codes.map(|c| c.to_string()).join(",")
            }
            ["faults"] => self.fault_walk(),
            other => panic!("unknown child command {other:?}"),
        }
    }

    /// Fail the allocator at the first, second, ... allocation until a call
    /// succeeds. One `budget:ret:state` row per call, where `state` is `clean`
    /// when the ledger balances and (for a failed call) the out-param is empty.
    unsafe fn fault_walk(&self) -> String {
        let mut rows = Vec::new();
        for budget in 0..64 {
            let state = FaultAlloc::with_budget(budget);
            let mut allocator = fault_allocator(&state);
            let mut nat: ffi::rmw_names_and_types_t = std::mem::zeroed();
            let ret = rmw_get_topic_names_and_types(self.node, &mut allocator, false, &mut nat);
            let empty = nat.names.size == 0 && nat.names.data.is_null() && nat.types.is_null();
            if ret == RMW_RET_OK {
                take_names_and_types(&mut nat, &allocator);
            }
            let clean = state.balanced() && (ret == RMW_RET_OK || empty);
            rows.push(format!(
                "{budget}:{ret}:{}",
                if clean { "clean" } else { "dirty" }
            ));
            if ret == RMW_RET_OK {
                break;
            }
        }
        rows.join(";")
    }
}

/// `#[ignore]`d: only an arm spawns it, with `ENV_CHILD=1` and the root.
// `process::exit` is the point of the `exit` command: the process dies with
// every endpoint live and nothing dropped, as a ROS process does when it ends.
#[allow(clippy::disallowed_methods)]
#[test]
#[ignore]
fn subprocess_child_rmw_process() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let config = child_iceoryx_config().expect("the parent sets the isolated root");
    // The singleton first, on the isolated root; `runtime()` then adopts it.
    let manager = TransportManager::init_with_config(
        TransportConfig {
            node_name: format!("rmw_cerulion_{}", std::process::id()),
            ..Default::default()
        },
        config,
    )
    .expect("the child initialises the transport on the isolated root");
    let mut out = std::io::stdout().lock();
    unsafe {
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr("graph_xproc_child"), cstr("/"));
        assert!(!node.is_null(), "node");
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        assert!(
            std::sync::Arc::ptr_eq(&rt.transport, &manager),
            "the rmw runtime did not adopt the isolated transport"
        );
        let mut state = ChildState {
            node,
            type_supports: HashMap::new(),
            publishers: HashMap::new(),
            subscriptions: HashMap::new(),
            natives: Vec::new(),
        };
        // A leading newline: libtest has printed `test <name> ... ` already.
        writeln!(out).expect("reply");
        writeln!(out, "{REPLY}ready").expect("reply");
        out.flush().expect("flush");
        for line in std::io::stdin().lock().lines() {
            let line = line.expect("stdin");
            if line.trim() == "exit" {
                std::process::exit(0);
            }
            let reply = state.run(&line);
            writeln!(out, "{REPLY}{reply}").expect("reply");
            out.flush().expect("flush");
        }
    }
    std::process::exit(0);
}

// =====================================================================
// THE PARENT
// =====================================================================

/// A child rmw process. Dropping it kills and reaps a child that still runs.
struct Child {
    name: &'static str,
    process: std::process::Child,
    stdin: Option<std::process::ChildStdin>,
    replies: Receiver<String>,
}

impl Child {
    fn spawn(root: &IsolatedRoot, name: &'static str) -> Child {
        let exe = std::env::current_exe().expect("current_exe");
        let mut process = std::process::Command::new(exe)
            .args([
                "--exact",
                CHILD_TEST,
                "--ignored",
                "--nocapture",
                "--test-threads=1",
            ])
            .env(ENV_CHILD, "1")
            .env(CHILD_IOX2_ROOT_ENV, &root.root)
            .env(CHILD_IOX2_PREFIX_ENV, &root.prefix)
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::inherit())
            .spawn()
            .expect("spawn child");
        let stdin = process.stdin.take();
        let stdout = process.stdout.take().expect("child stdout");
        let (tx, replies) = std::sync::mpsc::channel();
        // Drain stdout to its end even after the parent stops reading, so the
        // child never fails a write on a closed pipe.
        std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                let Ok(line) = line else { break };
                if let Some(at) = line.find(REPLY) {
                    let _ = tx.send(line[at + REPLY.len()..].to_string());
                }
            }
        });
        let mut child = Child {
            name,
            process,
            stdin,
            replies,
        };
        assert_eq!(child.reply("start"), "ready");
        child
    }

    fn reply(&mut self, what: &str) -> String {
        match self.replies.recv_timeout(REPLY_DEADLINE) {
            Ok(reply) => reply,
            Err(RecvTimeoutError::Timeout) => {
                panic!(
                    "child {} gave no reply to `{what}` in {REPLY_DEADLINE:?}",
                    self.name
                )
            }
            Err(RecvTimeoutError::Disconnected) => {
                panic!("child {} exited before it replied to `{what}`", self.name)
            }
        }
    }

    fn ask(&mut self, command: &str) -> String {
        let stdin = self.stdin.as_mut().expect("child stdin");
        writeln!(stdin, "{command}").expect("write a command");
        stdin.flush().expect("flush a command");
        self.reply(command)
    }

    /// End the child with `process::exit(0)`, every endpoint still live, and
    /// wait until it is gone.
    fn exit_without_destroying(mut self) {
        let stdin = self.stdin.as_mut().expect("child stdin");
        writeln!(stdin, "exit").expect("write exit");
        stdin.flush().expect("flush exit");
        let deadline = Instant::now() + REPLY_DEADLINE;
        loop {
            match self.process.try_wait().expect("try_wait") {
                Some(status) => {
                    assert!(status.success(), "child {} exited with {status}", self.name);
                    return;
                }
                None if Instant::now() > deadline => {
                    panic!("child {} did not exit in {REPLY_DEADLINE:?}", self.name)
                }
                None => std::thread::sleep(Duration::from_millis(10)),
            }
        }
    }
}

impl Drop for Child {
    fn drop(&mut self) {
        drop(self.stdin.take());
        if let Ok(None) = self.process.try_wait() {
            let _ = self.process.kill();
        }
        let _ = self.process.wait();
    }
}

/// A topic prefix no other arm uses.
fn prefix(arm: &str) -> String {
    format!("/rgx_{arm}_{}", std::process::id())
}

#[test]
#[serial]
fn another_process_lists_a_childs_publisher_and_subscription_only_topic_with_exact_types() {
    let root = IsolatedRoot::mint("rgxa");
    let p = prefix("a");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    assert_eq!(
        talker.ask(&format!("pub {p}/joint_states JointState")),
        "ok"
    );
    assert_eq!(talker.ask(&format!("sub {p}/commands Command")), "ok");
    assert_eq!(lister.ask(&format!("pub {p}/local Local")), "ok");
    let expected = format!(
        "{p}/commands=rmw_gx/msg/Command;{p}/joint_states=rmw_gx/msg/JointState;\
         {p}/local=rmw_gx/msg/Local"
    );
    assert_eq!(lister.ask(&format!("list {p}")), expected);
    assert_eq!(talker.ask(&format!("list {p}")), expected);
}

#[test]
#[serial]
fn a_destroyed_endpoint_leaves_the_listing_of_another_process() {
    let root = IsolatedRoot::mint("rgxb");
    let p = prefix("b");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    assert_eq!(
        talker.ask(&format!("pub {p}/joint_states JointState")),
        "ok"
    );
    assert_eq!(talker.ask(&format!("sub {p}/commands Command")), "ok");
    assert_eq!(
        lister.ask(&format!("list {p}")),
        format!("{p}/commands=rmw_gx/msg/Command;{p}/joint_states=rmw_gx/msg/JointState")
    );
    assert_eq!(talker.ask(&format!("unpub {p}/joint_states")), "ok");
    assert_eq!(
        lister.ask(&format!("list {p}")),
        format!("{p}/commands=rmw_gx/msg/Command")
    );
    assert_eq!(talker.ask(&format!("unsub {p}/commands")), "ok");
    assert_eq!(lister.ask(&format!("list {p}")), "");
}

#[test]
#[serial]
fn a_process_that_exits_without_destroying_leaves_the_listing_and_is_not_waited_for() {
    let root = IsolatedRoot::mint("rgxc");
    let p = prefix("c");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    assert_eq!(
        talker.ask(&format!("pub {p}/joint_states JointState")),
        "ok"
    );
    assert_eq!(
        lister.ask(&format!("gather {p}")),
        format!("settled {p}/joint_states=rmw_gx/msg/JointState")
    );
    talker.exit_without_destroying();
    // The exited process's port is still listed on the registry service; its
    // node is dead, so the gather does not wait for it and drops its pairs.
    assert_eq!(lister.ask(&format!("gather {p}")), "settled ");
    assert_eq!(lister.ask(&format!("list {p}")), "");
}

#[test]
#[serial]
fn a_topic_a_native_publisher_created_first_lists_with_the_ros_type() {
    let root = IsolatedRoot::mint("rgxd");
    let p = prefix("d");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    assert_eq!(lister.ask(&format!("native {p}/native_first")), "ok");
    assert_eq!(lister.ask(&format!("list {p}")), "");
    assert_eq!(talker.ask(&format!("pub {p}/native_first Gamma")), "ok");
    assert_eq!(
        lister.ask(&format!("list {p}")),
        format!("{p}/native_first=rmw_gx/msg/Gamma")
    );
}

#[test]
#[serial]
fn two_processes_with_different_types_on_one_topic_list_both_sorted() {
    let root = IsolatedRoot::mint("rgxe");
    let p = prefix("e");
    let mut first = Child::spawn(&root, "first");
    let mut second = Child::spawn(&root, "second");
    assert_eq!(first.ask(&format!("sub {p}/shared Zulu")), "ok");
    assert_eq!(second.ask(&format!("pub {p}/shared Alpha")), "ok");
    let expected = format!("{p}/shared=rmw_gx/msg/Alpha,rmw_gx/msg/Zulu");
    assert_eq!(first.ask(&format!("list {p}")), expected);
    assert_eq!(second.ask(&format!("list {p}")), expected);
}

#[test]
#[serial]
fn two_listings_of_the_same_graph_are_byte_identical_whatever_the_creation_order() {
    let root = IsolatedRoot::mint("rgxf");
    let p = prefix("f");
    let mut lister = Child::spawn(&root, "lister");
    let mut forward = Child::spawn(&root, "forward");
    let mut backward = Child::spawn(&root, "backward");
    for (topic, ty) in [("a", "Alpha"), ("b", "Bravo"), ("c", "Charlie")] {
        assert_eq!(forward.ask(&format!("pub {p}/fwd/{topic} {ty}")), "ok");
    }
    for (topic, ty) in [("c", "Charlie"), ("b", "Bravo"), ("a", "Alpha")] {
        assert_eq!(backward.ask(&format!("pub {p}/bwd/{topic} {ty}")), "ok");
    }
    let expected = format!(
        "{p}/bwd/a=rmw_gx/msg/Alpha;{p}/bwd/b=rmw_gx/msg/Bravo;{p}/bwd/c=rmw_gx/msg/Charlie;\
         {p}/fwd/a=rmw_gx/msg/Alpha;{p}/fwd/b=rmw_gx/msg/Bravo;{p}/fwd/c=rmw_gx/msg/Charlie"
    );
    assert_eq!(lister.ask(&format!("list {p}")), expected);
    assert_eq!(lister.ask(&format!("list {p}")), expected);
}

#[test]
#[serial]
fn an_endpoint_whose_type_name_is_over_the_frame_bound_is_created_but_listed_only_locally() {
    let root = IsolatedRoot::mint("rgxg");
    let p = prefix("g");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    // `rmw_gx/msg/` is 11 bytes, so a 250-byte name makes a 261-byte type name,
    // over the registry's 256-byte bound.
    let long = "L".repeat(250);
    assert_eq!(talker.ask(&format!("pub {p}/long {long}")), "ok");
    assert_eq!(talker.ask(&format!("pub {p}/short Short")), "ok");
    assert_eq!(
        talker.ask(&format!("list {p}")),
        format!("{p}/long=rmw_gx/msg/{long};{p}/short=rmw_gx/msg/Short")
    );
    assert_eq!(
        lister.ask(&format!("list {p}")),
        format!("{p}/short=rmw_gx/msg/Short")
    );
}

#[test]
#[serial]
fn a_listing_with_a_null_argument_is_refused() {
    let root = IsolatedRoot::mint("rgxh");
    let mut lister = Child::spawn(&root, "lister");
    let refused = RMW_RET_INVALID_ARGUMENT;
    assert_eq!(
        lister.ask("nulls"),
        format!("{refused},{refused},{refused}")
    );
}

#[test]
#[serial]
fn a_failed_allocation_at_every_point_frees_everything_and_leaves_the_out_param_empty() {
    let root = IsolatedRoot::mint("rgxi");
    let p = prefix("i");
    let mut lister = Child::spawn(&root, "lister");
    let mut talker = Child::spawn(&root, "talker");
    // One topic with two types, one from each process: the names array (its
    // buffer and one string), the types buffer, and the topic's types array
    // (its buffer and two strings) make six allocations.
    assert_eq!(lister.ask(&format!("sub {p}/mixed Zulu")), "ok");
    assert_eq!(talker.ask(&format!("pub {p}/mixed Alpha")), "ok");
    let bad = RMW_RET_BAD_ALLOC;
    assert_eq!(
        lister.ask("faults"),
        format!(
            "0:{bad}:clean;1:{bad}:clean;2:{bad}:clean;3:{bad}:clean;4:{bad}:clean;\
             5:{bad}:clean;6:0:clean"
        )
    );
}

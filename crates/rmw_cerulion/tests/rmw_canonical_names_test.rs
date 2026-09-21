// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! rmw canonical names + network-egress registration — the rmw half of "rmw
//! topics reach networked Cerulion hosts".
//!
//! Two contracts, each pinned against HAND oracles over REAL iceoryx2:
//!
//! 1. **Naming is the identity.** A ROS name is the Cerulion canonical name
//!    VERBATIM — an rmw publisher on ROS `/chatter` is the iceoryx2 service
//!    `/chatter/data`, which a plain `cerulion_core` subscriber opens by that
//!    name and READS (a delivery pin, not a name-string compare); the old
//!    slash-stripped spelling does NOT exist (negative control — no alias);
//!    the ROS-visible name round-trips `rmw_get_topic_names_and_types`
//!    unchanged; a service on `/add_two_ints` likewise, met by a native service
//!    client on the canonical name. A relative name is REFUSED (a null entity
//!    plus a loud `error!`), never silently prefixed — pinned for publisher,
//!    subscription, service, client and the count queries.
//! 2. **Every publisher registers for egress.** After `rmw_create_publisher`
//!    the process's runtime-egress registration set holds the canonical topic
//!    with the bridge's schema hash (a variable type too — its introspection
//!    hash), and the record CROSSES the reserved control channel: a gateway
//!    reader on the same SHM root drains it, stores the hash and announces the
//!    topic. A subscription registers nothing (egress = produced topics only),
//!    and a destroyed publisher's registration OUTLIVES it — the channel is
//!    additive and has no removal primitive; this pin is what keeps that
//!    statement in the destroy path true. And because there is no removal,
//!    the registration runs LAST in `rmw_create_publisher`: a failure injected
//!    after the transport publisher exists (the `test-seams` panic seam)
//!    leaves NO record — no phantom topic for a handle rcl never received.
//!
//! Every test is `#[traced_test]` (the loud-refusal pins read the log), so
//! this is its OWN binary — the macro takes the process-global subscriber
//! slot, the same reason the other traced rmw files are separate — and
//! `#[serial]` (the rmw runtime singleton over the process-global iceoryx2
//! root; the gateway arm opens a SECOND manager on that same root). Every
//! test DESTROYS ITS NODE before returning: the graph-guard registry holds
//! raw pointers that `runtime::notify_graph_change` dereferences on every
//! later graph mutation, so a leaked node is a use-after-free in the next
//! test of the binary (the documented unregister-before-free invariant):
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_canonical_names_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use tracing_test::traced_test;

use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::service::derive_client_guid;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use rmw_cerulion::ffi::{self, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use rmw_cerulion::runtime::{PublisherData, ServiceData};
use rmw_cerulion::*;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs / rmw_endpoint_info_test.rs)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_STRING: u8 = 16;

/// Bounded wait for anything iceoryx2 settles asynchronously in-process
/// (connection establishment, the service directory): 50 × 10 ms.
const SETTLE_POLLS: usize = 50;
const SETTLE_STEP: Duration = Duration::from_millis(10);

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: u32,
) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
    ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        offset_: offset,
        ..Default::default()
    }
}

fn make_message_ts(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr(namespace),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    let ts = ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// A variable-layout message: `{ id: f64, label: string }` — the shape whose
/// wire frames carry an offset table, so its schema hash is the
/// introspection-derived one and NOT a fixed-section byte count.
#[repr(C)]
struct CLabeled {
    id: f64,
    label: CRosString,
}

/// Type "rmw_cn__msg/<unique>" → qualified "rmw_cn/<unique>" → ROS graph
/// type "rmw_cn/msg/<unique>". A 3×f64 POD (the loanable, fixed shape).
fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_cn__msg",
        unique,
        std::mem::size_of::<CPoint>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ],
    )
}

/// The variable shape (see [`CLabeled`]).
fn labeled_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_cn__msg",
        unique,
        std::mem::size_of::<CLabeled>(),
        vec![
            member("id", ROS_TYPE_DOUBLE, 0),
            member(
                "label",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CLabeled, label) as u32,
            ),
        ],
    )
}

/// Service "rmw_cn__srv/<unique>": request `{ a: f64 }`, response
/// `{ sum: f64, msg: string }` → ROS graph type "rmw_cn/srv/<unique>".
fn service_ts(unique: &str) -> *const ffi::rosidl_service_type_support_t {
    let req = make_message_ts(
        "rmw_cn__srv",
        &format!("{unique}_Request"),
        std::mem::size_of::<f64>(),
        vec![member("a", ROS_TYPE_DOUBLE, 0)],
    );
    #[repr(C)]
    struct CResp {
        sum: f64,
        msg: CRosString,
    }
    let resp = make_message_ts(
        "rmw_cn__srv",
        &format!("{unique}_Response"),
        std::mem::size_of::<CResp>(),
        vec![
            member("sum", ROS_TYPE_DOUBLE, 0),
            member(
                "msg",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CResp, msg) as u32,
            ),
        ],
    );
    let req_members =
        unsafe { (*req).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
    let resp_members =
        unsafe { (*resp).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
    let sm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__ServiceMembers {
            service_namespace_: cstr("rmw_cn__srv"),
            service_name_: cstr(unique),
            request_members_: req_members,
            response_members_: resp_members,
            ..Default::default()
        },
    ));
    let ts = ffi::rosidl_service_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: sm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

unsafe fn setup_node(name: &str) -> (*mut ffi::rmw_node_t, Box<ffi::rmw_init_options_t>) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (node, options)
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

// A REAL malloc-backed rcutils_allocator_t — rcl always passes one, and the
// names-and-types fill allocates the arrays + strings through it.
extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}
unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    malloc(size)
}
unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fixture_allocate);
    a.deallocate = Some(fixture_deallocate);
    a
}

unsafe fn read_cstr(p: *const c_char) -> String {
    assert!(!p.is_null(), "a names-and-types string must not be null");
    CStr::from_ptr(p).to_str().expect("utf-8").to_string()
}

/// Read a filled `rmw_names_and_types_t` into owned `(name, first type)`
/// pairs, then free every allocation through the SAME allocator that made
/// it (rcl's `rmw_names_and_types_fini` lives in the rmw common library, not
/// in an implementation, so the test frees by hand).
unsafe fn take_names_and_types(
    nat: &mut ffi::rmw_names_and_types_t,
    allocator: &ffi::rcutils_allocator_t,
) -> Vec<(String, String)> {
    let dealloc = allocator.deallocate.expect("fixture deallocate");
    let mut out = Vec::with_capacity(nat.names.size);
    for i in 0..nat.names.size {
        let name_ptr = *nat.names.data.add(i);
        let types = &mut *nat.types.add(i);
        assert!(types.size >= 1, "every name carries at least one type");
        out.push((read_cstr(name_ptr), read_cstr(*types.data.add(0))));
        for k in 0..types.size {
            dealloc(*types.data.add(k) as *mut c_void, allocator.state);
        }
        dealloc(types.data as *mut c_void, allocator.state);
        dealloc(name_ptr as *mut c_void, allocator.state);
    }
    if !nat.types.is_null() {
        dealloc(nat.types as *mut c_void, allocator.state);
    }
    if !nat.names.data.is_null() {
        dealloc(nat.names.data as *mut c_void, allocator.state);
    }
    nat.names.data = std::ptr::null_mut();
    nat.names.size = 0;
    nat.types = std::ptr::null_mut();
    out
}

/// The ROS-visible topic graph: `rmw_get_topic_names_and_types`.
unsafe fn topic_names_and_types(node: *const ffi::rmw_node_t) -> Vec<(String, String)> {
    let mut allocator = malloc_allocator();
    let mut nat: ffi::rmw_names_and_types_t = std::mem::zeroed();
    assert_eq!(
        rmw_get_topic_names_and_types(node, &mut allocator, false, &mut nat),
        RMW_RET_OK
    );
    take_names_and_types(&mut nat, &allocator)
}

/// The ROS-visible service graph: `rmw_get_service_names_and_types`.
unsafe fn service_names_and_types(node: *const ffi::rmw_node_t) -> Vec<(String, String)> {
    let mut allocator = malloc_allocator();
    let mut nat: ffi::rmw_names_and_types_t = std::mem::zeroed();
    assert_eq!(
        rmw_get_service_names_and_types(node, &mut allocator, &mut nat),
        RMW_RET_OK
    );
    take_names_and_types(&mut nat, &allocator)
}

/// Whether NO iceoryx2 data service exists under `topic` — probed by trying to
/// open one (a `create_subscriber_open_only` on a missing service errs; the
/// probe is dropped at once when it does open).
fn transport_has_no_service(topic: &str) -> bool {
    TransportManager::get()
        .expect("transport")
        .create_subscriber_open_only(topic)
        .is_err()
}

/// A GATEWAY on the process-global DEFAULT iceoryx2 root — the root the rmw
/// singleton lives on — with an AllowAll, empty-announce plan: the runtime
/// registrations arrive over the control channel, never the boot plan. A
/// second manager on the same root models the separate gateway PROCESS a
/// robot runs (crib `cerulion_core/tests/reg_channel_iox2_test.rs`).
fn gateway_on_default_root(tag: &str) -> GatewayRuntime {
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rmw_cn_gw_{tag}"),
            network: Some(NetworkConfig {
                robot_identity: Some("rmw_cn_robot".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        iceoryx2::config::Config::global_config().clone(),
    )
    .expect("gateway manager on the default iceoryx2 root");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    GatewayRuntime::new(manager, plan).expect("gateway boot")
}

/// Deterministically settle the registration channel until `done` holds:
/// republish the worker's whole set inline, drive the gateway, re-check —
/// bounded, no dependence on the background pump's timing.
fn settle_until(
    transport: &TransportManager,
    gateway: &mut GatewayRuntime,
    mut done: impl FnMut(&GatewayRuntime) -> bool,
) -> bool {
    for _ in 0..SETTLE_POLLS {
        transport.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive_once");
        if done(gateway) {
            return true;
        }
        std::thread::sleep(SETTLE_STEP);
    }
    false
}

// =====================================================================
// (1) Naming is the identity
// =====================================================================

/// An rmw publisher on ROS `/rmw_cn/chatter/<s>` IS the canonical Cerulion
/// topic of that name: a plain `cerulion_core` subscriber opens the iceoryx2
/// service by the canonical name and RECEIVES the rmw's frame — the bridge's
/// schema hash in the header and the exact fixed-section bytes of a
/// hand-oracle point — while the slash-stripped spelling opens NOTHING (no
/// alias). The ROS-visible name round-trips the graph query unchanged, and
/// the count query keys by the same name (the stripped one is refused, not
/// counted 0).
#[test]
#[traced_test]
#[serial]
fn a_ros_topic_is_the_cerulion_canonical_name_and_a_native_subscriber_reads_it() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("Pt{suffix}");
        let ts = point_ts(&type_name);
        let (node, _opts) = setup_node(&format!("cn_node_{suffix}"));

        let ros_topic = format!("/rmw_cn/chatter/{suffix}");
        let stripped = format!("rmw_cn/chatter/{suffix}");
        let topic_c = CString::new(ros_topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        let publisher = rmw_create_publisher(node, ts, topic_c.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null(), "a fully-qualified name creates");
        let data = &*((*publisher).data as *const PublisherData);
        assert_eq!(
            data.topic, ros_topic,
            "the Cerulion topic the rmw stores IS the ROS name, verbatim"
        );
        let expected_hash = data.bridge.schema_hash();

        // The native side: a plain cerulion_core subscriber opens the SAME
        // iceoryx2 service (`<topic>/data`) by the canonical name ...
        let transport = TransportManager::get().expect("the rmw runtime's transport");
        let native = transport
            .create_subscriber_open_only(&ros_topic)
            .expect("the canonical name opens the rmw publisher's data service");
        // ... and the slash-stripped spelling opens NOTHING: there is no alias
        // and no second service (the negative control).
        assert!(
            transport.create_subscriber_open_only(&stripped).is_err(),
            "the stripped name `{stripped}` must not exist as a service"
        );

        // DELIVERY: one hand-oracle point; the native subscriber reads the
        // frame with the bridge's schema hash and the exact fixed section.
        let msg = CPoint {
            x: 1.5,
            y: -2.0,
            z: 3.25,
        };
        let mut oracle = Vec::with_capacity(24);
        oracle.extend_from_slice(&1.5f64.to_le_bytes());
        oracle.extend_from_slice(&(-2.0f64).to_le_bytes());
        oracle.extend_from_slice(&3.25f64.to_le_bytes());
        let mut got: Option<(u64, Vec<u8>)> = None;
        for _ in 0..SETTLE_POLLS {
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let mut seen: Option<(u64, Vec<u8>)> = None;
            native
                .try_receive_one(|frame| {
                    seen = Some((frame.header().schema_hash, frame.payload().to_vec()));
                })
                .expect("native receive");
            if seen.is_some() {
                got = seen;
                break;
            }
            std::thread::sleep(SETTLE_STEP);
        }
        let (hash, payload) =
            got.expect("the native subscriber received a frame on the canonical name");
        assert_eq!(
            hash, expected_hash,
            "the frame carries the bridge's schema hash"
        );
        assert!(payload.len() >= oracle.len(), "payload {}", payload.len());
        assert_eq!(&payload[..oracle.len()], &oracle[..], "fixed-section bytes");

        // The ROS-visible name round-trips the graph query UNCHANGED, with
        // the ROS graph type; nothing in the graph is a stripped name.
        let names = topic_names_and_types(node);
        assert!(
            names
                .iter()
                .any(|(n, t)| n == &ros_topic && t == &format!("rmw_cn/msg/{type_name}")),
            "the graph must list {ros_topic} with its ROS type; got {names:?}"
        );
        assert!(
            names.iter().all(|(n, _)| n.starts_with('/')),
            "every ROS-visible topic name is fully qualified: {names:?}"
        );
        assert!(
            !names.iter().any(|(n, _)| n == &stripped),
            "no stripped spelling may leak into the graph: {names:?}"
        );

        // Counts key by the same name; the stripped spelling is REFUSED (the
        // rcl contract for a malformed argument), not silently counted 0.
        let mut count = 0usize;
        assert_eq!(
            rmw_count_publishers(node, topic_c.as_ptr(), &mut count),
            RMW_RET_OK
        );
        assert_eq!(count, 1);
        let stripped_c = CString::new(stripped.clone()).expect("stripped");
        assert_eq!(
            rmw_count_publishers(node, stripped_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );

        drop(native);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A relative ROS name (no leading `/`) cannot arrive through rcl, so one
/// reaching the rmw is a caller bug: every entity create REFUSES it (null),
/// the refusal is LOUD (an `error!` naming the reason), and NOTHING is minted
/// under either spelling — neither the relative name nor a silently-prefixed
/// canonical one exists as a service or in the graph. The count queries
/// refuse the same way. An empty name is refused too.
#[test]
#[traced_test]
#[serial]
fn a_relative_ros_name_is_refused_loudly_and_never_prefixed() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("Rel{suffix}"));
        let sts = service_ts(&format!("RelSvc{suffix}"));
        let (node, _opts) = setup_node(&format!("cn_rel_node_{suffix}"));

        let relative = format!("rmw_cn/relative/{suffix}");
        let prefixed = format!("/{relative}");
        let relative_c = CString::new(relative.clone()).expect("relative");
        let empty_c = CString::new("").expect("empty");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        assert!(
            rmw_create_publisher(node, ts, relative_c.as_ptr(), &qos, &pub_opts).is_null(),
            "a relative topic name must not create a publisher"
        );
        assert!(
            rmw_create_subscription(node, ts, relative_c.as_ptr(), &qos, &sub_opts).is_null(),
            "a relative topic name must not create a subscription"
        );
        assert!(
            rmw_create_service(node, sts, relative_c.as_ptr(), &qos).is_null(),
            "a relative service name must not create a service"
        );
        assert!(
            rmw_create_client(node, sts, relative_c.as_ptr(), &qos).is_null(),
            "a relative service name must not create a client"
        );
        assert!(
            rmw_create_publisher(node, ts, empty_c.as_ptr(), &qos, &pub_opts).is_null(),
            "an empty topic name must not create a publisher"
        );
        // EMPTY SEGMENTS: the transport would accept a lone `/` and mint a
        // malformed `//data` service, so the grammar is enforced HERE — a
        // lone `/`, a trailing `/`, and a `//` inside are all refused (null),
        // for a publisher and for a service alike.
        let lone_slash = CString::new("/").expect("slash");
        let trailing = CString::new(format!("/rmw_cn/trail/{suffix}/")).expect("trailing");
        let doubled = CString::new(format!("/rmw_cn//dbl/{suffix}")).expect("doubled");
        for bad in [&lone_slash, &trailing, &doubled] {
            assert!(
                rmw_create_publisher(node, ts, bad.as_ptr(), &qos, &pub_opts).is_null(),
                "an empty-segment topic name must not create a publisher: {bad:?}"
            );
            assert!(
                rmw_create_service(node, sts, bad.as_ptr(), &qos).is_null(),
                "an empty-segment service name must not create a service: {bad:?}"
            );
        }
        assert!(
            transport_has_no_service("/"),
            "a lone '/' must not have minted a `//data` service"
        );

        // NOTHING was minted under either spelling.
        let transport = TransportManager::get().expect("transport");
        assert!(
            transport.create_subscriber_open_only(&relative).is_err(),
            "no service under the relative spelling"
        );
        assert!(
            transport.create_subscriber_open_only(&prefixed).is_err(),
            "no service under a silently-prefixed spelling either"
        );
        let names = topic_names_and_types(node);
        assert!(
            !names.iter().any(|(n, _)| n == &relative || n == &prefixed),
            "neither spelling may appear in the graph: {names:?}"
        );

        // The queries refuse the same way.
        let mut count = 0usize;
        assert_eq!(
            rmw_count_publishers(node, relative_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            rmw_count_subscribers(node, relative_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            rmw_count_services(node, relative_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            rmw_count_clients(node, relative_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );

        // LOUD: each refusal names its entity and the reason ...
        assert!(logs_contain("publisher creation refused"));
        assert!(logs_contain("subscription creation refused"));
        assert!(logs_contain("service entity creation refused"));
        assert!(logs_contain("no leading '/'"));
        assert!(logs_contain("ROS name is empty"));
        assert!(logs_contain("empty segment"));
        // ... and it IS a refusal, not a repaired success: no line claims a
        // publisher was created under either spelling.
        logs_assert(|lines: &[&str]| {
            match lines
                .iter()
                .find(|l| l.contains("publisher created") && l.contains(&relative))
            {
                Some(line) => Err(format!("a refused name must not be created: {line}")),
                None => Ok(()),
            }
        });

        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A ROS service name is the Cerulion service name VERBATIM: the rmw stores
/// `/rmw_cn/add_two_ints/<s>`, the graph query returns it unchanged with the
/// `pkg/srv/Name` type, and a NATIVE service client addressing that canonical
/// name sees the rmw server available — the two sides meet. The count query
/// keys by the same name and refuses the stripped spelling.
#[test]
#[traced_test]
#[serial]
fn a_ros_service_name_is_the_cerulion_service_name_verbatim() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("Add{suffix}");
        let sts = service_ts(&type_name);
        let (node, _opts) = setup_node(&format!("cn_svc_node_{suffix}"));

        let ros_service = format!("/rmw_cn/add_two_ints/{suffix}");
        let stripped = format!("rmw_cn/add_two_ints/{suffix}");
        let name_c = CString::new(ros_service.clone()).expect("service");
        let qos = default_qos();

        let service = rmw_create_service(node, sts, name_c.as_ptr(), &qos);
        assert!(!service.is_null(), "a fully-qualified service name creates");
        let data = &*((*service).data as *const ServiceData);
        assert_eq!(
            data.service_name, ros_service,
            "the Cerulion service name the rmw stores IS the ROS name"
        );

        // The ROS-visible name round-trips unchanged.
        let names = service_names_and_types(node);
        assert!(
            names
                .iter()
                .any(|(n, t)| n == &ros_service && t == &format!("rmw_cn/srv/{type_name}")),
            "the graph must list {ros_service} with its ROS srv type; got {names:?}"
        );
        assert!(
            names.iter().all(|(n, _)| n.starts_with('/')),
            "every ROS-visible service name is fully qualified: {names:?}"
        );
        assert!(!names.iter().any(|(n, _)| n == &stripped));

        // A NATIVE client on the CANONICAL name meets the rmw server.
        let transport = TransportManager::get().expect("transport");
        let guid = derive_client_guid("rmw_cn_native_probe", &ros_service, 0);
        let client = transport
            .create_service_client(
                &ros_service,
                guid,
                MaxSliceLen::try_new(4096).expect("slice len"),
                data.request_bridge.schema_hash(),
                data.response_bridge.schema_hash(),
            )
            .expect("a native client on the canonical service name");
        let mut available = false;
        for _ in 0..SETTLE_POLLS {
            if client.server_available() {
                available = true;
                break;
            }
            std::thread::sleep(SETTLE_STEP);
        }
        assert!(
            available,
            "the rmw server on {ros_service} must be visible to a native client on that name"
        );

        // Counts key by the same name; the stripped spelling is refused.
        let mut count = 0usize;
        assert_eq!(
            rmw_count_services(node, name_c.as_ptr(), &mut count),
            RMW_RET_OK
        );
        assert_eq!(count, 1);
        let stripped_c = CString::new(stripped).expect("stripped");
        assert_eq!(
            rmw_count_services(node, stripped_c.as_ptr(), &mut count),
            RMW_RET_INVALID_ARGUMENT
        );

        drop(client);
        assert_eq!(rmw_destroy_service(node, service), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// (2) Every publisher registers for network egress
// =====================================================================

/// After `rmw_create_publisher` the process's runtime-egress registration set
/// holds the canonical topic with the bridge's schema hash — for a fixed POD
/// type AND a variable one (its introspection hash; the two differ and
/// neither is 0) — and the record CROSSES the reserved control channel: a
/// gateway reader on the same SHM root, booted BEFORE the publishers existed,
/// drains it, stores the advertised hashes and ANNOUNCES both topics, with no
/// malformed or rejected record. Hand oracles: the hashes are read off the
/// bridges the rmw built, the gateway's view must equal them.
#[test]
#[traced_test]
#[serial]
fn an_rmw_publisher_registers_its_topic_for_egress_with_its_schema_hash() {
    unsafe {
        let suffix = unique_suffix();
        let pod_ts = point_ts(&format!("Eg{suffix}"));
        let var_ts = labeled_ts(&format!("EgVar{suffix}"));
        let (node, _opts) = setup_node(&format!("cn_eg_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        let pod_topic = format!("/rmw_cn/egress/pod/{suffix}");
        let var_topic = format!("/rmw_cn/egress/var/{suffix}");
        let pod_c = CString::new(pod_topic.clone()).expect("topic");
        let var_c = CString::new(var_topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        // Nothing registered before either create.
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&pod_topic),
            None
        );
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&var_topic),
            None
        );

        // Gateway FIRST (its reader creates the control service); the
        // publishers come after — the "gateway already up" fast path.
        let mut gateway = gateway_on_default_root(&suffix.to_string());
        assert!(
            gateway.runtime_registration_active(),
            "the gateway's control-channel reader opened at boot"
        );

        let pod_pub = rmw_create_publisher(node, pod_ts, pod_c.as_ptr(), &qos, &pub_opts);
        assert!(!pod_pub.is_null());
        let var_pub = rmw_create_publisher(node, var_ts, var_c.as_ptr(), &qos, &pub_opts);
        assert!(!var_pub.is_null());
        let pod_hash = (*((*pod_pub).data as *const PublisherData))
            .bridge
            .schema_hash();
        let var_hash = (*((*var_pub).data as *const PublisherData))
            .bridge
            .schema_hash();
        assert_ne!(pod_hash, 0);
        assert_ne!(var_hash, 0);
        assert_ne!(pod_hash, var_hash, "two types, two hashes");

        // Writer side: the authoritative set holds canonical topic → hash, and
        // the republish belt is running.
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&pod_topic),
            Some(pod_hash),
            "the POD publisher registered with its bridge hash"
        );
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&var_topic),
            Some(var_hash),
            "the variable-type publisher registered with its introspection hash"
        );
        assert!(
            rt.transport.registration_pump_active(),
            "the periodic republish belt is live after the first registration"
        );

        // The records CROSS the control channel to the gateway.
        let landed = settle_until(&rt.transport, &mut gateway, |g| {
            g.runtime_topic_schema_hash(&pod_topic).is_some()
                && g.runtime_topic_schema_hash(&var_topic).is_some()
        });
        assert!(landed, "both registrations must reach the gateway");
        let registered = gateway
            .runtime_registered_topics()
            .expect("runtime_registered_topics");
        assert!(registered.contains(&pod_topic), "{registered:?}");
        assert!(registered.contains(&var_topic), "{registered:?}");
        assert_eq!(
            gateway.runtime_topic_schema_hash(&pod_topic),
            Some(pod_hash),
            "the gateway stores the POD hash the rmw advertised"
        );
        assert_eq!(
            gateway.runtime_topic_schema_hash(&var_topic),
            Some(var_hash),
            "the gateway stores the variable-type hash the rmw advertised"
        );
        // Discovery truth: both are announced on the network plane.
        let announced = gateway
            .manager()
            .network()
            .expect("the gateway manager has a network")
            .announced_topics();
        assert!(announced.contains(&pod_topic), "{announced:?}");
        assert!(announced.contains(&var_topic), "{announced:?}");
        assert_eq!(gateway.registration_malformed_count(), 0);
        assert_eq!(gateway.registration_rejected_count(), 0);

        drop(gateway);
        assert_eq!(rmw_destroy_publisher(node, pod_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, var_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Egress is PRODUCED topics only: a subscription registers nothing — with
/// the anti-tautology half in the same body (a publisher in the same process
/// DOES register, so the `None` is not a dead seam). And a destroyed
/// publisher's registration OUTLIVES it: the channel is an additive
/// per-process set with no removal primitive, so the record stays (with no
/// live producer behind it) and a re-created publisher on the same ROS name
/// lands straight back on it. This is the pin behind the destroy path's
/// no-unregister note — a removal primitive must update both.
#[test]
#[traced_test]
#[serial]
fn a_subscription_registers_nothing_and_a_destroyed_publishers_registration_outlives_it() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("Sub{suffix}"));
        let (node, _opts) = setup_node(&format!("cn_sub_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        let sub_topic = format!("/rmw_cn/subonly/{suffix}");
        let pub_topic = format!("/rmw_cn/pubgone/{suffix}");
        let sub_c = CString::new(sub_topic.clone()).expect("topic");
        let pub_c = CString::new(pub_topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, sub_c.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&sub_topic),
            None,
            "a subscription registers NOTHING for egress"
        );

        // Anti-tautology: a publisher in the same process DOES register.
        let publisher = rmw_create_publisher(node, ts, pub_c.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let hash = (*((*publisher).data as *const PublisherData))
            .bridge
            .schema_hash();
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&pub_topic),
            Some(hash)
        );
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&sub_topic),
            None,
            "the publisher's registration does not leak onto the subscription's topic"
        );

        // Destroy the publisher: the SHM producer is gone ...
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(
            rt.transport.topic_publisher_count(&pub_topic),
            0,
            "no live producer remains on the topic"
        );
        // ... but the registration OUTLIVES it (documented, no removal exists).
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&pub_topic),
            Some(hash),
            "a destroyed publisher's egress registration is NOT withdrawn — the \
             channel is additive per process"
        );

        // A re-created publisher on the same ROS name lands on the standing
        // record, same hash (the register is idempotent).
        let again = rmw_create_publisher(node, ts, pub_c.as_ptr(), &qos, &pub_opts);
        assert!(!again.is_null());
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&pub_topic),
            Some(hash)
        );
        assert_eq!(rmw_destroy_publisher(node, again), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The register-LAST pin (verified with a panic probe): a
/// failure AFTER the transport publisher exists but BEFORE the handle is
/// complete must leave NO egress record. The channel is additive with no
/// removal primitive, so a record pushed before a later step panicked would
/// be a PHANTOM — a topic the gateway announces and accepts demand for with
/// zero producers, for the life of the process. The failure is injected
/// through the `test-seams` panic seam that sits just ahead of the
/// registration call, and the create must fail THERE (the seam's
/// fired counter moves by exactly one) so the `None` is attributable.
/// Control: the same create, disarmed, succeeds AND registers — the `None`
/// is the ordering, not a dead seam.
///
/// Moving `register_dynamic_egress` above the seam
/// fails this test at the no-record assertion with
/// `Some(<hash>)`.
#[test]
#[traced_test]
#[serial]
fn a_failure_after_transport_creation_leaves_no_phantom_egress_record() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("Ph{suffix}"));
        let (node, _opts) = setup_node(&format!("cn_ph_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        let topic = format!("/rmw_cn/phantom/{suffix}");
        let topic_c = CString::new(topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        let fired_before = rmw_cerulion::test_seams::publisher_create_panics_fired();
        let publisher = {
            let _armed = rmw_cerulion::test_seams::PublisherCreatePanicGuard::arm();
            rmw_create_publisher(node, ts, topic_c.as_ptr(), &qos, &pub_opts)
        };
        assert!(
            publisher.is_null(),
            "the injected failure must fail the create (ffi_guard → null)"
        );
        assert_eq!(
            rmw_cerulion::test_seams::publisher_create_panics_fired(),
            fired_before + 1,
            "the create failed AT the seam, exactly once (anti-tautology)"
        );

        // NO phantom: no egress record, no transport producer, no registry
        // entry — the failed create left nothing behind under this name.
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&topic),
            None,
            "a create that failed after its transport publisher existed must NOT \
             have registered the topic for egress"
        );
        assert_eq!(
            rt.transport.topic_publisher_count(&topic),
            0,
            "the transport publisher was released on unwind"
        );
        let mut count = usize::MAX;
        assert_eq!(
            rmw_count_publishers(node, topic_c.as_ptr(), &mut count),
            RMW_RET_OK
        );
        assert_eq!(count, 0, "no registry entry for the failed create");

        // Control: disarmed, the identical create succeeds and registers.
        let ok = rmw_create_publisher(node, ts, topic_c.as_ptr(), &qos, &pub_opts);
        assert!(!ok.is_null(), "the disarmed create succeeds");
        let hash = (*((*ok).data as *const PublisherData)).bridge.schema_hash();
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&topic),
            Some(hash),
            "the successful create registers — so the earlier None was the ordering"
        );
        assert_eq!(
            rmw_cerulion::test_seams::publisher_create_panics_fired(),
            fired_before + 1,
            "the disarmed seam did not fire"
        );

        assert_eq!(rmw_destroy_publisher(node, ok), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A publisher whose egress registration PANICS inside the transport (the
/// core `test-helpers` seam at the registration's live send — its one
/// panic-capable step) is CONTAINED, and the egress plane stays usable for
/// the rest of the process. Sorted FIRST in this binary on purpose (libtest
/// runs tests in name order under one thread): it is the process's
/// first-ever registration, so the pump-arm ordering is observable too.
///
/// A transport that holds its lazy-slot guard across the whole
/// registration lets the panic POISON it, and every later publisher in the
/// process fails registration (local-only forever); and a panic after the
/// record is retained but before the pump is armed leaves that record never
/// republished. Here: the handle survives (`catch_unwind`), the record is
/// retained, the pump is live, a SECOND publisher on another topic registers
/// normally, the latch reports recovery, and both topics reach a gateway that
/// boots afterwards — the first only via the republish belt.
///
/// Reverting the transport to hold the guard across the registration, in
/// retain/send/arm order, fails this test at the second publisher's record.
#[test]
#[traced_test]
#[serial]
fn a_contained_registration_panic_leaves_the_plane_usable() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("Cp{suffix}"));
        let (node, _opts) = setup_node(&format!("cn_cp_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        let first = format!("/rmw_cn/contained/first/{suffix}");
        let second = format!("/rmw_cn/contained/second/{suffix}");
        let first_c = CString::new(first.clone()).expect("topic");
        let second_c = CString::new(second.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        assert!(
            !rt.transport.registration_pump_active(),
            "this test must be the process's FIRST registration (it is name-sorted \
             first) for the pump-arm ordering half to be observable"
        );

        cerulion_core::transport::reg_channel::panic_on_next_register_send_for_test(&first);
        let first_pub = rmw_create_publisher(node, ts, first_c.as_ptr(), &qos, &pub_opts);
        assert!(
            !first_pub.is_null(),
            "a registration panic is CONTAINED — the caller keeps its handle"
        );
        let first_hash = (*((*first_pub).data as *const PublisherData))
            .bridge
            .schema_hash();
        assert!(logs_contain("dynamic egress registration failed"));
        assert!(logs_contain(
            cerulion_core::transport::reg_channel::REGISTER_SEND_PANIC_MSG
        ));

        // Retained + pump armed BEFORE the panic-capable send; the producer
        // itself is live.
        assert!(
            rt.transport.registration_pump_active(),
            "the pump is armed before the send, so the retained record is republished"
        );
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&first),
            Some(first_hash),
            "the record was retained before the send"
        );
        assert_eq!(rt.transport.topic_publisher_count(&first), 1);

        // The plane is usable: a SECOND publisher registers normally.
        let second_pub = rmw_create_publisher(node, ts, second_c.as_ptr(), &qos, &pub_opts);
        assert!(!second_pub.is_null());
        let second_hash = (*((*second_pub).data as *const PublisherData))
            .bridge
            .schema_hash();
        assert_eq!(
            rt.transport.dynamic_egress_record_hash_for_test(&second),
            Some(second_hash),
            "the next registration succeeds — nothing was poisoned"
        );
        assert_eq!(rt.transport.topic_publisher_count(&second), 1);
        // No recovery line is expected here: the shared latch's lone-failure
        // rule — a regime whose ONLY failure was the loud head re-arms
        // SILENTLY on success (nothing was suppressed, so there is nothing to
        // report). The successful registration above IS the recovery.

        // Both reach a gateway booted AFTERWARDS — the first only via the
        // republish belt (its live send never happened).
        let mut gateway = gateway_on_default_root(&suffix.to_string());
        let landed = settle_until(&rt.transport, &mut gateway, |g| {
            g.runtime_topic_schema_hash(&first).is_some()
                && g.runtime_topic_schema_hash(&second).is_some()
        });
        assert!(landed, "both registrations must reach the gateway");
        assert_eq!(gateway.runtime_topic_schema_hash(&first), Some(first_hash));
        assert_eq!(
            gateway.runtime_topic_schema_hash(&second),
            Some(second_hash)
        );

        drop(gateway);
        assert_eq!(rmw_destroy_publisher(node, first_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, second_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

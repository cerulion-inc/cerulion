// SPDX-License-Identifier: AGPL-3.0-only
//! Hosted `rosidl::Buffer` backends (`rmw_cerulion::buffer_backends`), on a
//! Lyrical or Rolling build that links them.
//!
//! The message is the distro's real `sensor_msgs/msg/Image`, through its own
//! generated C++ introspection and fastrtps_cpp type supports. The backend is
//! the shim's fixture backend, installed under the name `cuda`: its descriptor
//! has the field layout of NVIDIA's `CudaBufferDescriptor.msg` and is written
//! with the calls the generated code makes for it, and its "device memory" is
//! a process table keyed by `ipc_uid`.
//!
//! Two groups:
//! * CDR conformance against the S2 golden fixtures captured from
//!   rmw_fastrtps_cpp and rmw_zenoh_cpp with NVIDIA's backend on a Jetson
//!   (`tests/fixtures/cuda_buffer/golden/descriptor/`): the endpoint-aware
//!   serialization of the fixture image is byte-for-byte the captured wire,
//!   the plain one is the captured CPU form, and the captured wire decodes to
//!   the captured field values.
//! * Routing through the rmw C ABI: descriptors to an accepting subscription,
//!   plain CDR to a CPU-only one beside it, native frames once a subscriber
//!   that is not buffer-aware attaches, a stale descriptor dropped as a lost
//!   sample, nothing sent with no subscriber.
//!
//! The counters are process-wide, so every test runs `#[serial]` and reads
//! deltas.
#![cfg(all(target_os = "linux", cerulion_has_buffer_backends))]

use serial_test::serial;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};
use std::sync::Once;

use rmw_cerulion::buffer_backends::{self, ffi as bb};
use rmw_cerulion::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};
use rmw_cerulion::ffi::{self, rosidl_message_type_support_t, RMW_RET_OK};
use rmw_cerulion::runtime::{PublisherData, SubscriptionData};
use rmw_cerulion::*;

const FIXTURES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/cuda_buffer/golden/descriptor/"
);

#[repr(C)]
struct FixtureDescriptor {
    size: u64,
    element_type_name: *const c_char,
    device_id: i32,
    use_ipc: u8,
    vmm_pid: i32,
    vmm_block_id: u32,
    vmm_block_size: u64,
    vmm_socket_path: *const c_char,
    ipc_uid: u64,
    ipc_event_handle: [u8; 64],
}

extern "C" {
    fn rmw_cerulion_bb_install_fixture_backend(ty: *const c_char) -> c_int;
    fn rmw_cerulion_bb_fixture_deny(gid: *const u8, deny: c_int);
    fn rmw_cerulion_bb_fixture_forget(ipc_uid: u64);
    fn rmw_cerulion_bb_fixture_buffer_assign(
        buf: *mut c_void,
        ty: *const c_char,
        data: *const u8,
        len: usize,
        d: *const FixtureDescriptor,
    ) -> c_int;
    fn rmw_cerulion_bb_fixture_buffer_inspect(
        buf: *const c_void,
        type_out: *mut c_char,
        type_cap: usize,
        size: *mut usize,
        bytes: *mut *const u8,
        ipc_uid: *mut u64,
        vmm_pid: *mut i32,
    ) -> c_int;
    fn rmw_cerulion_cppstring_assign(s: *mut c_void, data: *const c_char, len: usize);
    fn rmw_cerulion_cppstring_view(s: *const c_void, data: *mut *const c_char, len: *mut usize);
    fn dlopen(filename: *const c_char, flags: c_int) -> *mut c_void;
    fn malloc(size: usize) -> *mut c_void;
    fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void;
    fn calloc(n: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    fn dlerror() -> *const c_char;
}

const RTLD_NOW: c_int = 2;
const RTLD_GLOBAL: c_int = 0x100;

/// Hex string to bytes.
fn unhex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex"))
        .collect()
}

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!("{FIXTURES}{name}")).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// The live descriptor the S2 probe captured (probe.json `live_descriptor`).
fn live_descriptor(ty: &CStr, path: &CStr) -> FixtureDescriptor {
    let mut handle = [0u8; 64];
    handle.copy_from_slice(&unhex(
        "c9200000000000000100000000000000ff0f000000000000000000000000000000000000000000000000\
         00000000000000000000000000000000000000000000",
    ));
    FixtureDescriptor {
        size: 48,
        element_type_name: ty.as_ptr(),
        device_id: 0,
        use_ipc: 1,
        vmm_pid: 8393,
        vmm_block_id: 0,
        vmm_block_size: 2_097_152,
        vmm_socket_path: path.as_ptr(),
        ipc_uid: 4_120_325_463_928_950_868,
        ipc_event_handle: handle,
    }
}

static INSTALL: Once = Once::new();

/// Install the fixture backend as `cuda`, once per process.
fn install() {
    INSTALL.call_once(|| unsafe {
        assert_eq!(
            rmw_cerulion_bb_install_fixture_backend(c"cuda".as_ptr()),
            0,
            "{}",
            bb::last_error()
        );
        assert!(buffer_backends::installed_backends().contains(&"cuda".to_owned()));
    });
}

/// The aggregate C++ type support of sensor_msgs/msg/Image.
fn image_ts() -> *const rosidl_message_type_support_t {
    unsafe {
        let lib = dlopen(
            c"libsensor_msgs__rosidl_typesupport_cpp.so".as_ptr(),
            RTLD_NOW | RTLD_GLOBAL,
        );
        assert!(
            !lib.is_null(),
            "sensor_msgs C++ type support not loadable: {}",
            CStr::from_ptr(dlerror()).to_string_lossy()
        );
        let sym = dlsym(
            lib,
            c"rosidl_typesupport_cpp__get_message_type_support_handle__sensor_msgs__msg__Image"
                .as_ptr(),
        );
        assert!(!sym.is_null(), "Image type support symbol missing");
        let get: unsafe extern "C" fn() -> *const rosidl_message_type_support_t =
            std::mem::transmute(sym);
        get()
    }
}

unsafe fn resolve(ts: *const rosidl_message_type_support_t, id: &CStr) -> *const c_void {
    let mut cur = ts;
    for _ in 0..4 {
        if CStr::from_ptr((*cur).typesupport_identifier) == id {
            return (*cur).data;
        }
        let next = ((*cur).func.expect("dispatcher"))(cur, id.as_ptr());
        assert!(!next.is_null(), "no {id:?} type support");
        cur = next;
    }
    panic!("type support chain too deep")
}

fn introspection(ts: *const rosidl_message_type_support_t) -> &'static CppMessageMembers {
    unsafe { &*(resolve(ts, c"rosidl_typesupport_introspection_cpp") as *const CppMessageMembers) }
}

fn fastrtps_callbacks(ts: *const rosidl_message_type_support_t) -> *const c_void {
    unsafe { resolve(ts, c"rosidl_typesupport_fastrtps_cpp") }
}

fn member<'a>(m: &'a CppMessageMembers, name: &str) -> &'a CppMessageMember {
    let members = unsafe { std::slice::from_raw_parts(m.members_, m.member_count_ as usize) };
    members
        .iter()
        .find(|x| unsafe { CStr::from_ptr(x.name_) }.to_str() == Ok(name))
        .unwrap_or_else(|| panic!("no member {name}"))
}

fn nested(m: &CppMessageMember) -> &'static CppMessageMembers {
    unsafe { &*((*m.members_).data as *const CppMessageMembers) }
}

/// A C++ Image built through its own init function, freed on drop.
struct Image {
    members: &'static CppMessageMembers,
    storage: Vec<u64>,
}

impl Image {
    fn new(ts: *const rosidl_message_type_support_t) -> Self {
        let members = introspection(ts);
        let mut storage = vec![0u64; members.size_of_.div_ceil(8)];
        unsafe { (members.init_function.expect("init"))(storage.as_mut_ptr() as *mut c_void, 0) };
        Self { members, storage }
    }
    fn ptr(&mut self) -> *mut c_void {
        self.storage.as_mut_ptr() as *mut c_void
    }
    unsafe fn at(&mut self, path: &[&str]) -> *mut u8 {
        let mut m = self.members;
        let mut p = self.ptr() as *mut u8;
        for (i, name) in path.iter().enumerate() {
            let mm = member(m, name);
            p = p.add(mm.offset_ as usize);
            if i + 1 < path.len() {
                m = nested(mm);
            }
        }
        p
    }
    unsafe fn set_string(&mut self, path: &[&str], v: &str) {
        rmw_cerulion_cppstring_assign(
            self.at(path) as *mut c_void,
            v.as_ptr() as *const c_char,
            v.len(),
        );
    }
    unsafe fn string(&mut self, path: &[&str]) -> String {
        let mut d = std::ptr::null();
        let mut n = 0;
        rmw_cerulion_cppstring_view(self.at(path) as *const c_void, &mut d, &mut n);
        String::from_utf8_lossy(std::slice::from_raw_parts(d as *const u8, n)).into_owned()
    }
    /// The fixture image of the S2 probe: stamp 1.2, frame "s2", 4x4 rgb8, step 12.
    unsafe fn fill_fixture_fields(&mut self) {
        *(self.at(&["header", "stamp", "sec"]) as *mut i32) = 1;
        *(self.at(&["header", "stamp", "nanosec"]) as *mut u32) = 2;
        self.set_string(&["header", "frame_id"], "s2");
        *(self.at(&["height"]) as *mut u32) = 4;
        *(self.at(&["width"]) as *mut u32) = 4;
        self.set_string(&["encoding"], "rgb8");
        *self.at(&["is_bigendian"]) = 0;
        *(self.at(&["step"]) as *mut u32) = 12;
    }
    unsafe fn set_device_data(&mut self, bytes: &[u8], d: &FixtureDescriptor) {
        let r = rmw_cerulion_bb_fixture_buffer_assign(
            self.at(&["data"]) as *mut c_void,
            c"cuda".as_ptr(),
            bytes.as_ptr(),
            bytes.len(),
            d,
        );
        assert_eq!(r, 0);
    }
    /// (backend, size, device bytes when it is a fixture device Buffer, ipc_uid).
    unsafe fn data(&mut self) -> (String, usize, Option<Vec<u8>>, u64) {
        let mut ty = [0 as c_char; 32];
        let (mut size, mut bytes, mut uid, mut pid) = (0usize, std::ptr::null(), 0u64, 0i32);
        let r = rmw_cerulion_bb_fixture_buffer_inspect(
            self.at(&["data"]) as *const c_void,
            ty.as_mut_ptr(),
            ty.len(),
            &mut size,
            &mut bytes,
            &mut uid,
            &mut pid,
        );
        assert_ne!(r, 2, "inspect threw");
        let ty = CStr::from_ptr(ty.as_ptr()).to_string_lossy().into_owned();
        let device = (r == 1).then(|| std::slice::from_raw_parts(bytes, size).to_vec());
        (ty, size, device, uid)
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        unsafe { (self.members.fini_function.expect("fini"))(self.ptr()) };
    }
}

/// Serialize through the shim. `gid` None: plain CDR.
unsafe fn serialize(cb: *const c_void, msg: *const c_void, gid: Option<&[u8; 16]>) -> Vec<u8> {
    let mut out = vec![0u8; 4096];
    let mut len = 0;
    let r = bb::rmw_cerulion_bb_serialize(
        cb,
        msg,
        gid.map_or(std::ptr::null(), |g| g.as_ptr()),
        c"sensor_msgs/msg/Image".as_ptr(),
        out.as_mut_ptr(),
        out.len(),
        &mut len,
    );
    assert_eq!(r, 0, "{}", bb::last_error());
    out.truncate(len);
    out
}

fn counter(name: &str) -> u64 {
    buffer_backends::counters()
        .into_iter()
        .find(|(n, _)| n.ends_with(name))
        .unwrap_or_else(|| panic!("no counter {name}"))
        .1
}

#[test]
#[serial]
fn the_endpoint_aware_cdr_is_the_captured_wire_byte_for_byte() {
    install();
    let ts = image_ts();
    let cb = fastrtps_callbacks(ts);
    assert_eq!(unsafe { bb::rmw_cerulion_bb_has_buffer_fields(cb) }, 1);
    let ty = c"h";
    let path = c"cuda_vmm_8393_0";
    let d = live_descriptor(ty, path);
    let mut img = Image::new(ts);
    unsafe {
        img.fill_fixture_fields();
        img.set_device_data(&[0xab; 48], &d);
        let gid = buffer_backends::buffer_gid(0, 1, 1);
        let created = counter("descriptors_created");
        let aware = serialize(cb, img.ptr(), Some(&gid));
        assert_eq!(
            aware,
            fixture("image_cuda_descriptor.ddscdr.cdr"),
            "rmw_zenoh_cpp wire"
        );
        assert_eq!(
            aware,
            fixture("image_cuda_descriptor.xcdr1.cdr"),
            "rmw_fastrtps_cpp wire"
        );
        assert_eq!(&aware[44..48], &[0xff; 4], "marker word 1 at 44");
        assert_eq!(&aware[48..52], b"2SOR", "marker word 2 at 48");
        assert_eq!(counter("descriptors_created"), created + 1);

        // A backend that declines this subscriber: the Buffer goes out as bytes.
        rmw_cerulion_bb_fixture_deny(gid.as_ptr(), 1);
        let refused = counter("descriptors_refused");
        let declined = serialize(cb, img.ptr(), Some(&gid));
        rmw_cerulion_bb_fixture_deny(gid.as_ptr(), 0);
        assert_eq!(declined, fixture("image_cpu_legacy.xcdr1.cdr"));
        assert_eq!(counter("descriptors_refused"), refused + 1);

        // The plain serialization writes the bytes too (the captured CPU form).
        assert_eq!(
            serialize(cb, img.ptr(), None),
            fixture("image_cuda_plain.xcdr1.cdr")
        );
    }
}

#[test]
#[serial]
fn the_captured_wire_decodes_to_the_captured_descriptor_and_a_stale_one_is_lost() {
    install();
    let ts = image_ts();
    let cb = fastrtps_callbacks(ts);
    let wire = fixture("image_cuda_descriptor.ddscdr.cdr");
    let d = live_descriptor(c"h", c"cuda_vmm_8393_0");
    unsafe {
        // Publish once so the fixture backend's "device memory" holds the bytes.
        let mut src = Image::new(ts);
        src.fill_fixture_fields();
        src.set_device_data(&[0xab; 48], &d);
        let _ = serialize(cb, src.ptr(), Some(&buffer_backends::buffer_gid(0, 1, 2)));

        let mut out = Image::new(ts);
        let gid = buffer_backends::buffer_gid(0, 1, 3);
        let r = bb::rmw_cerulion_bb_deserialize(
            cb,
            wire.as_ptr(),
            wire.len(),
            out.ptr(),
            gid.as_ptr(),
            c"sensor_msgs/msg/Image".as_ptr(),
        );
        assert_eq!(r, 0, "{}", bb::last_error());
        assert_eq!(out.string(&["header", "frame_id"]), "s2");
        assert_eq!(out.string(&["encoding"]), "rgb8");
        assert_eq!(*(out.at(&["step"]) as *const u32), 12);
        let (backend, size, bytes, uid) = out.data();
        assert_eq!((backend.as_str(), size, uid), ("cuda", 48, d.ipc_uid));
        assert_eq!(bytes.as_deref(), Some(&[0xab; 48][..]));

        rmw_cerulion_bb_fixture_forget(d.ipc_uid);
        let mut stale = Image::new(ts);
        let lost = bb::rmw_cerulion_bb_deserialize(
            cb,
            wire.as_ptr(),
            wire.len(),
            stale.ptr(),
            gid.as_ptr(),
            c"sensor_msgs/msg/Image".as_ptr(),
        );
        assert_eq!(lost, 3, "an empty import is reported, never delivered");
    }
}

// ---------------------------------------------------------------------------
// Through the rmw C ABI.

unsafe fn setup_node(name: &str) -> *mut ffi::rmw_node_t {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let n = CString::new(name).expect("name");
    let ns = CString::new("/").expect("ns");
    let node = rmw_create_node(context, n.as_ptr(), ns.as_ptr());
    assert!(!node.is_null());
    node
}

fn qos() -> ffi::rmw_qos_profile_t {
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

unsafe fn subscribe(
    node: *mut ffi::rmw_node_t,
    ts: *const rosidl_message_type_support_t,
    topic: &CStr,
    accept: Option<&'static CStr>,
) -> *mut ffi::rmw_subscription_t {
    let mut opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    if let Some(a) = accept {
        opts.acceptable_buffer_backends = a.as_ptr();
    }
    let s = rmw_create_subscription(node, ts, topic.as_ptr(), &qos(), &opts);
    assert!(!s.is_null());
    s
}

unsafe fn sub_data(s: *const ffi::rmw_subscription_t) -> &'static SubscriptionData {
    &*((*s).data as *const SubscriptionData)
}

/// Take until a message is delivered or the queue stays empty for 2 s.
unsafe fn take(
    s: *const ffi::rmw_subscription_t,
    ts: *const rosidl_message_type_support_t,
) -> Option<Image> {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while std::time::Instant::now() < deadline {
        let mut img = Image::new(ts);
        let mut taken = false;
        assert_eq!(
            rmw_take(s, img.ptr(), &mut taken, std::ptr::null_mut()),
            RMW_RET_OK
        );
        if taken {
            return Some(img);
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    None
}

unsafe fn publish_device(
    p: *const ffi::rmw_publisher_t,
    ts: *const rosidl_message_type_support_t,
    bytes: &[u8],
    uid: u64,
) {
    let mut d = live_descriptor(c"h", c"cuda_vmm_8393_0");
    d.ipc_uid = uid;
    d.size = bytes.len() as u64;
    let mut img = Image::new(ts);
    img.fill_fixture_fields();
    img.set_device_data(bytes, &d);
    assert_eq!(rmw_publish(p, img.ptr(), std::ptr::null_mut()), RMW_RET_OK);
}

fn pattern(n: usize, seed: u8) -> Vec<u8> {
    (0..n)
        .map(|i| (i as u8).wrapping_mul(13).wrapping_add(seed))
        .collect()
}

#[test]
#[serial]
fn descriptors_reach_accepting_subscribers_and_cpu_ones_get_bytes() {
    install();
    let ts = image_ts();
    unsafe {
        let node = setup_node("bb_route");
        let topic = CString::new(format!("/bb_route/{}", std::process::id())).expect("topic");
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos(), &pub_opts);
        assert!(!publisher.is_null());
        assert!(
            !(*publisher).can_loan_messages,
            "a buffer-aware publisher serves no borrow"
        );
        let pdata = &*((*publisher).data as *const PublisherData);
        assert!(
            pdata.inner.lock().expect("lock").buffer.is_some(),
            "publisher is buffer-aware"
        );

        // No subscriber: nothing is sent, nothing copied.
        let unsent = counter("unsent_no_subscriber");
        publish_device(publisher, ts, &pattern(64, 0), 100);
        assert_eq!(counter("unsent_no_subscriber"), unsent + 1);

        // One accepting subscription: the descriptor, zero host copies.
        let gpu = subscribe(node, ts, &topic, Some(c"cuda"));
        assert!(sub_data(gpu).buffer.is_some());
        assert!(!(*gpu).can_loan_messages);
        let before = (
            counter("descriptor_frames"),
            counter("host_copies"),
            counter("descriptors_refused"),
            counter("cpu_group_frames"),
        );
        let bytes = pattern(4096, 1);
        publish_device(publisher, ts, &bytes, 101);
        let mut got = take(gpu, ts).expect("descriptor frame delivered");
        let (backend, size, dev, uid) = got.data();
        assert_eq!((backend.as_str(), size, uid), ("cuda", 4096, 101));
        assert_eq!(
            dev.as_deref(),
            Some(&bytes[..]),
            "byte-exact through the descriptor"
        );
        assert_eq!(got.string(&["encoding"]), "rgb8");
        assert_eq!(
            (
                counter("descriptor_frames"),
                counter("host_copies"),
                counter("descriptors_refused"),
                counter("cpu_group_frames")
            ),
            (before.0 + 1, before.1, before.2, before.3),
            "zero host copies on the descriptor path"
        );

        // A CPU-only buffer-aware subscription beside it: plain CDR for it,
        // the descriptor still for the GPU one, no duplicate for either.
        let cpu = subscribe(node, ts, &topic, None);
        assert!(sub_data(cpu).buffer.is_some());
        let groups = counter("cpu_group_frames");
        let bytes = pattern(512, 2);
        publish_device(publisher, ts, &bytes, 102);
        let (b, _, dev, _) = take(gpu, ts).expect("gpu").data();
        assert_eq!((b.as_str(), dev.as_deref()), ("cuda", Some(&bytes[..])));
        let mut c = take(cpu, ts).expect("cpu");
        let (b, size, dev, _) = c.data();
        assert_eq!((b.as_str(), size, dev), ("cpu", 512, None));
        assert_eq!(counter("cpu_group_frames"), groups + 1);
        assert!(
            take_none(gpu, ts) && take_none(cpu, ts),
            "one delivery per subscription"
        );

        // A second GPU subscription the backend declines (its verdict is
        // asked at the first publish it sees): it joins the CPU group, and the
        // accepted one keeps its descriptor.
        let declined = subscribe(node, ts, &topic, Some(c"cuda"));
        rmw_cerulion_bb_fixture_deny(sub_data(declined).buffer.as_ref().unwrap().gid.as_ptr(), 1);
        let (frames, groups) = (counter("descriptor_frames"), counter("cpu_group_frames"));
        let bytes = pattern(64, 3);
        publish_device(publisher, ts, &bytes, 103);
        let (b, _, dev, _) = take(gpu, ts).expect("gpu beside a declined one").data();
        assert_eq!((b.as_str(), dev.as_deref()), ("cuda", Some(&bytes[..])));
        let (b, _, _, _) = take(declined, ts).expect("declined").data();
        assert_eq!(b, "cpu");
        let _ = take(cpu, ts).expect("cpu");
        assert_eq!(
            (counter("descriptor_frames"), counter("cpu_group_frames")),
            (frames + 1, groups + 1),
            "one descriptor frame, one plain frame for the two CPU-group subscriptions"
        );
        rmw_cerulion_bb_fixture_deny(sub_data(declined).buffer.as_ref().unwrap().gid.as_ptr(), 0);
        assert_eq!(rmw_destroy_subscription(node, declined), RMW_RET_OK);

        // A stale descriptor: dropped as a lost sample, never delivered empty.
        assert_eq!(rmw_destroy_subscription(node, cpu), RMW_RET_OK);
        let lost = counter("lost_samples");
        publish_device(publisher, ts, &pattern(64, 4), 104);
        rmw_cerulion_bb_fixture_forget(104);
        assert!(
            take(gpu, ts).is_none(),
            "a stale descriptor is not delivered"
        );
        assert_eq!(counter("lost_samples"), lost + 1);

        // A subscriber that is not buffer-aware (a native reader): native
        // frames, the Buffer copied to host memory, to everyone.
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let native = rt
            .transport
            .create_subscriber(topic.to_str().unwrap())
            .expect("native reader");
        let copies = counter("host_copies");
        let natives = counter("native_frames");
        let bytes = pattern(256, 5);
        publish_device(publisher, ts, &bytes, 105);
        let mut g = take(gpu, ts).expect("gpu with a native reader attached");
        let (b, size, _, _) = g.data();
        assert_eq!((b.as_str(), size), ("cpu", 256));
        assert_eq!(counter("host_copies"), copies + 1);
        assert_eq!(counter("native_frames"), natives + 1);
        drop(native);

        assert_eq!(rmw_destroy_subscription(node, gpu), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    }
}

unsafe extern "C" fn t_alloc(size: usize, _s: *mut c_void) -> *mut c_void {
    malloc(size)
}
unsafe extern "C" fn t_free(p: *mut c_void, _s: *mut c_void) {
    free(p)
}
unsafe extern "C" fn t_realloc(p: *mut c_void, size: usize, _s: *mut c_void) -> *mut c_void {
    realloc(p, size)
}
unsafe extern "C" fn t_calloc(n: usize, size: usize, _s: *mut c_void) -> *mut c_void {
    calloc(n, size)
}

fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(t_alloc);
    a.deallocate = Some(t_free);
    a.reallocate = Some(t_realloc);
    a.zero_allocate = Some(t_calloc);
    a
}

#[test]
#[serial]
fn a_serialized_take_gets_the_native_frame_and_a_cpu_message_goes_native() {
    install();
    let ts = image_ts();
    unsafe {
        let node = setup_node("bb_ser");
        let topic = CString::new(format!("/bb_ser/{}", std::process::id())).expect("topic");
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos(), &pub_opts);
        assert!(!publisher.is_null());
        let gpu = subscribe(node, ts, &topic, Some(c"cuda"));
        let native_hash = sub_data(gpu).bridge.schema_hash();

        // A serialized take of an addressed frame returns the native frame,
        // with the device Buffer copied to host memory (one counted copy).
        let copies = counter("host_copies");
        let bytes = pattern(300, 9);
        publish_device(publisher, ts, &bytes, 201);
        let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
        ser.allocator = malloc_allocator();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        let mut taken = false;
        while !taken && std::time::Instant::now() < deadline {
            assert_eq!(
                rmw_take_serialized_message(gpu, &mut ser, &mut taken, std::ptr::null_mut()),
                RMW_RET_OK
            );
        }
        assert!(taken, "the serialized take received the frame");
        let frame = std::slice::from_raw_parts(ser.buffer, ser.buffer_length);
        let header = cerulion_core::wire::WireHeader::read_from_buf(frame).expect("header");
        assert_eq!(
            header.schema_hash, native_hash,
            "a serialized message is the native frame"
        );
        assert!(
            frame.windows(bytes.len()).any(|w| w == &bytes[..]),
            "the Buffer's bytes are in it"
        );
        assert_eq!(counter("host_copies"), copies + 1);
        (ser.allocator.deallocate.unwrap())(ser.buffer as *mut c_void, std::ptr::null_mut());

        // A message whose Buffer is on the CPU goes out native even to an
        // accelerated subscription: no descriptor frame, no host copy.
        let (frames, natives, copies) = (
            counter("descriptor_frames"),
            counter("native_frames"),
            counter("host_copies"),
        );
        let mut img = Image::new(ts);
        img.fill_fixture_fields();
        let data = img.at(&["data"]) as *mut c_void;
        // A default-constructed Buffer is a CPU Buffer; fill it through the R0 shim.
        extern "C" {
            fn rmw_cerulion_buffer_u8_assign(
                buf: *mut c_void,
                data: *const u8,
                len: usize,
            ) -> c_int;
        }
        let cpu_bytes = pattern(128, 11);
        assert_ne!(
            rmw_cerulion_buffer_u8_assign(data, cpu_bytes.as_ptr(), cpu_bytes.len()),
            2
        );
        assert_eq!(
            rmw_publish(publisher, img.ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (b, size, _, _) = take(gpu, ts).expect("cpu message delivered").data();
        assert_eq!((b.as_str(), size), ("cpu", 128));
        assert_eq!(
            (
                counter("descriptor_frames"),
                counter("native_frames"),
                counter("host_copies")
            ),
            (frames, natives + 1, copies),
        );
        assert_eq!(rmw_destroy_subscription(node, gpu), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn a_volatile_publisher_on_a_service_with_history_is_not_buffer_aware() {
    install();
    let ts = image_ts();
    unsafe {
        let node = setup_node("bb_hist");
        let topic = CString::new(format!("/bb_hist/{}", std::process::id())).expect("topic");
        let mut tl = qos();
        tl.durability = ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let first = rmw_create_publisher(node, ts, topic.as_ptr(), &tl, &pub_opts);
        assert!(!first.is_null());
        let volatile = rmw_create_publisher(node, ts, topic.as_ptr(), &qos(), &pub_opts);
        assert!(!volatile.is_null());
        let pdata = &*((*volatile).data as *const PublisherData);
        assert!(
            pdata.inner.lock().expect("lock").buffer.is_none(),
            "the service retains history, so a descriptor could be replayed after its memory is gone"
        );
        assert_eq!(rmw_destroy_publisher(node, volatile), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, first), RMW_RET_OK);
    }
}

fn string_ts() -> *const rosidl_message_type_support_t {
    unsafe {
        let lib = dlopen(
            c"libstd_msgs__rosidl_typesupport_cpp.so".as_ptr(),
            RTLD_NOW | RTLD_GLOBAL,
        );
        assert!(!lib.is_null(), "std_msgs C++ type support not loadable");
        let sym = dlsym(
            lib,
            c"rosidl_typesupport_cpp__get_message_type_support_handle__std_msgs__msg__String"
                .as_ptr(),
        );
        assert!(!sym.is_null());
        let get: unsafe extern "C" fn() -> *const rosidl_message_type_support_t =
            std::mem::transmute(sym);
        get()
    }
}

#[test]
#[serial]
fn a_subscription_that_is_not_buffer_aware_passes_over_an_addressed_frame() {
    install();
    let ts = string_ts();
    unsafe {
        let node = setup_node("bb_pass");
        let topic = CString::new(format!("/bb_pass/{}", std::process::id())).expect("topic");
        let sub = subscribe(node, ts, &topic, None);
        assert!(
            sub_data(sub).buffer.is_none(),
            "a type with no Buffer is not buffer-aware"
        );
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let mut raw = rt
            .transport
            .create_publisher_simple(
                topic.to_str().unwrap(),
                cerulion_core::wire::MaxSliceLen::try_new(4096).unwrap(),
            )
            .expect("raw publisher");
        let mut frame = vec![0u8; cerulion_core::wire::WireHeader::SIZE + 40];
        let header = cerulion_core::wire::WireHeader {
            schema_hash: sub_data(sub).bridge.schema_hash() ^ 0x55,
            total_size: frame.len() as u32,
            offset_table_offset: cerulion_core::wire::RMW_ADDRESSED_FRAME_MARK,
            offset_table_count: 0,
            sequence: 1,
            timestamp_ns: 1,
        };
        header.write_to_buf(&mut frame[..cerulion_core::wire::WireHeader::SIZE]);
        let mismatches = || {
            cerulion_core::transport::failure_regime_latch::lock_regime_latch(
                &sub_data(sub).hash_mismatches,
            )
            .total_failures()
        };
        let before = (counter("skipped_frames"), mismatches());

        raw.publish_raw(&frame).expect("publish");
        std::thread::sleep(std::time::Duration::from_millis(50));
        let mut msg = vec![0u64; 64];
        (introspection(ts).init_function.unwrap())(msg.as_mut_ptr() as *mut c_void, 0);
        let mut taken = true;
        assert_eq!(
            rmw_take(
                sub,
                msg.as_mut_ptr() as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(!taken, "an addressed frame is not taken as the native type");

        raw.publish_raw(&frame).expect("publish");
        std::thread::sleep(std::time::Duration::from_millis(50));
        let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
        ser.allocator = malloc_allocator();
        let mut taken = true;
        assert_eq!(
            rmw_take_serialized_message(sub, &mut ser, &mut taken, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(!taken, "nor returned as a serialized message");
        assert_eq!(
            (counter("skipped_frames"), mismatches()),
            (before.0 + 2, before.1),
            "passed over and counted, never reported as a type skew"
        );
        (introspection(ts).fini_function.unwrap())(msg.as_mut_ptr() as *mut c_void);
        drop(raw);
        assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
    }
}

unsafe fn take_none(
    s: *const ffi::rmw_subscription_t,
    ts: *const rosidl_message_type_support_t,
) -> bool {
    std::thread::sleep(std::time::Duration::from_millis(50));
    let mut img = Image::new(ts);
    let mut taken = false;
    assert_eq!(
        rmw_take(s, img.ptr(), &mut taken, std::ptr::null_mut()),
        RMW_RET_OK
    );
    !taken
}

#[test]
#[serial]
fn a_transient_local_publisher_is_not_buffer_aware() {
    install();
    let ts = image_ts();
    unsafe {
        let node = setup_node("bb_tl");
        let topic = CString::new(format!("/bb_tl/{}", std::process::id())).expect("topic");
        let mut q = qos();
        q.durability = ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &q, &pub_opts);
        assert!(!publisher.is_null());
        let pdata = &*((*publisher).data as *const PublisherData);
        assert!(pdata.inner.lock().expect("lock").buffer.is_none());
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    }
}

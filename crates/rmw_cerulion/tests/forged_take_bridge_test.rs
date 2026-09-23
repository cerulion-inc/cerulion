// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! Bridge level: the FORGED loaned take's decision
//! function and codec arm on both typesupport bridges, over HAND-BUILT
//! introspection data and hand-built frames — no ROS install, no
//! transport. Parallel-safe across binaries; the arms that read the
//! fixtures' process-global init/fini counters are `#[serial]` within it.
//!
//! What is pinned here, against hand oracles (never a self-compare):
//!
//! - ELIGIBILITY (`can_loan_take` / `forged_sequence_count`): a type with a
//!   string AND an unbounded primitive sequence IS eligible (the
//!   `sensor_msgs/Image` shape — strings are COPIED into the shadow, the
//!   sequence is FORGED); a string-only type is NOT; a bounded, a `bool`,
//!   or a DEFAULTED sequence is not forged; a typesupport missing
//!   `init`/`fini` is not eligible. Each exclusion is one arm, so a widened
//!   predicate fails a named test rather than a vague one.
//! - THE FORGE: after `unflatten_forged` the shadow's sequence header aims
//!   INSIDE the frame at the entry's bytes, `capacity == size`, the copied
//!   members are exact and live OUTSIDE the frame; `unforge` returns the
//!   header to `{NULL, 0, 0}`; `destroy_shadow` runs the typesupport's
//!   `fini` exactly once (over empty sequences — the whole point).
//! - ADVERSARIAL: a forge target that is not element-aligned, or a length
//!   that is not an element multiple, is REFUSED all-or-nothing (the shadow
//!   stays un-forged).
//! - PLACEMENT: an entry whose offset sits BELOW the frame's data floor
//!   (`WireLayout::data_floor` — inside the fixed section or the offset
//!   table) is never forged: that member is COPIED (the bytes the entry
//!   designates, exactly what the copying take serves), counted in
//!   `ForgeOutcome::below_floor`, left OUT of the forged mask so un-forge
//!   never nulls the copy, and freed by `fini`; an entry exactly AT the
//!   floor is forged.
//! - DETERMINISM: two identical messages forge to identical observations.
//!
//! The real-transport half (the pointer aims into a HELD iceoryx2 sample;
//! the budget arm; the read-only fault) is `rmw_shadow_take_test.rs`.

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_cppstring_construct, rmw_cerulion_cppstring_destruct,
    rmw_cerulion_cppstring_sizeof, rmw_cerulion_cppstring_view, rmw_cerulion_vector_u8_capacity,
    rmw_cerulion_vector_u8_construct, rmw_cerulion_vector_u8_data, rmw_cerulion_vector_u8_destruct,
    rmw_cerulion_vector_u8_size, rmw_cerulion_vector_u8_sizeof, CppMessageMember,
    CppMessageMembers, VecTriplet,
};
use rmw_cerulion::ffi::{
    rosidl_runtime_c__message_initialization, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use rmw_cerulion::type_bridge::{BridgedMessage, ForgeOutcome};
use rmw_cerulion::type_bridge_cpp::CppBridgedMessage;

use cerulion_core::wire::WireHeader;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_BOOLEAN: u8 = 6;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    /// The shim's `std::vector<uint8_t>::assign` — `pub(crate)` in the
    /// crate on purpose (see `introspection_cpp.rs`); a fixture declares
    /// its own binding to fill a REAL vector.
    fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn frame_range(frame: &[u8]) -> std::ops::Range<usize> {
    let start = frame.as_ptr() as usize;
    start..start + frame.len()
}

/// The payload-relative offset a variable entry's bytes start at, read back
/// from the frame's own offset table (the hand oracle for "where the forged
/// pointer must aim").
fn entry_offset(frame: &[u8], fixed_size: usize, var_idx: usize) -> (usize, usize) {
    let base = WireHeader::SIZE + fixed_size + var_idx * 8;
    let off = u32::from_le_bytes(frame[base..base + 4].try_into().unwrap()) as usize;
    let len = u32::from_le_bytes(frame[base + 4..base + 8].try_into().unwrap()) as usize;
    (off, len)
}

// =====================================================================
// C fixtures — rosidl C structs with a real-shaped init/fini pair
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
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// LaserScan-shaped: fixed f32 + an unbounded `float32[]` + a string.
#[repr(C)]
struct CScanish {
    angle_min: f32,
    ranges: CF32Seq,
    frame_id: CRosString,
}

static C_INIT_CALLS: AtomicU64 = AtomicU64::new(0);
static C_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

/// rosidl-shaped `__init`: a string starts as a valid EMPTY allocation
/// (`calloc(1)`, capacity 1) — exactly what `rosidl_runtime_c__String__init`
/// does; sequences start `{NULL, 0, 0}`.
unsafe extern "C" fn scanish_init(
    msg: *mut c_void,
    _init: rosidl_runtime_c__message_initialization,
) {
    C_INIT_CALLS.fetch_add(1, Ordering::SeqCst);
    let m = &mut *(msg as *mut CScanish);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

/// rosidl-shaped `__fini`: frees every non-null container. THIS is the
/// destructor that would `free()` a shared-memory address if a forged
/// sequence were not un-forged first.
unsafe extern "C" fn scanish_fini(msg: *mut c_void) {
    C_FINI_CALLS.fetch_add(1, Ordering::SeqCst);
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

fn c_member(
    name: &str,
    type_id: u8,
    offset: u32,
) -> rosidl_typesupport_introspection_c__MessageMember {
    rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        offset_: offset,
        ..Default::default()
    }
}

fn c_sequence(
    name: &str,
    type_id: u8,
    offset: u32,
) -> rosidl_typesupport_introspection_c__MessageMember {
    let mut m = c_member(name, type_id, offset);
    m.is_array_ = true;
    m.array_size_ = 0;
    m.is_upper_bound_ = false;
    m
}

#[allow(clippy::type_complexity)]
fn c_members(
    name: &str,
    size_of: usize,
    members: Vec<rosidl_typesupport_introspection_c__MessageMember>,
    init: Option<unsafe extern "C" fn(*mut c_void, rosidl_runtime_c__message_initialization)>,
    fini: Option<unsafe extern "C" fn(*mut c_void)>,
) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("forge_test__msg"),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            init_function: init,
            fini_function: fini,
            ..Default::default()
        },
    ))
}

fn scanish_members(
    name: &str,
    with_init_fini: bool,
) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    c_members(
        name,
        std::mem::size_of::<CScanish>(),
        vec![
            c_member("angle_min", ROS_TYPE_FLOAT, 0),
            c_sequence(
                "ranges",
                ROS_TYPE_FLOAT,
                std::mem::offset_of!(CScanish, ranges) as u32,
            ),
            c_member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CScanish, frame_id) as u32,
            ),
        ],
        with_init_fini.then_some(scanish_init as unsafe extern "C" fn(_, _)),
        with_init_fini.then_some(scanish_fini as unsafe extern "C" fn(_)),
    )
}

fn scanish_bridge() -> BridgedMessage {
    unsafe { BridgedMessage::new(scanish_members("Scanish", true)) }.expect("bridge")
}

/// A source CScanish with heap-owned containers (freed by `drop_scanish`).
unsafe fn make_scanish(angle_min: f32, ranges: &[f32], frame_id: &str) -> CScanish {
    let rdata = calloc(ranges.len().max(1), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(ranges.as_ptr(), rdata, ranges.len());
    let sdata = calloc(frame_id.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(frame_id.as_ptr(), sdata, frame_id.len());
    CScanish {
        angle_min,
        ranges: CF32Seq {
            data: rdata,
            size: ranges.len(),
            capacity: ranges.len(),
        },
        frame_id: CRosString {
            data: sdata,
            size: frame_id.len(),
            capacity: frame_id.len() + 1,
        },
    }
}

unsafe fn drop_scanish(m: CScanish) {
    free(m.ranges.data as *mut c_void);
    free(m.frame_id.data as *mut c_void);
}

unsafe fn c_string_of(s: &CRosString) -> String {
    String::from_utf8_lossy(std::slice::from_raw_parts(s.data, s.size)).into_owned()
}

// =====================================================================
// C: eligibility oracle
// =====================================================================

#[test]
fn c_string_plus_unbounded_sequence_is_take_loanable_but_not_borrowable() {
    let bridge = scanish_bridge();
    assert!(
        !bridge.can_loan,
        "a variable type never borrows (publish side)"
    );
    assert!(
        bridge.can_loan_take(),
        "Image-shaped type (string + unbounded primitive sequence) must be take-loanable"
    );
    assert_eq!(
        bridge.forged_sequence_count(),
        1,
        "exactly the f32 sequence is forged"
    );
}

#[test]
fn c_string_only_type_is_not_take_loanable() {
    #[repr(C)]
    struct CStringMsg {
        data: CRosString,
    }
    unsafe extern "C" fn init(_m: *mut c_void, _i: rosidl_runtime_c__message_initialization) {}
    unsafe extern "C" fn fini(_m: *mut c_void) {}
    let members = c_members(
        "StringOnly",
        std::mem::size_of::<CStringMsg>(),
        vec![c_member("data", ROS_TYPE_STRING, 0)],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "nothing to forge: a string-only type keeps the copying take"
    );
    assert_eq!(bridge.forged_sequence_count(), 0);
}

#[test]
fn c_typesupport_without_init_or_fini_is_not_take_loanable() {
    let bridge =
        unsafe { BridgedMessage::new(scanish_members("NoInitFini", false)) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "a shadow needs the typesupport's own init/fini; without them the copying take stays"
    );
    assert_eq!(bridge.forged_sequence_count(), 0);
}

#[test]
fn c_bounded_bool_and_defaulted_sequences_are_not_forged() {
    #[repr(C)]
    struct COneSeq {
        x: u32,
        seq: CU8Seq,
    }
    unsafe extern "C" fn init(_m: *mut c_void, _i: rosidl_runtime_c__message_initialization) {}
    unsafe extern "C" fn fini(_m: *mut c_void) {}
    let off = std::mem::offset_of!(COneSeq, seq) as u32;
    let size = std::mem::size_of::<COneSeq>();

    // Control: the plain unbounded uint8[] IS forged.
    let plain = c_members(
        "Plain",
        size,
        vec![
            c_member("x", ROS_TYPE_UINT32, 0),
            c_sequence("seq", ROS_TYPE_UINT8, off),
        ],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { BridgedMessage::new(plain) }.expect("bridge");
    assert!(bridge.can_loan_take());
    assert_eq!(bridge.forged_sequence_count(), 1);

    // Bounded uint8[<=8]: rosidl carries a bound the wire does not validate.
    let mut bounded = c_sequence("seq", ROS_TYPE_UINT8, off);
    bounded.is_upper_bound_ = true;
    bounded.array_size_ = 8;
    let members = c_members(
        "Bounded",
        size,
        vec![c_member("x", ROS_TYPE_UINT32, 0), bounded],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "a bounded sequence is copied, never forged"
    );

    // bool[]: kept off the forge so the C and C++ bridges agree.
    let members = c_members(
        "Bools",
        size,
        vec![
            c_member("x", ROS_TYPE_UINT32, 0),
            c_sequence("seq", ROS_TYPE_BOOLEAN, off),
        ],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "a bool sequence is copied, never forged"
    );

    // A DEFAULTED sequence starts life owning heap memory the forge would
    // strand.
    let mut defaulted = c_sequence("seq", ROS_TYPE_UINT8, off);
    static DEFAULT_SENTINEL: u8 = 7;
    defaulted.default_value_ = &DEFAULT_SENTINEL as *const u8 as *const c_void;
    let members = c_members(
        "Defaulted",
        size,
        vec![c_member("x", ROS_TYPE_UINT32, 0), defaulted],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "a defaulted sequence is copied, never forged"
    );
}

// =====================================================================
// C: the forge itself
// =====================================================================

#[test]
#[serial]
fn c_forged_take_aims_the_sequence_into_the_frame_and_copies_the_rest() {
    let bridge = scanish_bridge();
    unsafe {
        let ranges = [1.5f32, -2.25, 3.0, 1.0e-3];
        let src = make_scanish(-0.75, &ranges, "lidar_link");
        let frame = bridge
            .flatten(&src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_scanish(src);
        let range = frame_range(&frame);
        let fixed_size = bridge.layout.fixed_size;
        // `ranges` is variable entry 0, `frame_id` entry 1.
        let (r_off, r_len) = entry_offset(&frame, fixed_size, 0);
        assert_eq!(r_len, 16, "4 × f32");
        assert_eq!(r_off % 4, 0, "the writer aligns f32 entries");

        C_INIT_CALLS.store(0, Ordering::SeqCst);
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let shadow = bridge.new_shadow().expect("shadow");
        assert_eq!(
            C_INIT_CALLS.load(Ordering::SeqCst),
            1,
            "init runs exactly once per shadow"
        );
        assert!(
            bridge.forged_members_are_empty(shadow, u64::MAX),
            "a fresh shadow holds no forged sequence"
        );

        assert!(bridge
            .unflatten_forged(&frame[WireHeader::SIZE..], shadow)
            .is_ok());
        let s = &*(shadow as *const CScanish);
        // Fixed field: copied, exact.
        assert_eq!(s.angle_min.to_bits(), (-0.75f32).to_bits());
        // Forged: the header aims INSIDE the frame at the entry's bytes.
        let expected_ptr = frame.as_ptr() as usize + WireHeader::SIZE + r_off;
        assert_eq!(
            s.ranges.data as usize, expected_ptr,
            "forged pointer aims at the entry"
        );
        assert!(range.contains(&(s.ranges.data as usize)));
        assert_eq!(s.ranges.size, 4);
        assert_eq!(
            s.ranges.capacity, s.ranges.size,
            "capacity == size (growth must reallocate)"
        );
        let seen = std::slice::from_raw_parts(s.ranges.data, s.ranges.size);
        for (i, r) in ranges.iter().enumerate() {
            assert_eq!(
                seen[i].to_bits(),
                r.to_bits(),
                "range {i} read through SHM alias"
            );
        }
        assert!(!bridge.forged_members_are_empty(shadow, u64::MAX));
        // Copied: the string lives OUTSIDE the frame (the shadow owns it).
        assert_eq!(c_string_of(&s.frame_id), "lidar_link");
        assert!(
            !range.contains(&(s.frame_id.data as usize)),
            "a string is copied into the shadow, never aliased"
        );

        // Un-forge: back to the rosidl empty header.
        bridge.unforge(shadow, u64::MAX);
        let s = &*(shadow as *const CScanish);
        assert!(s.ranges.data.is_null());
        assert_eq!((s.ranges.size, s.ranges.capacity), (0, 0));
        assert!(bridge.forged_members_are_empty(shadow, u64::MAX));
        assert_eq!(
            c_string_of(&s.frame_id),
            "lidar_link",
            "un-forge touches only forged members"
        );

        // Destroy: fini runs once, over an already-empty sequence (so the
        // libc free inside it never sees the frame's address).
        bridge.destroy_shadow(shadow, u64::MAX);
        assert_eq!(C_FINI_CALLS.load(Ordering::SeqCst), 1);
        drop(frame);
    }
}

#[test]
#[serial]
fn c_forged_take_refuses_a_misaligned_or_ragged_target_all_or_nothing() {
    let bridge = scanish_bridge();
    unsafe {
        let src = make_scanish(0.5, &[9.0, 8.0, 7.0], "f");
        let good = bridge
            .flatten(&src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_scanish(src);
        let fixed_size = bridge.layout.fixed_size;
        let entry = WireHeader::SIZE + fixed_size; // ranges is entry 0
        let (r_off, r_len) = entry_offset(&good, fixed_size, 0);
        assert_eq!(r_len, 12);

        // Arm 1: shift the entry ONE byte (still in bounds, still a whole
        // number of elements) — the forged f32* would be misaligned.
        let mut misaligned = good.clone();
        misaligned[entry..entry + 4].copy_from_slice(&((r_off + 1) as u32).to_le_bytes());
        misaligned[entry + 4..entry + 8].copy_from_slice(&((r_len - 4) as u32).to_le_bytes());
        let shadow = bridge.new_shadow().expect("shadow");
        assert!(
            bridge
                .unflatten_forged(&misaligned[WireHeader::SIZE..], shadow)
                .is_err(),
            "an element-misaligned forge target must be refused"
        );
        assert!(
            bridge.forged_members_are_empty(shadow, u64::MAX),
            "a refused frame leaves the shadow un-forged (all-or-nothing)"
        );

        // Arm 2: a length that is not a multiple of the element size.
        let mut ragged = good.clone();
        ragged[entry + 4..entry + 8].copy_from_slice(&((r_len - 1) as u32).to_le_bytes());
        assert!(bridge
            .unflatten_forged(&ragged[WireHeader::SIZE..], shadow)
            .is_err());
        assert!(bridge.forged_members_are_empty(shadow, u64::MAX));

        // Control: the untouched frame still forges on the same shadow.
        assert!(bridge
            .unflatten_forged(&good[WireHeader::SIZE..], shadow)
            .is_ok());
        assert!(!bridge.forged_members_are_empty(shadow, u64::MAX));
        bridge.unforge(shadow, u64::MAX);
        bridge.destroy_shadow(shadow, u64::MAX);
    }
}

#[test]
#[serial]
fn c_forged_take_is_deterministic_across_two_runs() {
    let bridge = scanish_bridge();
    unsafe {
        let oracle = [0.25f32, 0.5, 0.75];
        let mut observed: Vec<(u32, Vec<u32>, String, usize, usize)> = Vec::new();
        for _ in 0..2 {
            let src = make_scanish(2.0, &oracle, "run");
            let frame = bridge
                .flatten(&src as *const _ as *const c_void, 0, 0)
                .expect("flatten");
            drop_scanish(src);
            let shadow = bridge.new_shadow().expect("shadow");
            assert!(bridge
                .unflatten_forged(&frame[WireHeader::SIZE..], shadow)
                .is_ok());
            let s = &*(shadow as *const CScanish);
            let bits = std::slice::from_raw_parts(s.ranges.data, s.ranges.size)
                .iter()
                .map(|f| f.to_bits())
                .collect();
            observed.push((
                s.angle_min.to_bits(),
                bits,
                c_string_of(&s.frame_id),
                s.ranges.size,
                s.ranges.capacity,
            ));
            bridge.destroy_shadow(shadow, u64::MAX);
        }
        let expected = (
            2.0f32.to_bits(),
            oracle.iter().map(|f| f.to_bits()).collect::<Vec<_>>(),
            "run".to_string(),
            3,
            3,
        );
        assert_eq!(observed[0], expected, "run 1 must match the hand oracle");
        assert_eq!(observed[1], expected, "run 2 must match the hand oracle");
    }
}

// =====================================================================
// C++ fixtures — a REAL std::string + a REAL std::vector<uint8_t>
// =====================================================================

/// Storage for one std::string (32 bytes on libstdc++ and libc++).
#[repr(C, align(16))]
struct StringSlot([u8; 32]);

/// Storage for one std::vector<uint8_t> (3 words).
#[repr(C, align(8))]
struct VecU8Slot([usize; 3]);

/// Image-shaped: two fixed u32s, a string, an unbounded uint8[].
#[repr(C, align(16))]
struct CppImageish {
    height: u32,
    width: u32,
    encoding: StringSlot,
    data: VecU8Slot,
}

static CPP_INIT_CALLS: AtomicU64 = AtomicU64::new(0);
static CPP_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

/// The rosidl_generator_cpp ALL constructor, hand-written: every member
/// placement-constructed / zeroed.
unsafe extern "C" fn imageish_init(msg: *mut c_void, _init: u32) {
    CPP_INIT_CALLS.fetch_add(1, Ordering::SeqCst);
    let m = &mut *(msg as *mut CppImageish);
    m.height = 0;
    m.width = 0;
    rmw_cerulion_cppstring_construct(
        m.encoding.0.as_mut_ptr() as *mut c_void,
        std::ptr::null(),
        0,
    );
    rmw_cerulion_vector_u8_construct(m.data.0.as_mut_ptr() as *mut c_void);
}

/// The destructor: `~string` + `~vector` — the latter is what would
/// `operator delete` a shared-memory address if the vector were still
/// forged.
unsafe extern "C" fn imageish_fini(msg: *mut c_void) {
    CPP_FINI_CALLS.fetch_add(1, Ordering::SeqCst);
    let m = &mut *(msg as *mut CppImageish);
    rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
    rmw_cerulion_vector_u8_destruct(m.data.0.as_mut_ptr() as *mut c_void);
}

unsafe extern "C" fn vecu8_size(field: *const c_void) -> usize {
    rmw_cerulion_vector_u8_size(field)
}
unsafe extern "C" fn vecu8_get_const(field: *const c_void, idx: usize) -> *const c_void {
    rmw_cerulion_vector_u8_data(field).add(idx) as *const c_void
}
unsafe extern "C" fn vecu8_get(field: *mut c_void, idx: usize) -> *mut c_void {
    (rmw_cerulion_vector_u8_data(field) as *mut u8).add(idx) as *mut c_void
}

fn cpp_member(name: &str, type_id: u8, offset: u32) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: type_id,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        #[cfg(cerulion_has_is_key)]
        is_key_: false,
        is_array_: false,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset,
        default_value_: std::ptr::null(),
        size_function: None,
        get_const_function: None,
        get_function: None,
        fetch_function: None,
        assign_function: None,
        resize_function: None,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer_: false,
    }
}

fn cpp_u8_vector(name: &str, offset: u32) -> CppMessageMember {
    let mut m = cpp_member(name, ROS_TYPE_UINT8, offset);
    m.is_array_ = true;
    m.size_function = Some(vecu8_size);
    m.get_const_function = Some(vecu8_get_const);
    m.get_function = Some(vecu8_get);
    m
}

fn cpp_members(
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
    init: Option<unsafe extern "C" fn(*mut c_void, u32)>,
    fini: Option<unsafe extern "C" fn(*mut c_void)>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("forge_test::msg"),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        #[cfg(cerulion_has_is_key)]
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: init,
        fini_function: fini,
    }))
}

fn imageish_members(name: &str) -> *const CppMessageMembers {
    cpp_members(
        name,
        std::mem::size_of::<CppImageish>(),
        vec![
            cpp_member("height", ROS_TYPE_UINT32, 0),
            cpp_member("width", ROS_TYPE_UINT32, 4),
            cpp_member(
                "encoding",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppImageish, encoding) as u32,
            ),
            cpp_u8_vector("data", std::mem::offset_of!(CppImageish, data) as u32),
        ],
        Some(imageish_init),
        Some(imageish_fini),
    )
}

unsafe fn cpp_string_of(slot: *const c_void) -> String {
    let mut data: *const c_char = std::ptr::null();
    let mut len = 0usize;
    rmw_cerulion_cppstring_view(slot, &mut data, &mut len);
    String::from_utf8_lossy(std::slice::from_raw_parts(data as *const u8, len)).into_owned()
}

/// Hand-built source message: REAL containers filled through the shim.
unsafe fn make_imageish(height: u32, width: u32, encoding: &str, data: &[u8]) -> Box<CppImageish> {
    assert!(rmw_cerulion_cppstring_sizeof() <= 32);
    assert_eq!(rmw_cerulion_vector_u8_sizeof(), 24);
    let mut m: Box<CppImageish> = Box::new(std::mem::zeroed());
    imageish_init(&mut *m as *mut _ as *mut c_void, 0);
    m.height = height;
    m.width = width;
    rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
    rmw_cerulion_cppstring_construct(
        m.encoding.0.as_mut_ptr() as *mut c_void,
        encoding.as_ptr() as *const c_char,
        encoding.len(),
    );
    rmw_cerulion_vector_u8_assign(
        m.data.0.as_mut_ptr() as *mut c_void,
        data.as_ptr(),
        data.len(),
    );
    m
}

unsafe fn drop_imageish(m: Box<CppImageish>) {
    let mut m = m;
    imageish_fini(&mut *m as *mut _ as *mut c_void);
}

// =====================================================================
// C++: eligibility + the forge on a REAL std::vector
// =====================================================================

#[test]
fn cpp_image_shaped_type_is_take_loanable_and_string_only_is_not() {
    let bridge = unsafe { CppBridgedMessage::new(imageish_members("Imageish")) }.expect("bridge");
    assert!(!bridge.can_loan);
    assert!(
        bridge.can_loan_take(),
        "string + uint8[] must be take-loanable"
    );
    assert_eq!(bridge.forged_sequence_count(), 1);

    #[repr(C, align(16))]
    struct CppStringMsg {
        data: StringSlot,
    }
    unsafe extern "C" fn init(msg: *mut c_void, _i: u32) {
        rmw_cerulion_cppstring_construct(msg, std::ptr::null(), 0);
    }
    unsafe extern "C" fn fini(msg: *mut c_void) {
        rmw_cerulion_cppstring_destruct(msg);
    }
    let members = cpp_members(
        "StringOnly",
        std::mem::size_of::<CppStringMsg>(),
        vec![cpp_member("data", ROS_TYPE_STRING, 0)],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    assert!(
        !bridge.can_loan_take(),
        "a string-only C++ type keeps the copying take"
    );
    assert_eq!(bridge.forged_sequence_count(), 0);
}

#[test]
fn cpp_bounded_or_bool_vectors_and_missing_init_fini_are_not_forged() {
    #[repr(C, align(16))]
    struct CppOneVec {
        x: u32,
        data: VecU8Slot,
    }
    unsafe extern "C" fn init(msg: *mut c_void, _i: u32) {
        let m = &mut *(msg as *mut CppOneVec);
        m.x = 0;
        rmw_cerulion_vector_u8_construct(m.data.0.as_mut_ptr() as *mut c_void);
    }
    unsafe extern "C" fn fini(msg: *mut c_void) {
        let m = &mut *(msg as *mut CppOneVec);
        rmw_cerulion_vector_u8_destruct(m.data.0.as_mut_ptr() as *mut c_void);
    }
    let off = std::mem::offset_of!(CppOneVec, data) as u32;
    let size = std::mem::size_of::<CppOneVec>();

    // Control.
    let plain = cpp_members(
        "Plain",
        size,
        vec![
            cpp_member("x", ROS_TYPE_UINT32, 0),
            cpp_u8_vector("data", off),
        ],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { CppBridgedMessage::new(plain) }.expect("bridge");
    assert!(bridge.can_loan_take());

    // Bounded (rosidl BoundedVector — a different C++ type entirely).
    let mut bounded = cpp_u8_vector("data", off);
    bounded.is_upper_bound_ = true;
    bounded.array_size_ = 16;
    bounded.resize_function = Some(vecu8_size_as_resize);
    let members = cpp_members(
        "Bounded",
        size,
        vec![cpp_member("x", ROS_TYPE_UINT32, 0), bounded],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    assert!(!bridge.can_loan_take(), "a BoundedVector is never forged");

    // vector<bool>: bit-packed.
    let mut bools = cpp_u8_vector("data", off);
    bools.type_id_ = ROS_TYPE_BOOLEAN;
    let members = cpp_members(
        "Bools",
        size,
        vec![cpp_member("x", ROS_TYPE_UINT32, 0), bools],
        Some(init),
        Some(fini),
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    assert!(!bridge.can_loan_take(), "vector<bool> is never forged");

    // No init/fini: cannot host a shadow.
    let members = cpp_members(
        "NoInitFini",
        size,
        vec![
            cpp_member("x", ROS_TYPE_UINT32, 0),
            cpp_u8_vector("data", off),
        ],
        None,
        None,
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    assert!(!bridge.can_loan_take(), "no init/fini ⇒ copying take");
}

/// A resize stand-in for the bounded fixture (never called on the paths
/// under test; present so the member is well-formed).
unsafe extern "C" fn vecu8_size_as_resize(_field: *mut c_void, _size: usize) {}

#[test]
#[serial]
fn cpp_forged_take_aims_the_vector_into_the_frame_and_unforges_to_null() {
    let bridge = unsafe { CppBridgedMessage::new(imageish_members("ImageF")) }.expect("bridge");
    unsafe {
        let payload: Vec<u8> = (0..1024u32).map(|i| (i * 7 % 251) as u8).collect();
        let src = make_imageish(16, 64, "rgb8", &payload);
        let frame = bridge
            .flatten(&*src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_imageish(src);
        let range = frame_range(&frame);
        let fixed_size = bridge.layout.fixed_size;
        // `encoding` is variable entry 0, `data` entry 1.
        let (d_off, d_len) = entry_offset(&frame, fixed_size, 1);
        assert_eq!(d_len, 1024);

        CPP_INIT_CALLS.store(0, Ordering::SeqCst);
        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        let shadow = bridge.new_shadow().expect("shadow");
        assert_eq!(CPP_INIT_CALLS.load(Ordering::SeqCst), 1);
        assert!(bridge.forged_members_are_empty(shadow, u64::MAX));

        assert!(bridge
            .unflatten_forged(&frame[WireHeader::SIZE..], shadow)
            .is_ok());
        let s = &*(shadow as *const CppImageish);
        assert_eq!((s.height, s.width), (16, 64));
        let enc = s.encoding.0.as_ptr() as *const c_void;
        assert_eq!(cpp_string_of(enc), "rgb8");
        let vec = s.data.0.as_ptr() as *const c_void;
        let data_ptr = rmw_cerulion_vector_u8_data(vec) as usize;
        assert_eq!(
            data_ptr,
            frame.as_ptr() as usize + WireHeader::SIZE + d_off,
            "data() aims at the entry inside the frame"
        );
        assert!(range.contains(&data_ptr));
        assert_eq!(
            rmw_cerulion_vector_u8_size(vec),
            1024,
            "size() is the entry length"
        );
        assert_eq!(
            rmw_cerulion_vector_u8_capacity(vec),
            1024,
            "capacity() == size(): every growth path must reallocate"
        );
        assert_eq!(
            std::slice::from_raw_parts(data_ptr as *const u8, 1024),
            &payload[..]
        );
        assert!(!bridge.forged_members_are_empty(shadow, u64::MAX));

        bridge.unforge(shadow, u64::MAX);
        assert!(
            rmw_cerulion_vector_u8_data(vec).is_null(),
            "un-forged data() is null"
        );
        assert_eq!(rmw_cerulion_vector_u8_size(vec), 0);
        assert_eq!(rmw_cerulion_vector_u8_capacity(vec), 0);
        assert!(bridge.forged_members_are_empty(shadow, u64::MAX));
        assert_eq!(
            cpp_string_of(enc),
            "rgb8",
            "un-forge touches only forged members"
        );

        // `fini` (the C++ destructor) runs once over an EMPTY vector — the
        // whole point of un-forging first.
        bridge.destroy_shadow(shadow, u64::MAX);
        assert_eq!(CPP_FINI_CALLS.load(Ordering::SeqCst), 1);
        drop(frame);
    }
}

#[test]
#[serial]
fn cpp_empty_sequence_forges_to_the_empty_triplet() {
    let bridge = unsafe { CppBridgedMessage::new(imageish_members("ImageE")) }.expect("bridge");
    unsafe {
        let src = make_imageish(1, 1, "mono8", &[]);
        let frame = bridge
            .flatten(&*src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_imageish(src);
        let shadow = bridge.new_shadow().expect("shadow");
        let outcome = bridge
            .unflatten_forged(&frame[WireHeader::SIZE..], shadow)
            .expect("an empty entry is accepted");
        assert_eq!(
            outcome,
            ForgeOutcome {
                forged: 0b1,
                below_floor: 0
            },
            "an empty entry is RECORDED as forged — the null triplet alone cannot tell a \
             forge from a skipped member"
        );
        let s = &*(shadow as *const CppImageish);
        let vec = s.data.0.as_ptr() as *const c_void;
        assert!(rmw_cerulion_vector_u8_data(vec).is_null());
        assert_eq!(rmw_cerulion_vector_u8_size(vec), 0);
        assert!(
            bridge.forged_members_are_empty(shadow, u64::MAX),
            "an empty entry forges to the empty triplet, which is also the un-forged state"
        );
        bridge.destroy_shadow(shadow, u64::MAX);
    }
}

// =====================================================================
// PLACEMENT: the data floor (the FrameWalker's rule, shared through
// `WireLayout::data_floor`)
// =====================================================================

/// An entry exactly AT the floor is forged; one BELOW it (aimed into the
/// fixed section, then into the offset table itself) is COPIED, left out of
/// the mask, survives un-forge, and is freed by `fini`.
#[test]
#[serial]
fn c_entry_below_the_data_floor_is_copied_not_forged_and_one_at_the_floor_is_forged() {
    let bridge = scanish_bridge();
    unsafe {
        let ranges = [4.0f32, 5.0, 6.0];
        let src = make_scanish(1.25, &ranges, "fl");
        let good = bridge
            .flatten(&src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_scanish(src);
        let fixed_size = bridge.layout.fixed_size;
        let data_floor = bridge.layout.data_floor();
        assert_eq!(
            data_floor,
            fixed_size + 2 * 8,
            "floor = fixed section + the two offset-table entries"
        );

        // AT the floor: the writer places the first entry exactly there.
        let (r_off, _) = entry_offset(&good, fixed_size, 0);
        assert_eq!(
            r_off, data_floor,
            "the natural frame's first entry sits exactly AT the floor"
        );
        let shadow = bridge.new_shadow().expect("shadow");
        let outcome = bridge
            .unflatten_forged(&good[WireHeader::SIZE..], shadow)
            .expect("an entry at the floor is served");
        assert_eq!(
            outcome,
            ForgeOutcome {
                forged: 0b1,
                below_floor: 0
            },
            "at the floor ⇒ forged (bit 0), nothing copied"
        );
        let s = &*(shadow as *const CScanish);
        assert!(frame_range(&good).contains(&(s.ranges.data as usize)));
        bridge.unforge(shadow, outcome.forged);
        assert!(bridge.forged_members_are_empty(shadow, u64::MAX));

        // BELOW the floor: aim the entry at payload offset 0 — angle_min's own
        // four bytes in the fixed section. The wire forbids it; the copy path
        // serves exactly the bytes the entry designates.
        let entry = WireHeader::SIZE + fixed_size; // ranges = entry 0
        let mut below = good.clone();
        below[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        below[entry + 4..entry + 8].copy_from_slice(&4u32.to_le_bytes());
        let outcome = bridge
            .unflatten_forged(&below[WireHeader::SIZE..], shadow)
            .expect("a below-floor entry is served by copy, not refused");
        assert_eq!(
            outcome,
            ForgeOutcome {
                forged: 0,
                below_floor: 1
            },
            "below the floor ⇒ copied and counted, NOT in the mask"
        );
        let s = &*(shadow as *const CScanish);
        assert!(
            !frame_range(&below).contains(&(s.ranges.data as usize)),
            "copied: the header must NOT alias the frame"
        );
        assert_eq!(s.ranges.size, 1);
        assert_eq!(s.ranges.capacity, 1);
        assert_eq!(
            (*s.ranges.data).to_bits(),
            1.25f32.to_bits(),
            "the copy serves the bytes the entry designates (angle_min's)"
        );
        assert_eq!(
            c_string_of(&s.frame_id),
            "fl",
            "the rest of the message is intact"
        );
        // The copy is outside the mask: un-forging leaves it for `fini`.
        bridge.unforge(shadow, outcome.forged);
        assert!(
            !s.ranges.data.is_null(),
            "a copied member survives un-forge"
        );

        // An entry inside the OFFSET TABLE itself is below the floor too.
        let mut in_table = good.clone();
        in_table[entry..entry + 4].copy_from_slice(&(fixed_size as u32).to_le_bytes());
        in_table[entry + 4..entry + 8].copy_from_slice(&8u32.to_le_bytes());
        let outcome = bridge
            .unflatten_forged(&in_table[WireHeader::SIZE..], shadow)
            .expect("served by copy");
        assert_eq!(
            outcome,
            ForgeOutcome {
                forged: 0,
                below_floor: 1
            }
        );
        let s = &*(shadow as *const CScanish);
        assert!(!frame_range(&in_table).contains(&(s.ranges.data as usize)));
        assert_eq!(s.ranges.size, 2);

        C_FINI_CALLS.store(0, Ordering::SeqCst);
        bridge.destroy_shadow(shadow, outcome.forged);
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "fini frees the copy; nothing in the frame was ever aliased"
        );
    }
}

/// The C++ twin: a below-floor `uint8[]` entry lands in a REAL heap
/// `std::vector` (the `assign()` copy path), never a forged triplet, and the
/// destructor frees that copy.
#[test]
#[serial]
fn cpp_entry_below_the_data_floor_is_copied_into_a_real_vector_not_forged() {
    let bridge = unsafe { CppBridgedMessage::new(imageish_members("ImageFloor")) }.expect("bridge");
    unsafe {
        let payload: Vec<u8> = (0..16u8).collect();
        let src = make_imageish(7, 9, "rgb8", &payload);
        let good = bridge
            .flatten(&*src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_imageish(src);
        let fixed_size = bridge.layout.fixed_size;
        assert_eq!(bridge.layout.data_floor(), fixed_size + 2 * 8);
        // `data` is entry 1: aim it at payload offset 0, eight bytes of the
        // fixed section (height + width).
        let entry = WireHeader::SIZE + fixed_size + 8;
        let mut below = good.clone();
        below[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        below[entry + 4..entry + 8].copy_from_slice(&8u32.to_le_bytes());

        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        let shadow = bridge.new_shadow().expect("shadow");
        let outcome = bridge
            .unflatten_forged(&below[WireHeader::SIZE..], shadow)
            .expect("served by copy");
        assert_eq!(
            outcome,
            ForgeOutcome {
                forged: 0,
                below_floor: 1
            }
        );
        let s = &*(shadow as *const CppImageish);
        let vec = s.data.0.as_ptr() as *const c_void;
        let data_ptr = rmw_cerulion_vector_u8_data(vec) as usize;
        assert!(
            !frame_range(&below).contains(&data_ptr),
            "copied into a real heap std::vector, never a forged triplet"
        );
        assert_eq!(rmw_cerulion_vector_u8_size(vec), 8);
        assert_eq!(
            std::slice::from_raw_parts(data_ptr as *const u8, 8),
            &below[WireHeader::SIZE..WireHeader::SIZE + 8],
            "the bytes the entry designates: the fixed section's first 8 bytes"
        );
        assert_eq!(
            cpp_string_of(s.encoding.0.as_ptr() as *const c_void),
            "rgb8"
        );
        bridge.unforge(shadow, outcome.forged);
        assert!(
            !rmw_cerulion_vector_u8_data(vec).is_null(),
            "the copy survives un-forge"
        );
        bridge.destroy_shadow(shadow, outcome.forged);
        assert_eq!(
            CPP_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "~vector frees the copy"
        );
    }
}

/// A below-floor COPY followed by a malformed later entry: the decode fails
/// all-or-nothing for the ALIASES (`forged == 0`), but the copy made before
/// the failure still lives in the shadow — the `Err` carries that count so
/// the caller RETIRES the shadow (its `fini` frees the copy) instead of
/// recycling it under the next forge (recycling it would leak the copy).
#[test]
#[serial]
fn c_copied_then_malformed_decode_reports_the_copy_so_the_shadow_is_retired() {
    let bridge = scanish_bridge();
    unsafe {
        let src = make_scanish(2.5, &[1.0, 2.0], "rt");
        let good = bridge
            .flatten(&src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_scanish(src);
        let fixed_size = bridge.layout.fixed_size;
        let entry = WireHeader::SIZE + fixed_size; // ranges = entry 0, frame_id = entry 1
        let mut frame = good.clone();
        // ranges: below the floor ⇒ copied.
        frame[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        frame[entry + 4..entry + 8].copy_from_slice(&4u32.to_le_bytes());
        // frame_id: length past the frame ⇒ malformed, AFTER the copy.
        frame[entry + 12..entry + 16].copy_from_slice(&u32::MAX.to_le_bytes());

        let shadow = bridge.new_shadow().expect("shadow");
        let err = bridge
            .unflatten_forged(&frame[WireHeader::SIZE..], shadow)
            .expect_err("a malformed later entry fails the decode");
        assert_eq!(
            err,
            ForgeOutcome {
                forged: 0,
                below_floor: 1
            },
            "the Err reports the copy made before the failure and NO surviving alias"
        );
        let s = &*(shadow as *const CScanish);
        assert!(
            !s.ranges.data.is_null(),
            "the copy is still owned by the shadow — recycling it would leak on the next forge"
        );
        assert!(
            !bridge.forged_members_are_empty(shadow, u64::MAX),
            "a copied member is not the empty header"
        );
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        bridge.destroy_shadow(shadow, err.forged);
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "retiring the shadow frees the copy"
        );
    }
}

// =====================================================================
// The PRE-WRITE entry gate, on BOTH bridges
// =====================================================================

/// Patch one offset-table entry's OFFSET (payload-relative) in place.
fn patch_entry_offset(frame: &mut [u8], fixed_size: usize, var_idx: usize, offset: u32) {
    let base = WireHeader::SIZE + fixed_size + var_idx * 8;
    frame[base..base + 4].copy_from_slice(&offset.to_le_bytes());
}

/// Patch one offset-table entry's LENGTH in place.
fn patch_entry_len(frame: &mut [u8], fixed_size: usize, var_idx: usize, len: u32) {
    let base = WireHeader::SIZE + fixed_size + var_idx * 8;
    frame[base + 4..base + 8].copy_from_slice(&len.to_le_bytes());
}

/// The read-only pre-write gate answers the SAME question
/// the decode answers, on BOTH bridges — and the C++ half is not a second,
/// unexercised copy.
///
/// The property that matters is not "it refuses malformed frames" on its own;
/// it is that the gate is a strict SUBSET of the decode's entry refusals. A
/// gate that refused a frame the decode would have SERVED is data loss, and
/// it is worse than the clobber the gate exists to prevent. So every arm
/// asserts the gate and `unflatten_forged` AGREE on the same frame, with the
/// well-formed control proving neither is stuck on "refuse".
///
/// Three malformations, each a shape a real producer can emit: an entry whose
/// offset lies outside the payload, a primitive sequence whose length is not
/// a whole number of elements, and a payload too short for its own fixed
/// section plus offset table.
#[test]
fn c_and_cpp_pre_write_gates_refuse_exactly_what_their_decodes_refuse() {
    use rmw_cerulion::take_gate::EntryVerdict;

    // ---- C bridge: `ranges` is variable entry 0, `frame_id` entry 1. ----
    let bridge = scanish_bridge();
    unsafe {
        let src = make_scanish(1.5, &[1.0, 2.0, 3.0], "laser");
        let good = bridge
            .flatten(&src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_scanish(src);
        let fixed = bridge.layout.fixed_size;
        let (_, ranges_len) = entry_offset(&good, fixed, 0);
        assert_eq!(
            ranges_len, 12,
            "premise: entry 0 is `ranges`, three f32s — if a layout change moved it, every \
             patch below would be aimed at the wrong field and must fail HERE"
        );

        assert_eq!(
            bridge.frame_entries_readable(&good[WireHeader::SIZE..]),
            Ok(()),
            "the control frame is well formed — without this every refusal below would be \
             satisfied by a gate stuck on `refuse`"
        );

        // (1) the entry's offset points past the payload.
        let mut oob = good.clone();
        let payload_len = (good.len() - WireHeader::SIZE) as u32;
        patch_entry_offset(&mut oob, fixed, 0, payload_len + 4096);
        assert_eq!(
            bridge.frame_entries_readable(&oob[WireHeader::SIZE..]),
            Err((0, EntryVerdict::EntryOutOfBounds))
        );
        let shadow = bridge.new_shadow().expect("shadow");
        assert!(
            bridge
                .unflatten_forged(&oob[WireHeader::SIZE..], shadow)
                .is_err(),
            "and the decode refuses it too — the gate is a SUBSET, never a frame the decode \
             would have served"
        );

        // (2) a ragged primitive sequence: one byte short of three whole f32s.
        let mut ragged = good.clone();
        patch_entry_len(&mut ragged, fixed, 0, ranges_len as u32 - 1);
        assert_eq!(
            bridge.frame_entries_readable(&ragged[WireHeader::SIZE..]),
            Err((0, EntryVerdict::PartialElement))
        );
        assert!(bridge
            .unflatten_forged(&ragged[WireHeader::SIZE..], shadow)
            .is_err());

        // (3) a payload too short for its own fixed section + offset table.
        let short = &good[WireHeader::SIZE..WireHeader::SIZE + fixed];
        assert_eq!(
            bridge.frame_entries_readable(short),
            Err((0, EntryVerdict::EntryOutOfBounds))
        );
        assert!(bridge.unflatten_forged(short, shadow).is_err());
        bridge.destroy_shadow(shadow, 0);
    }

    // ---- C++ bridge: `encoding` is entry 0, `data` entry 1. ----
    let cpp = unsafe { CppBridgedMessage::new(imageish_members("ImageGate")) }.expect("bridge");
    unsafe {
        let payload: Vec<u8> = (0..64u32).map(|i| i as u8).collect();
        let src = make_imageish(4, 16, "rgb8", &payload);
        let good = cpp
            .flatten(&*src as *const _ as *const c_void, 0, 0)
            .expect("flatten");
        drop_imageish(src);
        let fixed = cpp.layout.fixed_size;
        let (_, data_len) = entry_offset(&good, fixed, 1);
        assert_eq!(
            data_len, 64,
            "premise: entry 1 is `data`, 64 bytes — a layout change must fail HERE"
        );

        assert_eq!(
            cpp.frame_entries_readable(&good[WireHeader::SIZE..]),
            Ok(()),
            "the C++ control frame is well formed"
        );

        let mut oob = good.clone();
        let payload_len = (good.len() - WireHeader::SIZE) as u32;
        patch_entry_offset(&mut oob, fixed, 1, payload_len + 4096);
        assert_eq!(
            cpp.frame_entries_readable(&oob[WireHeader::SIZE..]),
            Err((1, EntryVerdict::EntryOutOfBounds)),
            "the C++ gate is a real second implementation, not an unexercised copy"
        );
        let shadow = cpp.new_shadow().expect("shadow");
        assert!(cpp
            .unflatten_forged(&oob[WireHeader::SIZE..], shadow)
            .is_err());

        // `data` is `uint8`, so a ragged LENGTH is not expressible there (every
        // length is a whole number of 1-byte elements) — the ragged class is
        // pinned on the C bridge above, and the shared per-entry verdict is
        // what both call. The truncated-payload arm applies to both.
        let short = &good[WireHeader::SIZE..WireHeader::SIZE + fixed];
        assert_eq!(
            cpp.frame_entries_readable(short),
            Err((0, EntryVerdict::EntryOutOfBounds))
        );
        assert!(cpp.unflatten_forged(short, shadow).is_err());
        cpp.destroy_shadow(shadow, 0);
    }

    // ---- The length PRE-CHECK is load-bearing, not belt-and-braces. ----
    //
    // For a type that HAS a variable member, deleting it changes nothing: the
    // per-entry walk resolves the first entry against a truncated payload,
    // fails, and returns the same verdict — so every arm above would pass
    // without it: for such a type, removing the pre-check is
    // merely output-equivalent.
    //
    // It stops being equivalent the moment the type has NO variable member:
    // the walk then has nothing to check, would answer `Ok(())` for a payload
    // shorter than the fixed section it is about to be memcpy'd from, and the
    // decode would read past the end. The only caller cannot reach that
    // (arming requires a forgeable sequence), but `frame_entries_readable` is
    // a public bridge method and its answer must be total over what it
    // accepts — so the shape is pinned here rather than left to the caller.
    let fixed_only = unsafe {
        BridgedMessage::new(c_members(
            "FixedOnly",
            4,
            vec![c_member("angle_min", ROS_TYPE_FLOAT, 0)],
            None,
            None,
        ))
    }
    .expect("a fixed-only bridge");
    assert_eq!(
        fixed_only.layout.fixed_size, 4,
        "premise: the type really is 4 fixed bytes with NO variable entry — otherwise the \
         walk below would have something to check and the arm would prove nothing"
    );
    assert_eq!(
        fixed_only.frame_entries_readable(&[]),
        Err((0, EntryVerdict::EntryOutOfBounds)),
        "a payload shorter than the fixed section must be refused even when the walk has no \
         entry to refuse it on"
    );
    assert_eq!(
        fixed_only.frame_entries_readable(&[0u8; 4]),
        Ok(()),
        "...and a payload that IS long enough is accepted, so the pre-check is a length \
         test and not a blanket refusal"
    );

    // The same, on the C++ bridge — the two pre-checks are separate code and
    // a shared claim proved on one of them is proved on neither.
    let cpp_fixed_only = unsafe {
        CppBridgedMessage::new(cpp_members(
            "CppFixedOnly",
            4,
            vec![cpp_member("height", ROS_TYPE_UINT32, 0)],
            None,
            None,
        ))
    }
    .expect("a fixed-only C++ bridge");
    assert_eq!(
        cpp_fixed_only.layout.fixed_size, 4,
        "premise: 4 fixed bytes, NO variable entry"
    );
    assert_eq!(
        cpp_fixed_only.frame_entries_readable(&[]),
        Err((0, EntryVerdict::EntryOutOfBounds))
    );
    assert_eq!(cpp_fixed_only.frame_entries_readable(&[0u8; 4]), Ok(()));
}

/// Build a payload by hand for a C++ bridge with ONE variable member: the
/// fixed section, then a one-entry offset table, then `len` bytes of data.
///
/// By hand rather than through `flatten` because the gate reads nothing but
/// the payload and the table — so a hand-built frame can express a COUNT the
/// producer side would refuse to emit, which is the whole point of the arm
/// below.
fn cpp_one_entry_payload(fixed_size: usize, table_bytes: usize, len: usize) -> Vec<u8> {
    let data_off = fixed_size + table_bytes;
    let mut p = vec![0u8; data_off + len];
    p[fixed_size..fixed_size + 4].copy_from_slice(&(data_off as u32).to_le_bytes());
    p[fixed_size + 4..fixed_size + 8].copy_from_slice(&(len as u32).to_le_bytes());
    p
}

/// The C++ gate also mirrors the DECLARED
/// BOUND, because `write_prim_seq_cpp` is the one decode arm on either bridge
/// that enforces one.
///
/// An entry can resolve inside the payload and hold a whole number of
/// elements and STILL violate the type's own bound — a fixed `T[N]` whose
/// count is not exactly `N`, or a bounded `T[<=N]` whose count exceeds `N`.
/// That decode refuses such a member AFTER earlier members have been
/// written, which is exactly the partial write the pre-write gate exists to
/// prevent, so stride alignment alone leaves a hole in the guarantee.
///
/// Both directions are asserted, and the ACCEPTING half is the load-bearing
/// one: a gate that refuses a frame the decode would have SERVED drops a
/// deliverable frame, which is worse than the clobber. The refusal
/// conditions here are transcribed from `write_prim_seq_cpp`, never widened.
#[test]
fn the_cpp_gate_mirrors_the_declared_bound_its_decode_enforces() {
    use rmw_cerulion::take_gate::EntryVerdict;

    // A bounded `uint8[<=4]`.
    let mut bounded = cpp_u8_vector("data", 0);
    bounded.is_upper_bound_ = true;
    bounded.array_size_ = 4;
    let bridge = unsafe {
        CppBridgedMessage::new(cpp_members(
            "CppBounded",
            std::mem::size_of::<VecTriplet>(),
            vec![bounded],
            None,
            None,
        ))
    }
    .expect("a bounded-vector C++ bridge");
    let fixed = bridge.layout.fixed_size;
    let table = bridge.layout.offset_table_bytes();
    assert_eq!(
        table, 8,
        "premise: exactly ONE variable entry, so the hand-built table is one 8-byte entry"
    );

    for len in 0..=4 {
        assert_eq!(
            bridge.frame_entries_readable(&cpp_one_entry_payload(fixed, table, len)),
            Ok(()),
            "a count of {len} is AT or under the bound of 4 and the decode serves it —              refusing it here would drop a deliverable frame"
        );
    }
    assert_eq!(
        bridge.frame_entries_readable(&cpp_one_entry_payload(fixed, table, 5)),
        Err((0, EntryVerdict::BoundViolated)),
        "a count of 5 exceeds the bound of 4, which `write_prim_seq_cpp` refuses mid-decode"
    );

    // The decode's OTHER bound arm — a fixed `T[N]` whose count must equal N
    // — is NOT mirrored, and this is the evidence for why: a fixed array is
    // laid out INLINE in the fixed section, so it carries no offset-table
    // entry and the walk never sees it. Asserting that here keeps the
    // omission a measured fact rather than an assumption, and makes the day
    // it stops being true a failing test rather than a silent hole.
    let mut fixed_arr = cpp_u8_vector("data", 0);
    fixed_arr.is_upper_bound_ = false;
    fixed_arr.array_size_ = 4;
    let fixed_bridge = unsafe {
        CppBridgedMessage::new(cpp_members(
            "CppFixedArr",
            std::mem::size_of::<VecTriplet>(),
            vec![fixed_arr],
            None,
            None,
        ))
    }
    .expect("a fixed-array C++ bridge");
    assert_eq!(
        fixed_bridge.layout.offset_table_bytes(),
        0,
        "a fixed `uint8[4]` carries NO variable entry — which is why the gate does not mirror \
         the decode's count-must-equal-N arm: it is unreachable from a walk over entries"
    );
}

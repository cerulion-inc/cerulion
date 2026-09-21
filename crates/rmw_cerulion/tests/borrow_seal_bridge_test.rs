// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! Windowed borrow, bridge level: the borrow-window SEAL on both
//! typesupport bridges, over HAND-BUILT introspection data, a hand-laid
//! slot buffer and a hand-recovered window extent — no ROS install, no
//! transport, no heap hook (the seal is pure arithmetic + memory ops once
//! the caller hands it the window numbers, which is exactly why it is
//! testable on macOS too). Parallel-safe across binaries; the arms
//! that read the fixtures' process-global init/fini counters are
//! `#[serial]` within it.
//!
//! What is pinned here, against hand oracles (never a self-compare):
//!
//! - THE ADOPTED FRAME: a forgeable sequence whose storage sits inside the
//!   window publishes IN PLACE — the whole sealed payload is compared
//!   byte-for-byte against a hand-assembled expected frame (head, offset
//!   table, zeroed remnant, adopted bytes untouched, copied string above
//!   the cursor), and the rmw's own `unflatten` (the take side's
//!   INDEPENDENT decoder) round-trips it.
//! - THE FINI SAFETY RULE: an adopted member's header is EMPTIED before the
//!   typesupport's `fini` runs, so the destructor never frees the slot
//!   bytes (the C fixture's `fini` calls libc `free` on any non-null
//!   container — running it over a still-forged header would corrupt this
//!   test's heap, which is the detector).
//! - ESCAPEES: storage outside the window (heap), or inside it but
//!   element-misaligned, is COPIED above the cursor and counted `escaped`;
//!   the published entry is element-aligned regardless.
//! - GAP BYTES below the cursor ship VERBATIM (never zeroed — bytes the
//!   window handed out may be LIVE caller objects), and the gap budget
//!   refuses on the boundary, exactly (`max_gap = gap - 1` refuses,
//!   `max_gap = gap` seals).
//! - REFUSALS ARE PRE-MUTATION: an Overflow / ExcessiveGaps seal leaves
//!   the struct fully intact (values readable, `fini` not run), so the
//!   caller's copy-loan fallback still has a message to flatten.
//! - DETERMINISM: two identical fills seal to byte-identical payloads.
//!
//! The rmw C-ABI half (borrow → fill → publish over real iceoryx2, with a
//! test-seam hook driving arm/disarm) is `rmw_borrow_publish_test.rs`.

use cerulion_core::wire::WireHeader;
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::bridge::AnyBridge;
use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_cppstring_construct, rmw_cerulion_cppstring_destruct,
    rmw_cerulion_cppstring_sizeof, rmw_cerulion_cppstring_view, rmw_cerulion_vector_u8_construct,
    rmw_cerulion_vector_u8_destruct, rmw_cerulion_vector_u8_size, rmw_cerulion_vector_u8_sizeof,
    CppMessageMember, CppMessageMembers,
};
use rmw_cerulion::ffi::{
    rosidl_runtime_c__message_initialization, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use rmw_cerulion::type_bridge::{
    BridgedMessage, SealRefusal, SealScratch, WindowExtent, BORROW_TAIL_ALIGN,
};
use rmw_cerulion::type_bridge_cpp::CppBridgedMessage;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    fn rmw_cerulion_vector_u8_data(v: *const c_void) -> *const u8;
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

/// A fresh per-call seal scratch. Production holds ONE per publisher and
/// reuses it across publishes (the zero-alloc contract); these arms test
/// seal SEMANTICS, so a fresh scratch keeps them independent — except the
/// determinism arm, which deliberately REUSES one scratch across both
/// seals to pin the reset-not-reallocate contract (stale plan state
/// surviving a reuse would break byte-identity).
fn scratch() -> SealScratch {
    SealScratch::new()
}

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

/// The borrow slot stand-in: one page, 16-aligned like a real payload
/// region (the real one is 8-aligned; 16 keeps the hand oracles simple and
/// is strictly stronger).
#[repr(C, align(16))]
struct Slot([u8; 4096]);

impl Slot {
    fn new() -> Box<Self> {
        Box::new(Slot([0u8; 4096]))
    }
    fn payload(&mut self) -> *mut u8 {
        self.0.as_mut_ptr()
    }
}

const PAYLOAD_LEN: usize = 4096;
/// A generous default gap budget for arms not probing the threshold.
const GAP_OK: usize = 1 << 20;

// =====================================================================
// C fixture — LaserScan-shaped (crib of forged_take_bridge_test.rs)
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

/// Fixed f32 + an unbounded `float32[]` (forgeable) + a string (copied).
#[repr(C)]
struct CScanish {
    angle_min: f32,
    ranges: CF32Seq,
    frame_id: CRosString,
}

static C_INIT_CALLS: AtomicU64 = AtomicU64::new(0);
static C_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

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
/// destructor that would `free()` a slot address if the seal did not empty
/// an adopted header first.
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

fn scanish_bridge() -> BridgedMessage {
    let members = vec![
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
    ];
    let members = Box::leak(members.into_boxed_slice());
    let members = Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("seal_test__msg"),
            message_name_: cstr("Scanish"),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScanish>(),
            members_: members.as_ptr(),
            init_function: Some(scanish_init),
            fini_function: Some(scanish_fini),
            ..Default::default()
        },
    ));
    unsafe { BridgedMessage::new(members) }.expect("bridge")
}

/// Hand geometry oracle for CScanish: fixed section = 4 (`angle_min`),
/// table = 2 × 8, data floor = 20, tail at the struct size rounded up.
const C_FIXED: usize = 4;
const C_FLOOR: usize = C_FIXED + 16;
fn c_tail_off() -> usize {
    align_up(
        std::mem::size_of::<CScanish>().max(C_FLOOR),
        BORROW_TAIL_ALIGN,
    )
}

/// Construct a CScanish IN the slot with `ranges` aimed at `ranges_ptr`
/// (the caller decides slot-tail vs heap) and `frame_id` on the heap (the
/// state a real fill leaves: strings are copied members either way).
unsafe fn build_scanish_in_slot(
    payload: *mut u8,
    angle_min: f32,
    ranges_ptr: *mut f32,
    ranges_len: usize,
    frame_id: &str,
) {
    let m = &mut *(payload as *mut CScanish);
    m.angle_min = angle_min;
    m.ranges = CF32Seq {
        data: ranges_ptr,
        size: ranges_len,
        capacity: ranges_len,
    };
    let sdata = calloc(frame_id.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(frame_id.as_ptr(), sdata, frame_id.len());
    m.frame_id = CRosString {
        data: sdata,
        size: frame_id.len(),
        capacity: frame_id.len() + 1,
    };
}

/// The hand-assembled expected payload for the ADOPTED CScanish arm.
fn c_expected_adopted_payload(
    total: usize,
    angle_min: f32,
    ranges: &[f32],
    ranges_off: usize,
    frame_id: &str,
    frame_id_off: usize,
) -> Vec<u8> {
    let mut e = vec![0u8; total];
    e[0..4].copy_from_slice(&angle_min.to_le_bytes());
    e[C_FIXED..C_FIXED + 4].copy_from_slice(&(ranges_off as u32).to_le_bytes());
    e[C_FIXED + 4..C_FIXED + 8].copy_from_slice(&((ranges.len() * 4) as u32).to_le_bytes());
    e[C_FIXED + 8..C_FIXED + 12].copy_from_slice(&(frame_id_off as u32).to_le_bytes());
    e[C_FIXED + 12..C_FIXED + 16].copy_from_slice(&(frame_id.len() as u32).to_le_bytes());
    for (i, r) in ranges.iter().enumerate() {
        e[ranges_off + 4 * i..ranges_off + 4 * i + 4].copy_from_slice(&r.to_le_bytes());
    }
    e[frame_id_off..frame_id_off + frame_id.len()].copy_from_slice(frame_id.as_bytes());
    e
}

// =====================================================================
// C: geometry + the adopted zero-copy seal
// =====================================================================

#[test]
fn c_geometry_matches_the_hand_oracle_and_gates_on_eligibility() {
    let bridge = scanish_bridge();
    assert!(bridge.can_borrow_windowed());
    let geo = bridge.borrow_geometry().expect("geometry");
    assert_eq!(geo.c_size, std::mem::size_of::<CScanish>());
    assert_eq!(geo.data_floor, C_FLOOR);
    assert_eq!(geo.tail_off, c_tail_off());
    assert!(geo.tail_off >= geo.c_size && geo.tail_off.is_multiple_of(BORROW_TAIL_ALIGN));

    // A fixed type never borrows through the window (it has the plain
    // loan), and a forgeless type has nothing to adopt.
    #[repr(C)]
    struct CFixed {
        x: f32,
    }
    let members = Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("seal_test__msg"),
            message_name_: cstr("FixedOnly"),
            member_count_: 1,
            size_of_: std::mem::size_of::<CFixed>(),
            members_: Box::leak(vec![c_member("x", ROS_TYPE_FLOAT, 0)].into_boxed_slice()).as_ptr(),
            init_function: Some(scanish_init),
            fini_function: Some(scanish_fini),
            ..Default::default()
        },
    ));
    let fixed = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert!(fixed.can_loan, "control: the fixture really is fixed");
    assert!(!fixed.can_borrow_windowed());
    assert!(fixed.borrow_geometry().is_none());
}

#[test]
#[serial]
fn c_adopted_seal_publishes_in_place_and_the_whole_payload_matches_the_oracle() {
    let bridge = scanish_bridge();
    let ranges = [1.5f32, -2.25, 3.0, 1.0e-3];
    let frame_id = "lidar_link";
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        // Simulated bump fill: the sequence storage sits at the window
        // base, exactly where a clean single-resize fill lands.
        let ranges_ptr = payload.add(tail) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), ranges_ptr, ranges.len());
        build_scanish_in_slot(payload, -0.75, ranges_ptr, ranges.len(), frame_id);
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail + ranges.len() * 4,
        };

        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");

        // Hand-computed shape: adopted bytes stay at `tail`, the copied
        // string lands at the cursor, the remnant [floor, tail) zeroes.
        let ranges_off = tail;
        let frame_id_off = tail + ranges.len() * 4;
        let total = frame_id_off + frame_id.len();
        assert_eq!(seal.total_payload, total);
        assert_eq!(
            (seal.adopted, seal.escaped, seal.copied),
            (1, 0, 1),
            "the sequence adopts, the string copies"
        );
        assert_eq!(
            seal.gap_bytes,
            tail - C_FLOOR,
            "the only dead bytes are the zeroed struct remnant"
        );
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "the seal releases the struct through the typesupport's fini — \
             and the process surviving it proves the adopted header was \
             EMPTIED first (this fixture's fini frees any non-null container)"
        );

        let got = std::slice::from_raw_parts(payload, total);
        let expected =
            c_expected_adopted_payload(total, -0.75, &ranges, ranges_off, frame_id, frame_id_off);
        assert_eq!(got, &expected[..], "whole sealed payload vs hand oracle");

        // The take side's independent decoder round-trips it.
        let mut target: CScanish = std::mem::zeroed();
        scanish_init(&mut target as *mut _ as *mut c_void, 0);
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        assert!(bridge.unflatten(got, &mut target as *mut _ as *mut c_void));
        assert_eq!(target.angle_min.to_bits(), (-0.75f32).to_bits());
        assert_eq!(target.ranges.size, ranges.len());
        let seen = std::slice::from_raw_parts(target.ranges.data, target.ranges.size);
        for (i, r) in ranges.iter().enumerate() {
            assert_eq!(seen[i].to_bits(), r.to_bits(), "range {i}");
        }
        assert_eq!(
            String::from_utf8_lossy(std::slice::from_raw_parts(
                target.frame_id.data,
                target.frame_id.size
            )),
            frame_id
        );
        scanish_fini(&mut target as *mut _ as *mut c_void);
    }
}

#[test]
#[serial]
fn c_escaped_storage_is_copied_above_the_cursor_and_freed_by_fini() {
    let bridge = scanish_bridge();
    let ranges = [9.0f32, 8.0, 7.0];
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        // The fill escaped: storage on the heap (another thread / growth
        // past the tail / foreign allocator — address-wise identical).
        let heap = calloc(ranges.len(), 4) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), heap, ranges.len());
        build_scanish_in_slot(payload, 0.5, heap, ranges.len(), "f");
        // A window was armed but nothing bumped into it.
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail,
        };

        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        assert_eq!(
            (seal.adopted, seal.escaped, seal.copied),
            (0, 1, 1),
            "the sequence escaped (copied + latched), the string copies structurally"
        );
        // Copies start AT the cursor: ranges at tail (f32-aligned already),
        // the string right after.
        let ranges_off = tail;
        let frame_id_off = tail + ranges.len() * 4;
        assert_eq!(seal.total_payload, frame_id_off + 1);
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        for (i, r) in ranges.iter().enumerate() {
            assert_eq!(
                &got[ranges_off + 4 * i..ranges_off + 4 * i + 4],
                &r.to_le_bytes(),
                "escaped bytes are served verbatim from the copy"
            );
        }
        assert_eq!(&got[frame_id_off..frame_id_off + 1], b"f");
        // The offset TABLE must point at those copies — right bytes at a
        // wrong published offset would still be an undecodable frame.
        let e0_off = u32::from_le_bytes(got[C_FIXED..C_FIXED + 4].try_into().unwrap()) as usize;
        let e0_len = u32::from_le_bytes(got[C_FIXED + 4..C_FIXED + 8].try_into().unwrap()) as usize;
        let e1_off =
            u32::from_le_bytes(got[C_FIXED + 8..C_FIXED + 12].try_into().unwrap()) as usize;
        let e1_len =
            u32::from_le_bytes(got[C_FIXED + 12..C_FIXED + 16].try_into().unwrap()) as usize;
        assert_eq!((e0_off, e0_len), (ranges_off, ranges.len() * 4));
        assert_eq!((e1_off, e1_len), (frame_id_off, 1));
        // And the take-side decoder round-trips the sealed frame.
        let mut target: CScanish = std::mem::zeroed();
        scanish_init(&mut target as *mut _ as *mut c_void, 0);
        assert!(bridge.unflatten(got, &mut target as *mut _ as *mut c_void));
        assert_eq!(target.ranges.size, ranges.len());
        let seen = std::slice::from_raw_parts(target.ranges.data, target.ranges.size);
        for (i, r) in ranges.iter().enumerate() {
            assert_eq!(seen[i].to_bits(), r.to_bits(), "round-trip range {i}");
        }
        scanish_fini(&mut target as *mut _ as *mut c_void);
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            2,
            "the seal's fini ran and freed the escaped heap buffer (the \
             header was NOT emptied for an escapee; leaking it would be the \
             in-slot rule in reverse) — plus the round-trip target's fini"
        );
    }
}

#[test]
#[serial]
fn c_misaligned_in_window_storage_is_escaped_and_republished_aligned() {
    let bridge = scanish_bridge();
    let ranges = [4.0f32, 5.0];
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        // In-window but NOT f32-aligned (a hostile / degenerate fill): the
        // adopt test must refuse it — publishing that offset would hand
        // every reader a misaligned entry they refuse.
        let odd = payload.add(tail + 1) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr() as *const u8, odd as *mut u8, 8);
        build_scanish_in_slot(payload, 1.0, odd, ranges.len(), "x");
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail + 9,
        };
        // Dirty the alignment sliver the copy placement will skip
        // ([cursor, align_up(cursor, 4))) — a recycled pool slot carries
        // the previous frame's bytes there, and the commit must zero them
        // (a fresh test buffer is already zero, which would mask a
        // skipped zeroing).
        for i in 0..3 {
            *payload.add(tail + 9 + i) = 0xAB;
        }

        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        assert_eq!((seal.adopted, seal.escaped), (0, 1));
        // The published entry is element-aligned: align_up(cursor, 4).
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        let entry_off = u32::from_le_bytes(got[C_FIXED..C_FIXED + 4].try_into().unwrap()) as usize;
        assert_eq!(entry_off, align_up(tail + 9, 4));
        assert_eq!(entry_off % 4, 0);
        assert_eq!(&got[entry_off..entry_off + 4], &4.0f32.to_le_bytes());
        assert!(
            got[tail + 9..entry_off].iter().all(|&b| b == 0),
            "the rmw-owned alignment sliver above the cursor is zeroed \
             (deterministic frames), not shipped as recycled-slot residue"
        );
    }
}

#[test]
#[serial]
fn c_empty_sequence_seals_to_a_floor_anchored_empty_entry() {
    let bridge = scanish_bridge();
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        build_scanish_in_slot(payload, 2.5, std::ptr::null_mut(), 0, "e");
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail,
        };
        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        assert_eq!((seal.adopted, seal.escaped, seal.copied), (1, 0, 1));
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        let off = u32::from_le_bytes(got[C_FIXED..C_FIXED + 4].try_into().unwrap()) as usize;
        let len = u32::from_le_bytes(got[C_FIXED + 4..C_FIXED + 8].try_into().unwrap()) as usize;
        assert_eq!(
            (off, len),
            (C_FLOOR, 0),
            "empty entries anchor at the floor"
        );

        // Round-trip: the decoder serves an empty sequence.
        let mut target: CScanish = std::mem::zeroed();
        scanish_init(&mut target as *mut _ as *mut c_void, 0);
        assert!(bridge.unflatten(got, &mut target as *mut _ as *mut c_void));
        assert_eq!((target.ranges.size, target.ranges.capacity), (0, 0));
        scanish_fini(&mut target as *mut _ as *mut c_void);
    }
}

#[test]
#[serial]
fn c_windowless_seal_copies_everything_and_matches_the_oracle() {
    let bridge = scanish_bridge();
    let ranges = [0.25f32, 0.5];
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        let heap = calloc(ranges.len(), 4) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), heap, ranges.len());
        build_scanish_in_slot(payload, 3.0, heap, ranges.len(), "w");

        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(payload, PAYLOAD_LEN, &geo, None, GAP_OK, &mut scratch())
            .expect("seal");
        // Windowless: the forgeable member still counts as ESCAPED (the
        // caller's latch tells the operator zero-copy did not happen).
        assert_eq!((seal.adopted, seal.escaped, seal.copied), (0, 1, 1));
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        let expected = c_expected_adopted_payload(
            seal.total_payload,
            3.0,
            &ranges,
            tail,
            "w",
            tail + ranges.len() * 4,
        );
        assert_eq!(got, &expected[..]);
    }
}

// =====================================================================
// C: refusals are pre-mutation; the gap budget is a two-sided threshold
// =====================================================================

#[test]
#[serial]
fn c_overflow_refusal_leaves_the_struct_intact_for_the_copy_fallback() {
    let bridge = scanish_bridge();
    let ranges = [6.0f32, 6.5, 7.0, 7.5];
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        let heap = calloc(ranges.len(), 4) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), heap, ranges.len());
        build_scanish_in_slot(payload, -1.5, heap, ranges.len(), "overflow");

        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let geo = bridge.borrow_geometry().expect("geometry");
        // A payload too short for the 16 copied sequence bytes.
        let short_len = tail + 4;
        let refusal = bridge
            .seal_borrowed_frame(payload, short_len, &geo, None, GAP_OK, &mut scratch())
            .expect_err("must refuse");
        match refusal {
            SealRefusal::Overflow { needed, available } => {
                assert_eq!(available, short_len);
                assert_eq!(
                    needed,
                    tail + ranges.len() * 4 + "overflow".len(),
                    "the refusal names the exact byte the copies needed"
                );
            }
            other => panic!("expected Overflow, got {other:?}"),
        }
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            0,
            "a refusal runs NO fini — the struct still owns its containers"
        );
        // The struct is intact: the copy-loan fallback flattens to EXACTLY
        // the frame an untouched message produces (a refusal that corrupted
        // a field before returning would pass a nonempty-check).
        let frame = bridge
            .flatten(payload as *const c_void, 0, 0)
            .expect("fallback flatten");
        let tight = c_expected_adopted_payload(
            C_FLOOR + ranges.len() * 4 + "overflow".len(),
            -1.5,
            &ranges,
            C_FLOOR,
            "overflow",
            C_FLOOR + ranges.len() * 4,
        );
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &tight[..],
            "fallback payload byte-identical to the untouched message's"
        );
        // And the release path still works.
        bridge.fini_borrowed_payload(payload as *mut c_void, PAYLOAD_LEN);
        assert_eq!(C_FINI_CALLS.load(Ordering::SeqCst), 1);
    }
}

#[test]
#[serial]
fn c_gap_budget_is_a_threshold_pinned_on_both_sides() {
    let bridge = scanish_bridge();
    let ranges = [1.0f32, 2.0, 3.0, 4.0];
    let tail = c_tail_off();
    let frame_id = "gap";
    // The fill left its storage deep in the tail (earlier bumps were
    // abandoned): everything between the floor and the data is dead.
    let deep = 256usize;
    unsafe {
        let build = |slot: &mut Slot| {
            let payload = slot.payload();
            let ranges_ptr = payload.add(tail + deep) as *mut f32;
            std::ptr::copy_nonoverlapping(ranges.as_ptr(), ranges_ptr, ranges.len());
            build_scanish_in_slot(payload, 0.0, ranges_ptr, ranges.len(), frame_id);
            WindowExtent {
                base: payload as usize + tail,
                cursor: payload as usize + tail + deep + ranges.len() * 4,
            }
        };
        // Hand gap: remnant (tail - floor) + the abandoned region (deep).
        let expected_gap = (tail - C_FLOOR) + deep;

        // AT the budget: seals, and reports the exact gap.
        let mut slot = Slot::new();
        let window = build(&mut slot);
        let geo = bridge.borrow_geometry().expect("geometry");
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let seal = bridge
            .seal_borrowed_frame(
                slot.payload(),
                PAYLOAD_LEN,
                &geo,
                Some(window),
                expected_gap,
                &mut scratch(),
            )
            .expect("at-budget seal");
        assert_eq!(seal.gap_bytes, expected_gap);
        assert_eq!(seal.adopted, 1);

        // ONE below: refused, struct intact.
        let mut slot2 = Slot::new();
        let window2 = build(&mut slot2);
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let refusal = bridge
            .seal_borrowed_frame(
                slot2.payload(),
                PAYLOAD_LEN,
                &geo,
                Some(window2),
                expected_gap - 1,
                &mut scratch(),
            )
            .expect_err("must refuse one below the budget");
        match refusal {
            SealRefusal::ExcessiveGaps {
                gap_bytes,
                max_gap_bytes,
            } => {
                assert_eq!(gap_bytes, expected_gap);
                assert_eq!(max_gap_bytes, expected_gap - 1);
            }
            other => panic!("expected ExcessiveGaps, got {other:?}"),
        }
        assert_eq!(C_FINI_CALLS.load(Ordering::SeqCst), 0);
        // The refused struct's sequence still aims into the slot; the
        // release walk empties exactly that (in-slot ⇒ never allocator
        // memory) and fini frees the heap string — hookless-sound, which
        // is what lets this arm run on a desk with no preload at all.
        bridge.fini_borrowed_payload(slot2.payload() as *mut c_void, PAYLOAD_LEN);
    }
}

#[test]
#[serial]
fn c_gap_bytes_below_the_cursor_ship_verbatim_never_zeroed() {
    let bridge = scanish_bridge();
    let ranges = [11.0f32];
    let tail = c_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        // An incidental allocation's bytes sit between the adopted data and
        // the cursor — they may be a LIVE caller object, so the seal must
        // not touch them.
        let ranges_ptr = payload.add(tail) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), ranges_ptr, ranges.len());
        let sentinel_off = tail + 4;
        let sentinel_len = 32;
        for i in 0..sentinel_len {
            *payload.add(sentinel_off + i) = 0xEE;
        }
        build_scanish_in_slot(payload, 0.0, ranges_ptr, ranges.len(), "s");
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + sentinel_off + sentinel_len,
        };
        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        assert!(
            got[sentinel_off..sentinel_off + sentinel_len]
                .iter()
                .all(|&b| b == 0xEE),
            "bump-region bytes below the cursor must ship untouched"
        );
        // …and they are accounted as gap.
        assert_eq!(seal.gap_bytes, (tail - C_FLOOR) + sentinel_len);
        // While the remnant is deterministically zero.
        assert!(got[C_FLOOR..tail].iter().all(|&b| b == 0));
    }
}

#[test]
#[serial]
fn c_two_identical_fills_seal_byte_identically() {
    let bridge = scanish_bridge();
    let ranges = [0.1f32, 0.2, 0.3];
    let tail = c_tail_off();
    unsafe {
        let mut outputs: Vec<Vec<u8>> = Vec::new();
        // ONE scratch across both seals — the production shape (per-
        // publisher reuse): a reset that left stale plan state behind
        // would break the byte-identity below.
        let mut shared_scratch = scratch();
        for _ in 0..2 {
            let mut slot = Slot::new();
            let payload = slot.payload();
            let ranges_ptr = payload.add(tail) as *mut f32;
            std::ptr::copy_nonoverlapping(ranges.as_ptr(), ranges_ptr, ranges.len());
            build_scanish_in_slot(payload, 5.0, ranges_ptr, ranges.len(), "det");
            let window = WindowExtent {
                base: payload as usize + tail,
                cursor: payload as usize + tail + ranges.len() * 4,
            };
            let geo = bridge.borrow_geometry().expect("geometry");
            let seal = bridge
                .seal_borrowed_frame(
                    payload,
                    PAYLOAD_LEN,
                    &geo,
                    Some(window),
                    GAP_OK,
                    &mut shared_scratch,
                )
                .expect("seal");
            outputs.push(std::slice::from_raw_parts(payload, seal.total_payload).to_vec());
        }
        assert_eq!(outputs[0], outputs[1], "identical fills, identical bytes");
        // Anchored to a hand oracle so this is not a self-compare.
        let expected = c_expected_adopted_payload(
            outputs[0].len(),
            5.0,
            &ranges,
            tail,
            "det",
            tail + ranges.len() * 4,
        );
        assert_eq!(outputs[0], expected);
    }
}

// =====================================================================
// C++ fixture — Image-shaped with a REAL std::string and a vector triplet
// =====================================================================

/// Storage for one std::string (32 bytes on libstdc++ and libc++).
#[repr(C, align(16))]
struct StringSlot([u8; 32]);

/// Storage for one std::vector<uint8_t> (3 words).
#[repr(C, align(8))]
struct VecU8Slot([usize; 3]);

#[repr(C, align(16))]
struct CppImageish {
    height: u32,
    width: u32,
    encoding: StringSlot,
    data: VecU8Slot,
}

static CPP_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

unsafe extern "C" fn imageish_init(msg: *mut c_void, _init: u32) {
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

/// `~string` + `~vector` — the destructor that would `operator delete` a
/// slot address if the seal did not empty an adopted triplet first.
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
    }
}

fn imageish_bridge() -> CppBridgedMessage {
    let mut data = cpp_member(
        "data",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CppImageish, data) as u32,
    );
    data.is_array_ = true;
    data.size_function = Some(vecu8_size);
    data.get_const_function = Some(vecu8_get_const);
    data.get_function = Some(vecu8_get);
    let members = vec![
        cpp_member("height", ROS_TYPE_UINT32, 0),
        cpp_member("width", ROS_TYPE_UINT32, 4),
        cpp_member(
            "encoding",
            ROS_TYPE_STRING,
            std::mem::offset_of!(CppImageish, encoding) as u32,
        ),
        data,
    ];
    let members = Box::leak(members.into_boxed_slice());
    let members = Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("seal_test::msg"),
        message_name_: cstr("Imageish"),
        member_count_: members.len() as u32,
        size_of_: std::mem::size_of::<CppImageish>(),
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: Some(imageish_init),
        fini_function: Some(imageish_fini),
    }));
    unsafe { CppBridgedMessage::new(members) }.expect("bridge")
}

const CPP_FIXED: usize = 8;
const CPP_FLOOR: usize = CPP_FIXED + 16;
fn cpp_tail_off() -> usize {
    align_up(
        std::mem::size_of::<CppImageish>().max(CPP_FLOOR),
        BORROW_TAIL_ALIGN,
    )
}

unsafe fn cpp_string_of(slot: *const c_void) -> String {
    let mut data: *const c_char = std::ptr::null();
    let mut len = 0usize;
    rmw_cerulion_cppstring_view(slot, &mut data, &mut len);
    String::from_utf8_lossy(std::slice::from_raw_parts(data as *const u8, len)).into_owned()
}

// =====================================================================
// C++: adopted + escaped seals through the typesupport accessors
// =====================================================================

#[test]
#[serial]
fn cpp_adopted_seal_publishes_the_forged_vector_in_place() {
    assert!(unsafe { rmw_cerulion_cppstring_sizeof() } <= 32);
    assert_eq!(unsafe { rmw_cerulion_vector_u8_sizeof() }, 24);
    let bridge = imageish_bridge();
    assert!(bridge.can_borrow_windowed());
    let geo = bridge.borrow_geometry().expect("geometry");
    assert_eq!(geo.data_floor, CPP_FLOOR);
    assert_eq!(geo.tail_off, cpp_tail_off());

    let payload_bytes: Vec<u8> = (0..1024u32).map(|i| (i * 7 % 251) as u8).collect();
    let encoding = "rgb8";
    let tail = cpp_tail_off();
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        // Construct the message in the slot (the borrow path's
        // init_function step), then simulate the bump fill: the pixel
        // bytes land at the window base, and the vector's triplet — the
        // state a real bump-backed `resize` leaves — aims at them.
        imageish_init(payload as *mut c_void, 0);
        let m = &mut *(payload as *mut CppImageish);
        m.height = 16;
        m.width = 64;
        rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
        rmw_cerulion_cppstring_construct(
            m.encoding.0.as_mut_ptr() as *mut c_void,
            encoding.as_ptr() as *const c_char,
            encoding.len(),
        );
        let data_abs = payload as usize + tail;
        std::ptr::copy_nonoverlapping(
            payload_bytes.as_ptr(),
            data_abs as *mut u8,
            payload_bytes.len(),
        );
        // The empty vector owns nothing, so overwriting its triplet leaks
        // nothing: {begin, end, cap_end} — a bump-filled vector's exact
        // shape (capacity == size: every growth path reallocates).
        m.data.0 = [
            data_abs,
            data_abs + payload_bytes.len(),
            data_abs + payload_bytes.len(),
        ];
        let window = WindowExtent {
            base: data_abs,
            cursor: data_abs + payload_bytes.len(),
        };

        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        assert_eq!(
            (seal.adopted, seal.escaped, seal.copied),
            (1, 0, 1),
            "the vector adopts, the string copies"
        );
        assert_eq!(
            CPP_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "fini (the C++ destructor) ran — surviving it proves the \
             adopted triplet was emptied first (~vector would otherwise \
             operator-delete the slot bytes)"
        );
        let data_off = tail;
        let enc_off = tail + payload_bytes.len();
        assert_eq!(seal.total_payload, enc_off + encoding.len());

        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        // Fixed section.
        assert_eq!(&got[0..4], &16u32.to_le_bytes());
        assert_eq!(&got[4..8], &64u32.to_le_bytes());
        // Offset table: encoding is entry 0, data entry 1 (declaration
        // order).
        let e0_off = u32::from_le_bytes(got[8..12].try_into().unwrap()) as usize;
        let e0_len = u32::from_le_bytes(got[12..16].try_into().unwrap()) as usize;
        let e1_off = u32::from_le_bytes(got[16..20].try_into().unwrap()) as usize;
        let e1_len = u32::from_le_bytes(got[20..24].try_into().unwrap()) as usize;
        assert_eq!((e0_off, e0_len), (enc_off, encoding.len()));
        assert_eq!((e1_off, e1_len), (data_off, payload_bytes.len()));
        // The adopted bytes never moved; the copied string sits above the
        // cursor; the remnant zeroes.
        assert_eq!(
            &got[data_off..data_off + payload_bytes.len()],
            &payload_bytes[..]
        );
        assert_eq!(&got[enc_off..enc_off + encoding.len()], encoding.as_bytes());
        assert!(got[CPP_FLOOR..tail].iter().all(|&b| b == 0));

        // Round-trip through the C++ copying decoder.
        let mut target: CppImageish = std::mem::zeroed();
        imageish_init(&mut target as *mut _ as *mut c_void, 0);
        assert!(bridge.unflatten(got, &mut target as *mut _ as *mut c_void));
        assert_eq!((target.height, target.width), (16, 64));
        assert_eq!(
            cpp_string_of(target.encoding.0.as_ptr() as *const c_void),
            encoding
        );
        let vec = target.data.0.as_ptr() as *const c_void;
        assert_eq!(rmw_cerulion_vector_u8_size(vec), payload_bytes.len());
        let seen =
            std::slice::from_raw_parts(rmw_cerulion_vector_u8_data(vec), payload_bytes.len());
        assert_eq!(seen, &payload_bytes[..]);
        imageish_fini(&mut target as *mut _ as *mut c_void);
    }
}

#[test]
#[serial]
fn cpp_escaped_vector_is_copied_and_the_struct_survives_a_refusal() {
    let bridge = imageish_bridge();
    let geo = bridge.borrow_geometry().expect("geometry");
    let tail = cpp_tail_off();
    let payload_bytes: Vec<u8> = (0..64u8).collect();
    unsafe {
        // Escapee: a REAL heap-backed vector (assign) with a window that
        // never saw it.
        let mut slot = Slot::new();
        let payload = slot.payload();
        imageish_init(payload as *mut c_void, 0);
        let m = &mut *(payload as *mut CppImageish);
        m.height = 2;
        m.width = 3;
        rmw_cerulion_vector_u8_assign(
            m.data.0.as_mut_ptr() as *mut c_void,
            payload_bytes.as_ptr(),
            payload_bytes.len(),
        );
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail,
        };
        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        assert_eq!((seal.adopted, seal.escaped, seal.copied), (0, 1, 1));
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        let e1_off = u32::from_le_bytes(got[16..20].try_into().unwrap()) as usize;
        assert_eq!(
            &got[e1_off..e1_off + payload_bytes.len()],
            &payload_bytes[..]
        );
        assert_eq!(
            CPP_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "the destructor freed the escaped heap vector"
        );

        // Refusal pre-mutation: a too-short payload refuses and the struct
        // (heap vector included) survives for the fallback flatten.
        let mut slot2 = Slot::new();
        let payload2 = slot2.payload();
        imageish_init(payload2 as *mut c_void, 0);
        let m2 = &mut *(payload2 as *mut CppImageish);
        m2.height = 9;
        // Every ORIGINAL field carries a distinct non-init value so the
        // "struct intact" claim below is total — a refusal that zeroed or
        // clobbered any one of them cannot pass on the others.
        m2.width = 77;
        rmw_cerulion_cppstring_destruct(m2.encoding.0.as_mut_ptr() as *mut c_void);
        rmw_cerulion_cppstring_construct(
            m2.encoding.0.as_mut_ptr() as *mut c_void,
            b"mono8".as_ptr() as *const c_char,
            5,
        );
        rmw_cerulion_vector_u8_assign(
            m2.data.0.as_mut_ptr() as *mut c_void,
            payload_bytes.as_ptr(),
            payload_bytes.len(),
        );
        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        let refusal = bridge
            .seal_borrowed_frame(payload2, tail + 8, &geo, None, GAP_OK, &mut scratch())
            .expect_err("must refuse");
        assert!(matches!(refusal, SealRefusal::Overflow { .. }));
        assert_eq!(CPP_FINI_CALLS.load(Ordering::SeqCst), 0);
        // The fallback frame decodes to EXACTLY the original fields (a
        // refusal that corrupted the struct would pass a nonempty-check).
        let frame = bridge
            .flatten(payload2 as *const c_void, 0, 0)
            .expect("fallback flatten still works");
        let mut target: CppImageish = std::mem::zeroed();
        imageish_init(&mut target as *mut _ as *mut c_void, 0);
        assert!(bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut target as *mut _ as *mut c_void
        ));
        assert_eq!(target.height, 9, "fixed field survives the refusal");
        assert_eq!(target.width, 77, "second fixed field survives the refusal");
        assert_eq!(
            cpp_string_of(target.encoding.0.as_ptr() as *const c_void),
            "mono8",
            "string field survives the refusal"
        );
        let vec = target.data.0.as_ptr() as *const c_void;
        assert_eq!(rmw_cerulion_vector_u8_size(vec), payload_bytes.len());
        let seen =
            std::slice::from_raw_parts(rmw_cerulion_vector_u8_data(vec), payload_bytes.len());
        assert_eq!(
            seen,
            &payload_bytes[..],
            "sequence bytes survive the refusal"
        );
        imageish_fini(&mut target as *mut _ as *mut c_void);
        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        bridge.fini_borrowed_payload(payload2 as *mut c_void, PAYLOAD_LEN);
        assert_eq!(CPP_FINI_CALLS.load(Ordering::SeqCst), 1);
    }
}

// =====================================================================
// The windowed-borrow ceiling gate (pure; the create path's predicate)
// =====================================================================

/// The borrow writes `WireHeader + tail_off` unconditionally, so a slice
/// ceiling below that must turn the windowed surface OFF (constructing in
/// an undersized loan would write past it) — pinned on BOTH sides of the
/// boundary, plus the hook gate and the fixed-type control.
#[test]
fn windowed_gate_requires_the_ceiling_to_hold_the_slot_geometry() {
    let bridge = AnyBridge::C(scanish_bridge());
    let need = WireHeader::SIZE + c_tail_off();
    // No hook ⇒ never offered, however generous the ceiling.
    assert!(rmw_cerulion::windowed_borrow_geometry(&bridge, usize::MAX, false).is_none());
    // The MaxSliceLen legal minimum (a bare WireHeader) must refuse.
    assert!(
        rmw_cerulion::windowed_borrow_geometry(&bridge, WireHeader::SIZE, true).is_none(),
        "a 32-byte ceiling cannot hold the struct"
    );
    // Boundary, both sides.
    assert!(
        rmw_cerulion::windowed_borrow_geometry(&bridge, need - 1, true).is_none(),
        "one byte short of the geometry must refuse"
    );
    let geo = rmw_cerulion::windowed_borrow_geometry(&bridge, need, true)
        .expect("a ceiling that exactly holds WireHeader + tail_off is offered");
    assert_eq!(geo.tail_off, c_tail_off());
    assert_eq!(geo.data_floor, C_FLOOR);
}

// =====================================================================
// The counting oracle: empty members count by FORGEABILITY
// =====================================================================

/// Scanish plus a BOUNDED (non-forgeable) `uint8[<=8]` member — the shape
/// whose empty member must NOT inflate the adopted count.
#[repr(C)]
struct CScanCaps {
    angle_min: f32,
    ranges: CF32Seq,
    caps: CU8Seq,
    frame_id: CRosString,
}

#[repr(C)]
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

unsafe extern "C" fn scancaps_init(
    msg: *mut c_void,
    _init: rosidl_runtime_c__message_initialization,
) {
    let m = &mut *(msg as *mut CScanCaps);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

unsafe extern "C" fn scancaps_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CScanCaps);
    for ptr in [
        m.frame_id.data as *mut c_void,
        m.ranges.data as *mut c_void,
        m.caps.data as *mut c_void,
    ] {
        if !ptr.is_null() {
            free(ptr);
        }
    }
    m.frame_id.data = std::ptr::null_mut();
    m.ranges.data = std::ptr::null_mut();
    m.caps.data = std::ptr::null_mut();
}

fn scancaps_bridge() -> BridgedMessage {
    let mut caps = c_sequence(
        "caps",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CScanCaps, caps) as u32,
    );
    caps.is_upper_bound_ = true;
    caps.array_size_ = 8;
    let members = vec![
        c_member("angle_min", ROS_TYPE_FLOAT, 0),
        c_sequence(
            "ranges",
            ROS_TYPE_FLOAT,
            std::mem::offset_of!(CScanCaps, ranges) as u32,
        ),
        caps,
        c_member(
            "frame_id",
            ROS_TYPE_STRING,
            std::mem::offset_of!(CScanCaps, frame_id) as u32,
        ),
    ];
    let members = Box::leak(members.into_boxed_slice());
    let members = Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("seal_test__msg"),
            message_name_: cstr("ScanCaps"),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScanCaps>(),
            members_: members.as_ptr(),
            init_function: Some(scancaps_init),
            fini_function: Some(scancaps_fini),
            ..Default::default()
        },
    ));
    unsafe { BridgedMessage::new(members) }.expect("bridge")
}

#[test]
#[serial]
fn empty_members_count_by_forgeability_never_inflating_adopted() {
    let bridge = scancaps_bridge();
    // Control: exactly ONE forgeable member (`ranges`); the bounded `caps`
    // is not adoption-eligible.
    assert_eq!(bridge.forged_sequence_count(), 1);
    let floor = 4 + 3 * 8; // fixed f32 + three offset-table entries
    let tail = align_up(
        std::mem::size_of::<CScanCaps>().max(floor),
        BORROW_TAIL_ALIGN,
    );
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        let m = &mut *(payload as *mut CScanCaps);
        m.angle_min = 2.0;
        m.ranges = CF32Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        m.caps = CU8Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        let s = calloc(2, 1) as *mut u8;
        *s = b'e';
        m.frame_id = CRosString {
            data: s,
            size: 1,
            capacity: 2,
        };
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail,
        };
        let geo = bridge.borrow_geometry().expect("geometry");
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal");
        // THE counting oracle: the empty FORGEABLE `ranges` is trivially
        // adopted; the empty NON-forgeable `caps` is a zero-byte structural
        // copy; the string copies. An implementation counting every empty
        // member as adopted reads (2, 0, 1) here and fails.
        assert_eq!(
            (seal.adopted, seal.escaped, seal.copied),
            (1, 0, 2),
            "empty members must count by forgeability — adopted is the \
             number an operator diagnoses degrades with"
        );
        // Both empty entries anchor at the floor; the string is placed.
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        for idx in 0..2 {
            let base = 4 + idx * 8;
            let off = u32::from_le_bytes(got[base..base + 4].try_into().unwrap()) as usize;
            let len = u32::from_le_bytes(got[base + 4..base + 8].try_into().unwrap()) as usize;
            assert_eq!(
                (off, len),
                (floor, 0),
                "empty entry {idx} anchors at the floor"
            );
        }
        let s_off = u32::from_le_bytes(got[20..24].try_into().unwrap()) as usize;
        let s_len = u32::from_le_bytes(got[24..28].try_into().unwrap()) as usize;
        assert_eq!((s_off, s_len), (tail, 1));
        assert_eq!(&got[s_off..s_off + 1], b"e");
    }
}

// =====================================================================
// AnyBridge dispatch (the seam pubsub drives)
// =====================================================================

#[test]
#[serial]
fn any_bridge_dispatch_serves_the_c_seal() {
    let bridge = AnyBridge::C(scanish_bridge());
    assert!(bridge.can_borrow_windowed());
    let geo = bridge.borrow_geometry().expect("geometry");
    let tail = c_tail_off();
    let ranges = [42.0f32];
    unsafe {
        let mut slot = Slot::new();
        let payload = slot.payload();
        let ranges_ptr = payload.add(tail) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), ranges_ptr, 1);
        build_scanish_in_slot(payload, 7.0, ranges_ptr, 1, "d");
        let window = WindowExtent {
            base: payload as usize + tail,
            cursor: payload as usize + tail + 4,
        };
        let seal = bridge
            .seal_borrowed_frame(
                payload,
                PAYLOAD_LEN,
                &geo,
                Some(window),
                GAP_OK,
                &mut scratch(),
            )
            .expect("seal through the enum");
        assert_eq!(seal.adopted, 1);
        let got = std::slice::from_raw_parts(payload, seal.total_payload);
        assert_eq!(&got[tail..tail + 4], &42.0f32.to_le_bytes());
    }
}

extern "C" {
    /// The shim's `std::vector<uint8_t>::assign` — `pub(crate)` in the
    /// crate on purpose; the fixture declares its own binding to fill a
    /// REAL vector (same idiom as `forged_take_bridge_test.rs`).
    fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Zero copy across the plugin boundary, proven by ADDRESS.
//!
//! Every other cdylib test in the tree proves PARITY: the payload a plugin
//! produces over the FFI is byte identical to the payload an in process twin
//! produces (`cdylib_portwrite_e2e_test`). A copy produces byte identical
//! payloads too, so parity cannot see the difference between a plugin that
//! writes straight into the publisher's shared memory and one that fills a heap
//! buffer and marshals it across. This file asks the question parity cannot:
//! WHERE did the plugin write?
//!
//! The plugin records the virtual address of the output slot it writes through
//! (`test_node_macro_clockprobe_cdylib` under `CER_ZC_ADDR_PROBE`), the host
//! reads that address back through the fixture's `#[no_mangle]` accessor, and
//! the host asks the OPERATING SYSTEM which mapping contains it:
//!
//! * Linux reports the mapping's pathname in `/proc/self/maps`. The publisher's
//!   iceoryx2 data segment is a `MAP_SHARED` mapping of a `/dev/shm` object
//!   whose name carries this run's iceoryx2 `global.prefix` and the configured
//!   `data_segment_suffix`, so the pathname NAMES the segment outright.
//! * macOS has no pathname for a POSIX shared memory object (`proc_pidinfo`
//!   returns a vnode path only for file backed mappings, and iceoryx2's macOS
//!   layer maps a `shm_open` object). What it does report is the VM object id,
//!   which is IDENTICAL for every mapping of one object, so the segment is
//!   named by matching the writer's region against the region the READER's view
//!   of the same frame lands in.
//!
//! Both platforms then run the same three assertions, and the control that
//! makes them mean something:
//!
//! 1. **plugin arm**: the address the plugin wrote through is inside a shared
//!    memory mapping, inside the SAME OS memory object the host reads the
//!    delivered frame out of, and at the SAME position inside that object. The
//!    position matters because object identity alone would admit a producer that
//!    wrote one slot and moved the bytes to another slot of the same segment
//!    before publishing, which is still a copy. Both sides therefore name the
//!    FIRST frame of the run, so the comparison is exact.
//! 2. **in process twin**: a non cdylib node whose tick mirrors the fixture's
//!    probe branch passes the SAME checker, so the checker is not accidentally
//!    passing on something specific to the FFI path.
//! 3. **positive controls**: the payload copied into a heap buffer FAILS the same
//!    checker, and a frame read from a NEIGHBOURING slot of the same segment fails
//!    the position arm, both with the checker's own panic captured, so neither
//!    check can degrade into an unconditional pass unnoticed.
//!
//! The delivered bytes carry a hand oracle as well: `angular.z` must arrive as
//! the exact bit pattern the fixture stamps (`PROBE_MARKER_BITS`, written here
//! as an independent literal, never read back out of the fixture), and
//! `angular.x` must arrive as 0.0 because no external clock is wired.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_clockprobe_cdylib`. All tests
//! `#[serial]`: the cdylib `NODES` singleton, the fixture's process global
//! address slot, and the process global `CER_ZC_ADDR_PROBE` env var (paired with
//! an `EnvVarGuard` RAII).
//!
//! # Windows
//!
//! The mapping query is POSIX only, so the file compiles to nothing off Linux
//! and macOS. Windows has no cdylib node support in this tree today.

#![cfg(any(target_os = "linux", target_os = "macos"))]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Twist;
use serial_test::serial;

/// The bit pattern `test_node_macro_clockprobe_cdylib` stamps into `angular.z`
/// under the probe. Declared HERE as an independent literal: the oracle for the
/// delivered bytes is this constant, never a value read back out of the fixture.
const PROBE_MARKER_BITS: u64 = 0x5A5A_C0FF_EE00_1234;

// ---------------------------------------------------------------------------
// RAII env guard (the cdylib_portwrite_e2e_test pattern)
// ---------------------------------------------------------------------------

struct EnvVarGuard {
    key: &'static str,
}
impl EnvVarGuard {
    fn set(key: &'static str, val: &str) -> Self {
        std::env::set_var(key, val);
        Self { key }
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.key);
    }
}

// ===========================================================================
// The checker: what the OS says about the mapping holding an address.
// ===========================================================================

/// One virtual memory mapping of this process, as the OS reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct MappedRegion {
    /// First address of the mapping.
    base: usize,
    /// Length of the mapping in bytes.
    size: usize,
    /// The OS says this mapping is backed by a SHARED memory object rather than
    /// private (heap, stack, anonymous) pages. Linux: the `perms` token's last
    /// character is `s`. macOS: `pri_share_mode` is one of the shared modes.
    shared: bool,
    /// Identity of the underlying OS memory object, equal for every mapping of
    /// the same object in this process. Linux: the `dev:inode` pair. macOS: the
    /// `pri_obj_id` the kernel assigns the VM object.
    object: String,
    /// What the OS calls the mapping. Linux: the pathname (empty for an
    /// anonymous mapping). macOS: the share mode and allocation tag, since a
    /// POSIX shared memory object has no vnode path.
    os_label: String,
    /// Offset INTO the backing object at which this mapping starts. Linux: the
    /// `offset` column of `/proc/self/maps`. macOS: `pri_offset`. Added to an
    /// address's distance from `base` it gives that address's position inside the
    /// object, which is comparable across two mappings of one object even when
    /// the kernel has split a mapping into several regions.
    object_offset: usize,
}

impl MappedRegion {
    /// Position of `addr` inside the backing OBJECT, not inside this mapping.
    fn offset_of(&self, addr: usize) -> usize {
        (addr - self.base) + self.object_offset
    }
}

/// Where an address lives, as far as the OS is concerned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// Inside a mapping the OS reports as shared memory.
    SharedMemory,
    /// Inside a mapping the OS reports as private: heap, stack, anonymous.
    PrivateMemory,
    /// Inside no mapping at all.
    Unmapped,
}

/// The mapping containing `addr`, or `None` when no mapping does.
///
/// Linux reads `/proc/self/maps` with the same column discipline the production
/// `transport::shm_guard::shm_vma_ranges` scan uses (range, perms, offset, dev,
/// inode, then a pathname that may itself contain spaces).
#[cfg(target_os = "linux")]
fn region_of(addr: usize) -> Option<MappedRegion> {
    let maps = std::fs::read_to_string("/proc/self/maps")
        .expect("Linux: /proc/self/maps must be readable to locate a mapping");
    for line in maps.lines() {
        let mut fields = line.split_whitespace();
        let (Some(range), Some(perms), Some(offset), Some(dev), Some(inode)) = (
            fields.next(),
            fields.next(),
            fields.next(),
            fields.next(),
            fields.next(),
        ) else {
            continue;
        };
        let Ok(object_offset) = usize::from_str_radix(offset, 16) else {
            continue;
        };
        let Some((lo, hi)) = range.split_once('-') else {
            continue;
        };
        let (Ok(base), Ok(end)) = (usize::from_str_radix(lo, 16), usize::from_str_radix(hi, 16))
        else {
            continue;
        };
        if addr < base || addr >= end {
            continue;
        }
        // Everything from the sixth field on is the pathname, which may contain
        // spaces, so rejoin rather than take the first token.
        let path: Vec<&str> = fields.collect();
        return Some(MappedRegion {
            base,
            size: end - base,
            shared: perms.ends_with('s'),
            object: format!("{dev}:{inode}"),
            os_label: path.join(" "),
            object_offset,
        });
    }
    None
}

/// Darwin's region query. `proc_pidinfo` ships in libSystem, which every Rust
/// binary on this target already links, so the declarations below add no
/// dependency. Layout mirrors `<sys/proc_info.h>`'s `struct proc_regioninfo`;
/// the call is rejected below unless the kernel writes back exactly that many
/// bytes, which fails loudly rather than silently misreading a changed layout.
#[cfg(target_os = "macos")]
mod darwin {
    /// `PROC_PIDREGIONINFO`.
    pub const FLAVOR_REGION_INFO: i32 = 7;
    /// `SM_SHARED`, `SM_TRUESHARED`, `SM_SHARED_ALIASED` from `<mach/vm_region.h>`.
    pub const SHARED_MODES: [u32; 3] = [4, 5, 7];

    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    pub struct ProcRegionInfo {
        pub protection: u32,
        pub max_protection: u32,
        pub inheritance: u32,
        pub flags: u32,
        pub offset: u64,
        pub behavior: u32,
        pub user_wired_count: u32,
        pub user_tag: u32,
        pub pages_resident: u32,
        pub pages_shared_now_private: u32,
        pub pages_swapped_out: u32,
        pub pages_dirtied: u32,
        pub ref_count: u32,
        pub shadow_depth: u32,
        pub share_mode: u32,
        pub private_pages_resident: u32,
        pub shared_pages_resident: u32,
        pub obj_id: u32,
        pub depth: u32,
        pub address: u64,
        pub size: u64,
    }

    extern "C" {
        pub fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut core::ffi::c_void,
            buffersize: i32,
        ) -> i32;
    }
}

/// The mapping containing `addr`, or `None` when no mapping does.
///
/// `PROC_PIDREGIONINFO` returns the first region at OR ABOVE `arg`, so
/// containment is checked rather than assumed.
#[cfg(target_os = "macos")]
fn region_of(addr: usize) -> Option<MappedRegion> {
    let mut info = darwin::ProcRegionInfo::default();
    let want = core::mem::size_of::<darwin::ProcRegionInfo>();
    // SAFETY: `info` is a live, correctly sized, correctly aligned buffer of
    // exactly `want` bytes, and the kernel writes at most that many into it.
    let got = unsafe {
        darwin::proc_pidinfo(
            std::process::id() as i32,
            darwin::FLAVOR_REGION_INFO,
            addr as u64,
            core::ptr::from_mut(&mut info).cast(),
            want as i32,
        )
    };
    if got <= 0 {
        return None;
    }
    assert_eq!(
        got as usize, want,
        "proc_pidinfo wrote {got} bytes for PROC_PIDREGIONINFO but \
         `struct proc_regioninfo` is {want} bytes here: the layout this test \
         mirrors has changed and every reading below it is untrustworthy"
    );
    let base = info.address as usize;
    let size = info.size as usize;
    if addr < base || addr >= base + size {
        return None;
    }
    Some(MappedRegion {
        base,
        size,
        shared: darwin::SHARED_MODES.contains(&info.share_mode),
        object: format!("obj_id=0x{:x}", info.obj_id),
        os_label: format!(
            "share_mode={} user_tag=0x{:x} ref_count={}",
            info.share_mode, info.user_tag, info.ref_count
        ),
        object_offset: info.offset as usize,
    })
}

/// Where the OS says `addr` lives, plus the mapping it landed in.
fn classify(addr: usize) -> (Verdict, Option<MappedRegion>) {
    match region_of(addr) {
        None => (Verdict::Unmapped, None),
        Some(r) if r.shared => (Verdict::SharedMemory, Some(r)),
        Some(r) => (Verdict::PrivateMemory, Some(r)),
    }
}

/// THE checker. Panics unless `addr` lies inside a mapping the OS reports as
/// shared memory, and on Linux unless that mapping's pathname names the
/// publisher's iceoryx2 data segment for THIS run.
///
/// Shared by the plugin arm, the in process twin and the copy control, so all
/// three are judged by one rule.
///
/// `shm_prefix` is the run's `global.prefix` and `data_suffix` its
/// `global.service.data_segment_suffix`, both read off the transport the graph
/// actually built, so the pathname match cannot pass on an unrelated shared
/// mapping (another run's segment, a mapped font, the loader's own file maps).
fn assert_in_publisher_shm(
    addr: usize,
    who: &str,
    shm_prefix: &str,
    data_suffix: &str,
) -> MappedRegion {
    let (verdict, region) = classify(addr);
    let region = region.unwrap_or_else(|| {
        panic!("{who}: address 0x{addr:x} lies in no mapping at all (verdict {verdict:?})")
    });
    assert_eq!(
        verdict,
        Verdict::SharedMemory,
        "{who}: address 0x{addr:x} is in a {verdict:?} mapping \
         (base=0x{:x} size=0x{:x} os={}), not in shared memory. A payload written \
         through a heap or stack buffer and copied out lands exactly here.",
        region.base,
        region.size,
        region.os_label,
    );
    if cfg!(target_os = "linux") {
        assert!(
            region.os_label.contains(shm_prefix) && region.os_label.contains(data_suffix),
            "{who}: address 0x{addr:x} is in the shared mapping `{}`, which is not \
             this run's iceoryx2 data segment (expected a pathname carrying the \
             prefix `{shm_prefix}` and the suffix `{data_suffix}`)",
            region.os_label,
        );
    }
    region
}

/// Both addresses lie in mappings of ONE OS memory object.
///
/// This is what names the segment on macOS, where a POSIX shared memory object
/// has no pathname: the writer's region and the reader's region carry the same
/// VM object id. On Linux it is the `dev:inode` pair, and it holds for the same
/// reason. iceoryx2 maps a publisher's data segment once per port, so the two
/// addresses are usually different while the object behind them is one.
fn assert_same_os_object(writer: &MappedRegion, reader: &MappedRegion, who: &str) {
    assert_eq!(
        writer.object, reader.object,
        "{who}: the slot the producer wrote (base=0x{:x} object={} os={}) and the \
         slot the consumer read the delivered frame out of (base=0x{:x} object={} \
         os={}) are different OS memory objects, so the payload was copied \
         between them",
        writer.base, writer.object, writer.os_label, reader.base, reader.object, reader.os_label,
    );
}

/// Both addresses sit at the SAME position inside that one object.
///
/// Object identity alone cannot see a copy that stays INSIDE the segment: a
/// producer that wrote slot A and then moved the bytes to slot B before
/// publishing would satisfy it, because both slots belong to the same object.
/// The position is what excludes that. It is compared as an offset into the
/// OBJECT rather than as a raw address, because a publisher and a consumer map
/// the same segment at different base addresses.
///
/// Both sides name the FIRST frame of the run, which is what makes an exact
/// comparison meaningful: the producer records only its first probed tick and the
/// consumer keeps only its first delivered frame, so the two describe one frame
/// rather than whichever frame each happened to see last.
fn assert_same_object_offset(
    writer_addr: usize,
    writer: &MappedRegion,
    reader_addr: usize,
    reader: &MappedRegion,
    who: &str,
) {
    let w = writer.offset_of(writer_addr);
    let r = reader.offset_of(reader_addr);
    assert_eq!(
        w, r,
        "{who}: the producer wrote at offset 0x{w:x} into the shared memory \
         object, and the consumer read the delivered frame at offset 0x{r:x} \
         into that same object, so the payload was moved between two slots of \
         one segment, which is a copy that object identity alone cannot see \
         (writer base=0x{:x} addr=0x{writer_addr:x}, reader base=0x{:x} \
         addr=0x{reader_addr:x})",
        writer.base, reader.base,
    );
}

// ===========================================================================
// Graph: a Twist producer feeding an in process capture consumer.
// ===========================================================================

/// What the consumer saw: the delivered bytes plus the address it read them at.
#[derive(Clone, Debug, PartialEq, cerulion_core::state::CerulionState)]
struct Captured {
    angular_z_bits: u64,
    angular_x_bits: u64,
    /// Address of the `angular.z` slot in the consumer's view of the frame.
    read_addr: usize,
}

/// Data triggered consumer of the probe's `Twist`: records the delivered bits
/// and the address its view of them lives at.
#[cerulion_node]
#[derive(Default)]
struct TwistCapture {
    #[input(trigger)]
    probe: Twist,
    captured: Arc<Mutex<Option<Captured>>>,
}
#[cerulion_node_impl]
impl TwistCapture {
    fn tick(&mut self) -> Result<(), NodeError> {
        let cap = Captured {
            angular_z_bits: self.probe.angular.z.to_bits(),
            angular_x_bits: self.probe.angular.x.to_bits(),
            read_addr: std::ptr::from_ref(&self.probe.angular.z) as usize,
        };
        // Keep the FIRST delivered frame only: the producer records the position
        // of its first write, so the offset comparison has to be against the same
        // frame, not against whichever frame arrived last.
        let mut slot = self.captured.lock().unwrap();
        if slot.is_none() {
            *slot = Some(cap);
        }
        Ok(())
    }
}

/// In process twin of the cdylib fixture's PROBE branch. Same schema, same
/// slot, same recorded address, no FFI: if the checker passes the plugin arm it
/// must pass this one too, and if it fails here the fault is the checker's, not
/// the plugin path's.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct TwistProbeTwin {
    #[output]
    out: Twist,
    write_addr: Arc<Mutex<usize>>,
}
#[cerulion_node_impl]
impl TwistProbeTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.angular.x = 0.0;
        self.out.angular.z = f64::from_bits(PROBE_MARKER_BITS);
        let slot = std::ptr::from_ref(&self.out.angular.z) as usize;
        // First tick only, mirroring the fixture's `compare_exchange` from 0.
        let mut recorded = self.write_addr.lock().unwrap();
        if *recorded == 0 {
            *recorded = slot;
        }
        Ok(())
    }
}

/// What one graph run observed.
struct RunOutcome {
    captured: Option<Captured>,
    fire_count: u64,
    /// `global.prefix` of the isolated iceoryx2 namespace this run built on.
    shm_prefix: String,
    /// `global.service.data_segment_suffix` of the same namespace.
    data_suffix: String,
    /// Kept alive so the mappings the addresses name are still valid while the
    /// caller queries the OS about them.
    _runtime: GraphRuntime,
}

/// Build `probe(Twist out) -> capture(Twist trigger)` over an isolated per test
/// iceoryx2 root, run `steps` ticks, and hand back the run WITHOUT dropping it.
fn run_probe_graph(probe: Box<dyn NodeEntry>, prefix: &str, steps: usize) -> RunOutcome {
    let captured: Arc<Mutex<Option<Captured>>> = Arc::new(Mutex::new(None));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("zcaddr_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "probe".to_string(),
                node_type: "twist_probe".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Twist".to_string(),
                    max_slice_len: Some(4096),
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "capture".to_string(),
                node_type: "twist_capture".to_string(),
                inputs: vec![InputDef {
                    name: "probe".to_string(),
                    source: "probe/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("probe".to_string(), probe);
    let capture_node = TwistCapture {
        captured: Arc::clone(&captured),
        ..Default::default()
    };
    factories.insert(
        "capture".to_string(),
        Box::new(TwistCaptureEntry::with_state(capture_node)),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build probe graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let fire_count = runtime.node_handle("probe").unwrap().fire_count();
    let result = captured.lock().unwrap().clone();
    let ix = runtime
        .test_transport()
        .expect("build_for_test parks its isolated transport on the runtime")
        .iox_shm_identity();
    RunOutcome {
        captured: result,
        fire_count,
        shm_prefix: ix.prefix,
        data_suffix: ix.service_data_segment_suffix,
        _runtime: runtime,
    }
}

// ===========================================================================
// Fixture handle: the probe accessors live in the .so, read via a second dlopen
// of the same path (the process global static is shared).
// ===========================================================================

fn clockprobe_path() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_clockprobe_cdylib")
}

fn clockprobe_cdylib() -> Box<dyn NodeEntry> {
    Box::new(DylibNodeEntry::load(&clockprobe_path()).expect("load clockprobe fixture"))
}

/// Load the fixture through our OWN `libloading` handle so we can call its probe
/// accessors. A second `dlopen` of the same path shares the object's data
/// segment (mapped once, reference counted), so this handle's view of the
/// address slot IS the one the runtime's `DylibNodeEntry` wrote.
fn load_accessor_handle() -> libloading::Library {
    unsafe { libloading::Library::new(clockprobe_path()) }
        .expect("load clockprobe cdylib for probe accessors")
}

fn reset_shm_write_addr(lib: &libloading::Library) {
    let f: libloading::Symbol<unsafe extern "C" fn()> = unsafe {
        lib.get(b"cerulion_test_reset_shm_write_addr\0")
            .expect("missing cerulion_test_reset_shm_write_addr symbol")
    };
    unsafe { f() };
}

fn get_shm_write_addr(lib: &libloading::Library) -> usize {
    let f: libloading::Symbol<unsafe extern "C" fn(u64) -> u64> = unsafe {
        lib.get(b"cerulion_test_get_shm_write_addr\0")
            .expect("missing cerulion_test_get_shm_write_addr symbol")
    };
    // handle ignored by the fixture (the slot is process global).
    unsafe { f(0) as usize }
}

// ===========================================================================
// 1. The plugin arm.
// ===========================================================================

/// The address a REAL cdylib node writes its output payload through lies inside
/// the publisher's shared memory segment, and inside the same OS memory object
/// the host reads the delivered frame out of.
#[test]
#[serial]
fn cdylib_tick_writes_through_an_address_inside_the_publisher_shm_segment() {
    let _guard = EnvVarGuard::set("CER_ZC_ADDR_PROBE", "1");
    let lib = load_accessor_handle();
    reset_shm_write_addr(&lib);

    let run = run_probe_graph(clockprobe_cdylib(), "zca", 4);
    let got = run
        .captured
        .clone()
        .expect("the probe node must publish a Twist frame");
    assert!(run.fire_count > 0, "the cdylib probe node must have fired");

    // Hand oracle on the bytes: the fixture stamps this exact pattern and no
    // external clock is wired, so both values are known without reading the
    // fixture back. Bytes alone do not prove zero copy; they prove the frame
    // under test is the frame the probe produced.
    assert_eq!(
        got.angular_z_bits, PROBE_MARKER_BITS,
        "the delivered angular.z must carry the probe's exact bit pattern"
    );
    assert_eq!(
        got.angular_x_bits,
        0.0f64.to_bits(),
        "no external clock is wired, so angular.x must arrive as 0.0"
    );

    let write_addr = get_shm_write_addr(&lib);
    assert_ne!(
        write_addr, 0,
        "the probed tick must have recorded the address it wrote through; 0 means \
         the probe branch never ran (is CER_ZC_ADDR_PROBE reaching the cdylib?)"
    );

    let writer = assert_in_publisher_shm(
        write_addr,
        "cdylib plugin write",
        &run.shm_prefix,
        &run.data_suffix,
    );
    let reader = assert_in_publisher_shm(
        got.read_addr,
        "host read of the delivered frame",
        &run.shm_prefix,
        &run.data_suffix,
    );
    assert_same_os_object(&writer, &reader, "cdylib plugin write vs host read");
    assert_same_object_offset(
        write_addr,
        &writer,
        got.read_addr,
        &reader,
        "cdylib plugin write vs host read",
    );

    // Control for the offset arm itself: a DIFFERENT slot in the SAME object must
    // be rejected. Without this, an offset check that had collapsed into a
    // tautology would keep the arm above green while admitting an intra segment
    // copy, which object identity alone cannot see.
    let other_slot = got.read_addr + std::mem::size_of::<f64>();
    assert!(
        reader.offset_of(other_slot) != reader.offset_of(got.read_addr),
        "the neighbouring slot must sit at a different object offset"
    );
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let moved = std::panic::catch_unwind({
        let (w, r) = (writer.clone(), reader.clone());
        move || assert_same_object_offset(write_addr, &w, other_slot, &r, "intra segment copy")
    });
    std::panic::set_hook(previous);
    assert!(
        moved.is_err(),
        "the offset arm must REJECT a frame read from a different slot of the \
         same shared memory object: that is an intra segment copy"
    );
}

// ===========================================================================
// 2. The in process twin, judged by the same checker.
// ===========================================================================

/// A non cdylib node writing the same slot passes the same checker, so a
/// failure on the plugin arm is attributable to the FFI path rather than to the
/// checker or to the transport.
#[test]
#[serial]
fn in_process_twin_writes_through_an_address_inside_the_publisher_shm_segment() {
    let write_addr: Arc<Mutex<usize>> = Arc::new(Mutex::new(0));
    let twin = TwistProbeTwin {
        write_addr: Arc::clone(&write_addr),
        ..Default::default()
    };
    let run = run_probe_graph(Box::new(TwistProbeTwinEntry::with_state(twin)), "zct", 4);
    let got = run
        .captured
        .clone()
        .expect("the twin must publish a Twist frame");
    assert!(run.fire_count > 0, "the twin node must have fired");
    assert_eq!(
        got.angular_z_bits, PROBE_MARKER_BITS,
        "the twin writes the same marker the fixture does"
    );

    let addr = *write_addr.lock().unwrap();
    assert_ne!(addr, 0, "the twin must have recorded its write address");

    let writer =
        assert_in_publisher_shm(addr, "in process write", &run.shm_prefix, &run.data_suffix);
    let reader = assert_in_publisher_shm(
        got.read_addr,
        "host read of the delivered frame",
        &run.shm_prefix,
        &run.data_suffix,
    );
    assert_same_os_object(&writer, &reader, "in process write vs host read");
    assert_same_object_offset(
        addr,
        &writer,
        got.read_addr,
        &reader,
        "in process write vs host read",
    );
}

// ===========================================================================
// 3. The positive control: a copy of the very same payload must FAIL.
// ===========================================================================

/// Copy the delivered payload into a heap buffer and run the SAME checker over
/// the copy's address. It must fail, and it must fail for the stated reason.
///
/// Without this arm, a checker that had quietly degraded into an unconditional
/// pass would keep both arms above green forever. The copy carries byte
/// identical content to the frame that just passed, which is the point: bytes
/// cannot separate the two, addresses can.
#[test]
#[serial]
fn a_heap_copy_of_the_same_payload_fails_the_same_checker() {
    let _guard = EnvVarGuard::set("CER_ZC_ADDR_PROBE", "1");
    let lib = load_accessor_handle();
    reset_shm_write_addr(&lib);

    let run = run_probe_graph(clockprobe_cdylib(), "zcc", 4);
    let got = run
        .captured
        .clone()
        .expect("the probe node must publish a Twist frame");
    let write_addr = get_shm_write_addr(&lib);
    assert_ne!(write_addr, 0, "the probed tick must record its address");

    // The shared memory path passes, so the control is measured against a live
    // positive rather than against nothing.
    assert_in_publisher_shm(
        write_addr,
        "cdylib plugin write",
        &run.shm_prefix,
        &run.data_suffix,
    );

    // The marshalling a copy would do: same eight bytes, different home.
    let copy: Vec<u8> = got.angular_z_bits.to_le_bytes().to_vec();
    assert_eq!(
        u64::from_le_bytes(copy.as_slice().try_into().unwrap()),
        PROBE_MARKER_BITS,
        "the copy carries byte identical content to the frame that just passed"
    );
    let copy_addr = copy.as_ptr() as usize;

    let (verdict, region) = classify(copy_addr);
    assert_eq!(
        verdict,
        Verdict::PrivateMemory,
        "a heap buffer must be reported as private memory, not {verdict:?} \
         (region {region:?})"
    );

    // Run the real checker and require it to panic. A checker that passed here
    // would pass anything.
    let prefix = run.shm_prefix.clone();
    let suffix = run.data_suffix.clone();
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let outcome = std::panic::catch_unwind(move || {
        assert_in_publisher_shm(copy_addr, "heap copy control", &prefix, &suffix);
    });
    std::panic::set_hook(previous);

    let payload = outcome.expect_err("the checker must REJECT a heap copy of the payload");
    let message = payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_string()))
        .unwrap_or_default();
    assert!(
        message.contains("not in shared memory"),
        "the checker must reject the copy for being outside shared memory, \
         got: {message}"
    );
}

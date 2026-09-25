// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion_core::dynamic` hot-path allocation gate: after
//! `FrameEncoder::new` / `SchemaSet` construction, encoding a frame and
//! validating one with `FrameView` perform ZERO heap allocations. A binding
//! runs these per message straight into a `loan_raw_uninit` slot, so the
//! zero-copy-hot-path principle applies to them as it does to the generated
//! writers.
//!
//! The counting allocator is process-global; every test here is `#[serial]`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use cerulion_core::dynamic::{FrameEncoder, FrameView, SchemaSet};
use serial_test::serial;

struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        // SAFETY: forwarding the exact `layout` to the system allocator.
        unsafe { self.inner.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from `self.inner.alloc` with this `layout`.
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

const YAML: &str = "\
schemas:
  Probe:
    fields:
      uint32 id: {}
      uint8 flag: {}
      string name: {}
      float64[] samples: {}
";

fn probe_set() -> SchemaSet {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    set.add_yaml_str(YAML).expect("fixture parses");
    set
}

#[test]
#[serial]
fn encode_after_new_does_not_allocate() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut buf = [0u8; 128];
    let lens = [2usize, 16];

    ALLOCATOR.enable();
    let n = {
        let mut cur = enc.begin(&mut buf, &lens, 7).expect("begin");
        cur.fixed_field_mut("id")
            .expect("id")
            .copy_from_slice(&1u32.to_le_bytes());
        cur.fixed_field_mut("flag").expect("flag")[0] = 1;
        cur.variable_field_mut("name")
            .expect("name")
            .copy_from_slice(b"ab");
        cur.variable_field_mut("samples").expect("samples")[..8]
            .copy_from_slice(&1.0f64.to_le_bytes());
        cur.finish()
    };
    let allocs = ALLOCATOR.disable();

    assert_eq!(n, 80);
    assert_eq!(
        allocs, 0,
        "FrameEncoder::begin..finish allocated {allocs} time(s)"
    );
}

#[test]
#[serial]
fn view_validation_and_field_access_do_not_allocate() {
    let set = probe_set();
    let layout = set.layout("Probe").expect("Probe");
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut buf = [0u8; 80];
    {
        let mut cur = enc.begin(&mut buf, &[2, 16], 7).expect("begin");
        cur.variable_field_mut("name")
            .expect("name")
            .copy_from_slice(b"ab");
        cur.finish();
    }

    ALLOCATOR.enable();
    let (id_len, name, samples) = {
        let view = FrameView::new(set.walker(), &buf).expect("valid");
        (
            view.fixed_field("id").expect("id").len(),
            view.str_field("name").expect("name"),
            view.prim_array_field("samples").expect("samples").count,
        )
    };
    let allocs = ALLOCATOR.disable();

    assert_eq!((id_len, name, samples), (4, "ab", 2));
    assert_eq!(allocs, 0, "FrameView happy path allocated {allocs} time(s)");
}

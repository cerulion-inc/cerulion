// SPDX-License-Identifier: AGPL-3.0-only
//! Follow-up RUNTIME coverage for the cdylib tier-2
//! `ExternalSource::Blocking` → doorbell-pipe collapse (FFI kind 2), which was
//! compiled but never runtime-exercised.
//!
//! A closure cannot cross the C ABI, so an `#[cerulion_node(external)]` cdylib
//! whose `external_source()` returns `ExternalSource::Blocking(..)` has its
//! macro-emitted `cerulion_node_external_source` export materialize the closure
//! into a `pipe(2)` + detached helper thread (via
//! `cerulion_core::graph::node::spawn_cdylib_blocking_doorbell`, running INSIDE
//! the cdylib's statically-linked cerulion_core) and return the READ end as
//! kind 2 (`EXTERNAL_SOURCE_KIND_DOORBELL_FD`). The host
//! (`DylibNodeEntry::external_source`) binds it as an OWNED, drained
//! `DoorbellFdSource`: attached to the live WaitSet AND drained to EAGAIN each
//! sweep (there is no node `tick()` reading it — the bytes are pure doorbell
//! signal). This file exercises the whole chain over a REAL cdylib
//! (`test_node_macro_blocking_cdylib`) + real iceoryx2.
//!
//! Coverage:
//! - (a) load + classification: the optional symbol resolves; `external_source()`
//!   yields kind 2 (`Fd` + the `#[doc(hidden)]` drained-doorbell marker `true`);
//!   the fd is valid + nonblocking-drainable (a real ringing pipe).
//! - (b) live fire + downstream delivery: the Blocking cdylib node fires when the
//!   helper rings and a data-trigger consumer DELIVERS its published value
//!   (delivery oracle, not fire-count presence — a fire records even on tick Err).
//! - (c) panic containment + no-busy-wake: under `CER_FAIL_MODE=blocking_panic`
//!   the helper panics → caught → poisoned → closes its write end; the host sees
//!   EOF, UNBINDS the doorbell LOUDLY (the busy-loop fix), the node never fires,
//!   and the process survives.
//! - (d) drop lifecycle: after teardown a second build/run in the SAME process
//!   re-loads + delivers cleanly (no leaked-fd / no-hang).
//! - (e) `CER_EXT_MODE` tier classification through the real FFI:
//!   `device_fd` → a poll-only tier-1 `Fd` with the doorbell marker
//!   FALSE (draining it each sweep would steal the node's data); `error_panic` →
//!   the collect-thread panic is contained by the FFI `catch_unwind`, reported as
//!   `None`, and logs one loud host error.
//! - (f) library-leak breadcrumb on drop: a dropped doorbell
//!   cdylib LEAKS its `Library` handle (a detached helper thread makes `dlclose`
//!   a use-after-unmap) and logs the breadcrumb.
//! - (g) NODES poison containment: a panicking `external_source()`
//!   does NOT poison the cdylib's `NODES` — a SECOND node from the same cdylib
//!   still inits + classifies functionally (order-robust, hand oracle).
//!
//! No fake data (Principle #13): every wake is a real helper ring (a real pipe
//! byte) and every oracle is a hand value, never a self-compare. `#[serial]` —
//! real iceoryx2 over the process-global SHM singleton (per-test SHM root via
//! `build_for_test`) + the cdylib `NODES` singleton.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_blocking_cdylib
//! cargo test -p cerulion_core --test cdylib_blocking_doorbell_test -- --test-threads=1
//! ```

use std::os::unix::io::RawFd;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level, logged_at};
// `ExternalSource`, `NodeContext`, `NodeEntry`, `AnyPublisher`, `MaxSliceLen`,
// `GraphRuntime` all come via the prelude glob; `DylibNodeEntry` + the `testing`
// harness are not in the prelude, so import them explicitly.
use cerulion_core::graph::node::DylibNodeEntry;
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Sentinel for "the consumer never recorded a value" (distinct from any real
/// published payload, which is `>= 1`).
const MISSING: u64 = u64::MAX;

/// Monotonic prefix counter so re-builds within one process never collide on an
/// iceoryx2 service name (mirrors non_trigger_hold / external_live_fire).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    let n = PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("bdb{stem}{n}")
}

/// RAII env-var guard (panic-safe cleanup — `chunk_c_ffi_codes_3_4_test.rs`
/// precedent). `#[serial]` serializes the bodies but does NOT reset env between
/// them, so a panic between `set_var` and a manual `remove_var` would leak the
/// mode to the next test.
struct EnvVarGuard {
    name: &'static str,
}

impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

/// Locate the Blocking-source test cdylib. Built by
/// `cargo build -p test_node_macro_blocking_cdylib`.
fn find_blocking_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_blocking_cdylib")
}

/// Set `O_NONBLOCK` on `fd` (the host does this at collect time in production;
/// the standalone classification test does it by hand).
fn set_read_nonblocking(fd: RawFd) {
    // SAFETY: `fd` is the live doorbell read end just returned by the cdylib.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    assert!(flags >= 0, "F_GETFL on the doorbell fd must succeed");
    // SAFETY: same valid fd; set O_NONBLOCK on the read end.
    let rc = unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
    assert_eq!(rc, 0, "F_SETFL O_NONBLOCK on the doorbell fd must succeed");
}

/// One non-blocking drain attempt on `fd`: `true` if a ring byte was read.
fn drained_a_ring(fd: RawFd) -> bool {
    let mut buf = [0u8; 64];
    // SAFETY: `fd` is our O_NONBLOCK doorbell read end; read into a live buffer.
    let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast::<libc::c_void>(), buf.len()) };
    n > 0
}

/// Drive the live seam until `pred` holds or `deadline` elapses; returns whether
/// `pred` held (mirrors `external_live_fire_iox2_test::step_until`).
fn step_until(rt: &mut GraphRuntime, deadline: Duration, mut pred: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if pred() {
            return true;
        }
        rt.run_live_step_once_for_test(Duration::from_millis(50));
    }
    pred()
}

/// Poll `pred` until true or `deadline` elapses (for a helper thread's async
/// effect, e.g. a ring appearing on the pipe).
fn wait_until(deadline: Duration, mut pred: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    pred()
}

// ---------------------------------------------------------------------------
// Downstream consumer (in-process delivery oracle)
// ---------------------------------------------------------------------------

/// Data-trigger consumer recording the last received `x` (delivery oracle) and a
/// fire count. Fires ONLY when the Blocking cdylib producer actually publishes.
#[cerulion_node]
#[derive(Default)]
struct RecvConsumer {
    #[input(trigger)]
    inp: Vector3,
    last: Arc<AtomicU64>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl RecvConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        self.last.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// Build a graph: the REAL Blocking cdylib producer (`producer`, external) →
/// `producer/out` → in-process data-trigger `consumer` (delivery oracle).
/// Returns the runtime + the consumer's `last`/`fires` shared records.
fn build_blocking_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, Arc<AtomicU64>) {
    let last = Arc::new(AtomicU64::new(MISSING));
    let fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_blocking_doorbell".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "blocking_ext".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "recv_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib")),
    );
    factories.insert(
        "consumer".to_string(),
        Box::new(RecvConsumerEntry::with_state(RecvConsumer {
            last: Arc::clone(&last),
            fires: Arc::clone(&fires),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    let rt = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build blocking-doorbell graph");
    (rt, last, fires)
}

// ---------------------------------------------------------------------------
// (a) Load + classification
// ---------------------------------------------------------------------------

/// The Blocking cdylib collapses to a kind-2 doorbell: `external_source()`
/// yields `Fd(fd)` with the `#[doc(hidden)]` drained-doorbell marker `true`, and
/// the fd is a real, nonblocking-drainable ringing pipe. `external_source()`
/// needs a live handle, so the node is `init`'d first with a context carrying
/// its `out` output.
#[test]
#[serial]
fn blocking_cdylib_classifies_as_drained_doorbell_fd() {
    let mut node = DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib");

    // Minimal context carrying the fixture's single `out` output.
    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(
        "test/blocking_doorbell/classify_out_local",
        MaxSliceLen::const_new(256),
        0,
    );
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());
    node.init(ctx).expect("init blocking cdylib");

    // Kind 2: the closure was collapsed to a doorbell pipe; the host gets the
    // READ end as an `Fd` AND the drained-doorbell marker latches `true`.
    let src = node.external_source();
    let fd = match src {
        Some(ExternalSource::Fd(fd)) => fd,
        other => panic!("blocking cdylib must classify as ExternalSource::Fd, got {other:?}"),
    };
    assert!(fd >= 0, "doorbell read fd must be valid, got {fd}");
    assert!(
        node.external_source_is_drained_doorbell_fd(),
        "a tier-2 Blocking collapse must be flagged as a DRAINED doorbell fd (kind 2), \
         distinct from a poll-only device fd"
    );
    // …and the collapse LATCHES the library leak: the detached helper thread
    // makes `dlclose` a use-after-unmap, so the entry must keep the handle. The
    // level-free complement of the `debug!` breadcrumb test (f) pins — this
    // one holds in a release build too.
    assert!(
        node.leaks_library_on_drop(),
        "a doorbell collapse must latch leak_library_on_drop"
    );

    // The fd is a real, nonblocking-drainable, ringing pipe: set O_NONBLOCK and
    // prove a helper ring arrives (the helper sleeps ~1ms then writes a byte).
    set_read_nonblocking(fd);
    assert!(
        wait_until(Duration::from_secs(5), || drained_a_ring(fd)),
        "the doorbell helper must ring the pipe (a drainable byte) within the deadline"
    );

    // Stop the helper: close the read end → its next write(2) fails → it exits.
    // (The DylibNodeEntry Drop keeps the cdylib mapped while the detached helper
    // is winding down — see the runtime-drop lifecycle fix.)
    // SAFETY: `fd` is the doorbell read end; we own it here and close it once.
    unsafe { libc::close(fd) };
}

// ---------------------------------------------------------------------------
// (b) Live fire + downstream delivery
// ---------------------------------------------------------------------------

/// The Blocking cdylib node fires off its doorbell over the live seam and a
/// data-trigger consumer DELIVERS the published value. Level-triggered
/// continuous source ⇒ N rings coalesce to ≥1 fire; the delivery oracle is
/// "consumer saw a real payload (`>= 1`)", not a fire count.
#[test]
#[serial]
fn blocking_cdylib_fires_and_delivers_downstream() {
    let prefix = unique_prefix("fire");
    let (mut rt, last, fires) = build_blocking_graph(&prefix);

    // Production order: collect once (spawns the doorbell binding), then loop.
    rt.collect_external_sources_for_test();
    assert_eq!(
        rt.external_binding_count(),
        1,
        "the Blocking cdylib must produce exactly one (doorbell) external binding"
    );

    // Drive the live seam until the consumer delivers a real payload.
    let delivered = step_until(&mut rt, Duration::from_secs(10), || {
        last.load(Ordering::Relaxed) != MISSING
    });
    assert!(
        delivered,
        "the Blocking cdylib doorbell must fire the node and deliver downstream within the deadline"
    );

    // Delivery oracle: the consumer saw the producer's incrementing payload
    // (>= 1.0), and it fired at least once.
    let seen = last.load(Ordering::Relaxed);
    assert!(
        seen >= 1 && seen != MISSING,
        "consumer must record the cdylib's published payload (>= 1), got {seen}"
    );
    assert!(
        fires.load(Ordering::Relaxed) >= 1,
        "consumer must have fired at least once on the delivered data"
    );
}

// ---------------------------------------------------------------------------
// (c) Panic containment + no-busy-wake (unbind) contract
// ---------------------------------------------------------------------------

/// Under `CER_FAIL_MODE=blocking_panic` the helper panics on its first call →
/// caught on the helper thread → source poisoned → helper closes its write end.
/// The host's doorbell drain sees EOF and UNBINDS the binding LOUDLY (the
/// busy-loop fix: an EOF'd pipe read end is permanently readable, so leaving it
/// bound would wake the WaitSet forever). The node NEVER fires, the binding
/// count drops to 0 (no-busy-wake), and the process survives.
#[test]
#[serial]
#[traced_test]
fn blocking_cdylib_panic_is_contained_and_unbinds() {
    let _env = EnvVarGuard::set("CER_FAIL_MODE", "blocking_panic");

    let prefix = unique_prefix("panic");
    let (mut rt, last, fires) = build_blocking_graph(&prefix);

    // Collect spawns the helper (which panics on its first call); the binding
    // forms with the doorbell read end.
    rt.collect_external_sources_for_test();
    assert_eq!(
        rt.external_binding_count(),
        1,
        "the doorbell binding must form even when the helper will panic"
    );

    // Drive the live seam: the poisoned helper closes its write end, the host
    // sees EOF and unbinds. Sequential bounded loop (the binding-count read and
    // the live step both borrow `rt`, so they cannot share a `step_until`
    // closure) — stop as soon as the binding unbinds.
    let mut unbound = false;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(10) {
        if rt.external_binding_count() == 0 {
            unbound = true;
            break;
        }
        rt.run_live_step_once_for_test(Duration::from_millis(50));
    }
    assert!(
        unbound,
        "a poisoned (panicked) doorbell helper must lead the host to UNBIND the doorbell \
         (no permanent busy-wake on the EOF'd read end)"
    );

    // The node NEVER fired: no payload reached the consumer.
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "a panicking Blocking cdylib must never fire the node"
    );
    assert_eq!(
        last.load(Ordering::Relaxed),
        MISSING,
        "a panicking Blocking cdylib must deliver nothing downstream"
    );

    // The loud, host-emitted unbind error was logged (the cdylib's own poison
    // log is emitted by the cdylib's tracing dispatcher and is NOT captured
    // here; the host `runtime.rs` unbind error IS).
    // "Loud" is a LEVEL claim, and the level is read from the line HEADER by the
    // ONE shared helper: an unbind demoted to `warn!`/`info!` cannot pass as the
    // loud error, and neither can a WARN/INFO line whose own message or field
    // VALUE happens to carry the token `ERROR`.
    logs_assert(|lines: &[&str]| logged_at(lines, "ERROR", "UNBINDING the doorbell"));

    // Further live steps keep the node inert (fire count frozen) — the
    // no-busy-wake contract holds after unbind.
    let fires_before = fires.load(Ordering::Relaxed);
    for _ in 0..20 {
        rt.run_live_step_once_for_test(Duration::from_millis(10));
    }
    assert_eq!(
        fires.load(Ordering::Relaxed),
        fires_before,
        "after the doorbell unbinds, the node must stay inert across further live steps"
    );
    // Reaching here (no SIGSEGV / no hang) proves the panic was contained.
}

// ---------------------------------------------------------------------------
// (d) Drop lifecycle — a second build/run in the same process re-loads cleanly
// ---------------------------------------------------------------------------

/// After a first Blocking-cdylib graph delivers and is dropped (teardown closes
/// the doorbell read end → the helper self-terminates; the cdylib stays mapped
/// while the detached helper winds down), a SECOND build+run in the SAME process
/// re-loads the fixture and delivers cleanly — no leaked fd, no hang.
#[test]
#[serial]
fn blocking_cdylib_reloads_cleanly_after_drop() {
    // First session: build, fire, deliver, then drop.
    {
        let prefix = unique_prefix("reloadA");
        let (mut rt, last, _fires) = build_blocking_graph(&prefix);
        rt.collect_external_sources_for_test();
        assert!(
            step_until(&mut rt, Duration::from_secs(10), || last
                .load(Ordering::Relaxed)
                != MISSING),
            "first session must deliver before drop"
        );
        // `rt` drops here (teardown).
    }

    // Second session: a fresh build+run in the same process must work.
    {
        let prefix = unique_prefix("reloadB");
        let (mut rt, last, fires) = build_blocking_graph(&prefix);
        rt.collect_external_sources_for_test();
        assert!(
            step_until(&mut rt, Duration::from_secs(10), || last
                .load(Ordering::Relaxed)
                != MISSING),
            "second session must re-load the cdylib and deliver (no leaked fd / no hang)"
        );
        assert!(
            fires.load(Ordering::Relaxed) >= 1,
            "second-session consumer must fire on delivered data"
        );
    }
    // Reaching here without a hang IS the no-leak assertion.
}

// ---------------------------------------------------------------------------
// (e) CER_EXT_MODE tier classification through the real FFI
// ---------------------------------------------------------------------------

/// Build a minimal `NodeContext` carrying the fixture's single `out` output, so
/// the just-loaded cdylib can be `init`'d before `external_source()` is queried
/// (the FFI export needs a live handle). `local_topic` keeps each test's local
/// publisher on its own iceoryx2 service name.
fn init_with_out_ctx(node: &mut DylibNodeEntry, local_topic: &str) {
    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(local_topic, MaxSliceLen::const_new(256), 0);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::for_tests(publishers, IndexMap::new());
    node.init(ctx).expect("init blocking cdylib");
}

/// `CER_EXT_MODE=device_fd` reports a poll-only tier-1 device fd through the
/// real FFI — `external_source()` yields `Some(Fd(fd))` with a VALID fd AND the
/// drained-doorbell marker `false`. Mutation this pins: latching the doorbell
/// marker for kind 1 would make the host destructively DRAIN a real device fd
/// each sweep (stealing the node's data) — this asserts the marker stays false.
#[test]
#[serial]
fn device_fd_mode_classifies_as_poll_only_device_fd() {
    let _env = EnvVarGuard::set("CER_EXT_MODE", "device_fd");
    let mut node = DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib");
    init_with_out_ctx(&mut node, "test/blocking_doorbell/device_fd_out_local");

    let src = node.external_source();
    let fd = match src {
        Some(ExternalSource::Fd(fd)) => fd,
        other => panic!("device_fd mode must classify as ExternalSource::Fd, got {other:?}"),
    };
    assert!(fd >= 0, "device fd must be valid, got {fd}");
    assert!(
        !node.external_source_is_drained_doorbell_fd(),
        "a poll-only device fd (kind 1) must NOT be flagged as a DRAINED doorbell — \
         draining it each sweep would steal the node's data"
    );

    // The read end is a real, valid pipe (quiescent — not ringing here). Close it
    // (we own it in this standalone classification test).
    // SAFETY: `fd` is the device read end just returned by the cdylib.
    unsafe { libc::close(fd) };
}

/// `CER_EXT_MODE=error_panic` — `external_source()` panics on the collect
/// thread INSIDE the cdylib → the macro-emitted `cerulion_node_external_source`
/// export's `catch_unwind` returns `EXTERNAL_SOURCE_KIND_ERROR` (-1) → the host
/// reports `None`, leaves the doorbell marker false, and logs ONE loud host
/// error. The process survives (the panic was contained by the FFI catch_unwind).
///
/// This test only proves CONTAINMENT (no abort) + classification; it says nothing
/// about `NODES` poison, and it happens to sort LAST alphabetically so a poisoned
/// `NODES` here would mask no later test — pure luck. The no-poison contract
/// is pinned independently and order-robustly by
/// [`external_source_panic_does_not_poison_nodes`] below (it re-uses the SAME
/// cdylib within one test, so ordering is irrelevant).
#[test]
#[serial]
#[traced_test]
fn error_panic_mode_reports_none_and_logs_host_error() {
    let _env = EnvVarGuard::set("CER_EXT_MODE", "error_panic");
    let mut node = DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib");
    init_with_out_ctx(&mut node, "test/blocking_doorbell/error_panic_out_local");

    let src = node.external_source();
    assert!(
        src.is_none(),
        "a cdylib whose external_source() panics must be reported as None (kind -1), got {src:?}"
    );
    assert!(
        !node.external_source_is_drained_doorbell_fd(),
        "the kind -1 error arm must leave the drained-doorbell marker false"
    );
    // "Loud" is a LEVEL claim, read from the line HEADER (see the sibling arm).
    logs_assert(|lines: &[&str]| logged_at(lines, "ERROR", "node has no external source"));
    // Reaching here (no abort) proves the FFI catch_unwind contained the panic.
}

/// NODES poison containment: a panicking `external_source()`
/// must NOT poison the cdylib's process-global `NODES` registry. The generated
/// `cerulion_node_external_source` export drives the user's `external_source()`
/// under an INNER `catch_unwind` BEFORE the `NODES` MutexGuard drops, so the panic
/// unwinds only the closure, never the live guard.
///
/// Proof (order-robust, single test): after node A's `error_panic` query, a SECOND
/// node B from the SAME cdylib (default doorbell mode) still `init`s AND classifies
/// as a functional drained doorbell (`Some(Fd)` + marker `true`). WITHOUT the inner catch the
/// panic would poison `NODES`, so B's FFI calls would short-circuit to the poison
/// error — B's `init` would fail (`.expect` panic) or its `external_source()` would
/// report `None`. Node A is kept alive across B's query (error_panic does NOT leak
/// the library) so the shared `NODES` stays mapped. Hand oracle, never a
/// self-compare.
#[test]
#[serial]
fn external_source_panic_does_not_poison_nodes() {
    // Node A: query under error_panic (panics INSIDE external_source()).
    let mut node_a = DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib A");
    {
        let _env = EnvVarGuard::set("CER_EXT_MODE", "error_panic");
        init_with_out_ctx(&mut node_a, "test/blocking_doorbell/nopoison_a_out_local");
        let src_a = node_a.external_source();
        assert!(
            src_a.is_none(),
            "the panicking external_source() must be reported as None, got {src_a:?}"
        );
        // `_env` drops HERE → CER_EXT_MODE unset before node B is queried.
    }

    // Node B: same cdylib, DEFAULT (doorbell) mode. If A's panic had poisoned
    // NODES, B's init / external_source() FFI calls would short-circuit to the
    // poison error and B would classify as None here.
    let mut node_b = DylibNodeEntry::load(&find_blocking_cdylib()).expect("load blocking cdylib B");
    init_with_out_ctx(&mut node_b, "test/blocking_doorbell/nopoison_b_out_local");
    let src_b = node_b.external_source();
    let fd = match src_b {
        Some(ExternalSource::Fd(fd)) => fd,
        other => panic!(
            "after a panicking external_source(), a fresh node from the SAME cdylib must still \
             classify functionally (NODES not poisoned), got {other:?}"
        ),
    };
    assert!(fd >= 0, "doorbell read fd must be valid, got {fd}");
    assert!(
        node_b.external_source_is_drained_doorbell_fd(),
        "the second node must classify as a functional drained doorbell — a poisoned NODES \
         would have forced the FFI to report None (no marker)"
    );

    // Stop node B's helper: close the read end so its next write(2) fails and it
    // exits. SAFETY: `fd` is node B's doorbell read end; close it once here.
    unsafe { libc::close(fd) };

    // Keep node A alive until here so the cdylib (and its shared NODES) stays
    // mapped for node B's query (error_panic mode does NOT leak the library).
    drop(node_a);
}

// ---------------------------------------------------------------------------
// (f) Library-leak breadcrumb on drop
// ---------------------------------------------------------------------------

/// A dropped doorbell cdylib LEAKS its `Library` handle (it spawned a
/// detached helper thread, so `dlclose` would be a use-after-unmap) and logs the
/// breadcrumb. The default (Blocking → doorbell) mode latches
/// `leak_library_on_drop` inside `external_source()`, so `collect` then drop must
/// emit the `debug!` breadcrumb (captured via `traced_test` + the `no-env-filter`
/// feature). Pins the leak-on-doorbell contract's OBSERVABILITY where `debug!`
/// exists; in a release build the breadcrumb cannot, and the contract itself is
/// pinned level-free by `blocking_cdylib_classifies_as_drained_doorbell_fd`'s
/// `leaks_library_on_drop()` assert.
#[test]
#[serial]
#[traced_test]
fn blocking_cdylib_drop_logs_library_leak_breadcrumb() {
    {
        let prefix = unique_prefix("leakbc");
        let (mut rt, _last, _fires) = build_blocking_graph(&prefix);
        // collect queries external_source() → kind DOORBELL_FD → latches the
        // producer's leak_library_on_drop.
        rt.collect_external_sources_for_test();
        assert_eq!(
            rt.external_binding_count(),
            1,
            "the doorbell binding must form so leak_library_on_drop is latched"
        );
        // `rt` drops here → the producer's DylibNodeEntry::drop leaks + logs.
    }
    // The breadcrumb is `debug!`: it exists only where `debug!` is compiled in.
    // Level-free twin: the library-leak breadcrumb must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        for level in ["WARN", "INFO", "ERROR"] {
            if lines.iter().any(|l| {
                line_level(l) == Some(level) && (l.contains("leaking cdylib library handle"))
            }) {
                return Err(format!(
                    "the library-leak breadcrumb was emitted at {level}"
                ));
            }
        }
        Ok(())
    });
    // Checked AT DEBUG, not by text: the breadcrumb re-emitted at `trace!`
    // still satisfies a `logs_contain`, and the loud sweep above permits TRACE.
    logs_assert(|lines: &[&str]| {
        let breadcrumbs = count_at_exclusively(lines, "DEBUG", &["leaking cdylib library handle"])?;
        if breadcrumbs == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "a dropped doorbell cdylib must log the library-leak breadcrumb at DEBUG \
                 EXACTLY once, got {breadcrumbs}"
            ))
        }
    });
}

// SPDX-License-Identifier: AGPL-3.0-only
//! Mixed Listener + raw-fd WaitSet wake sources over real
//! iceoryx2.
//!
//! Chunk 1 is the reactor FOUNDATION for the driver-ingress external trigger (an
//! `#[cerulion_node(external)]` node that self-triggers off a device fd). It
//! teaches `WaitSetReactor::run_once` to accept a heterogeneous `WaitSource`
//! slice — iceoryx2 `Listener`s (as before) PLUS non-owning raw device fds
//! (`FdSource`). NO scheduler marking, NO macro wiring yet, so
//! listener-only graphs stay byte-identical.
//!
//! These tests drive the two cfg-gated `GraphRuntime` seams that bridge to the
//! `pub(crate)` reactor:
//!   * `run_waitset_reactor_once_with_fds_for_test` — runs one reactor cycle over
//!     the graph's listener sources PLUS extra raw fds, reporting the fired-set,
//!     per-fd construction result, and whether it blocked.
//!   * `run_waitset_reactor_stale_fd_for_test` — constructs a VALID fd source,
//!     then closes the fd so it goes stale, then runs a cycle: the attach-time
//!     `is_valid` probe must SKIP it (HAZARD 1 — in this scenario the staleness
//!     pre-dates the probe, so the guard deterministically catches it before
//!     iceoryx2's `select` EBADF fatal abort; in general the probe is a
//!     best-effort TOCTOU narrowing).
//!
//! A `libc::pipe` supplies a real, portable (macOS + Linux) fd: the read end is
//! the wake source, readable once a byte is written to the write end. Every test
//! is `#[serial]` (the seams build an iceoryx2 WaitSet over the process-global
//! shared-memory singleton). No fake data (Principle #13): every wake is a real
//! pipe byte or a real published message.

use std::os::unix::io::RawFd;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::CerulionPublisher;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The consumer's absolute external trigger topic — no in-graph producer, so the
/// graph provisions it `External` and an out-of-graph publisher may attach and
/// publish onto the exact topic the consumer subscribes.
const EXT_TOPIC: &str = "/wsfd/ext/cam";

/// Data-trigger consumer: fires whenever its `inp` input receives data. Its
/// iceoryx2 event `Listener` is the graph's (only) listener wake source.
#[cerulion_node]
#[derive(Default)]
struct FdWsConsumer {
    #[input(trigger)]
    inp: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl FdWsConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }
}

/// A `libc::pipe` with RAII cleanup. The read end is a valid fd usable as a
/// WaitSet wake source (readable once a byte is written to the write end).
struct Pipe {
    read: RawFd,
    write: RawFd,
}

impl Pipe {
    fn new() -> Pipe {
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: `fds` is a 2-element array; `pipe` writes exactly two fds.
        let rc = unsafe { libc::pipe(fds.as_mut_ptr()) };
        assert_eq!(rc, 0, "libc::pipe must succeed");
        Pipe {
            read: fds[0],
            write: fds[1],
        }
    }

    /// Write one byte to the write end, making the read end readable.
    fn write_byte(&self) {
        let b: u8 = 1;
        // SAFETY: `self.write` is a valid open fd; we write exactly one byte from
        // a live one-byte buffer.
        let n = unsafe { libc::write(self.write, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
        assert_eq!(n, 1, "pipe write must succeed");
    }

    /// Read one byte from the read end. Returns the `read(2)` result (`1` on
    /// success, `-1` on a closed/invalid fd) — the survives-detach proof.
    fn read_byte(&self) -> isize {
        let mut b: u8 = 0;
        // SAFETY: `self.read` is (in the survives-detach test) still a valid open
        // fd; we read at most one byte into a live one-byte buffer.
        unsafe { libc::read(self.read, std::ptr::addr_of_mut!(b) as *mut libc::c_void, 1) }
    }

    /// Relinquish ownership of the READ fd to the caller (who becomes responsible
    /// for closing it). After this, `Drop` will NOT close the read end — used
    /// when a seam takes ownership of the fd and closes it itself (the stale-fd
    /// seam), so `Drop` cannot double-close a since-reused fd number.
    fn take_read(&mut self) -> RawFd {
        let r = self.read;
        self.read = -1;
        r
    }
}

impl Drop for Pipe {
    fn drop(&mut self) {
        // Close each end that this Pipe still owns (>= 0). SAFETY: each is a fd
        // this Pipe opened and has not relinquished; closed at most once.
        if self.read >= 0 {
            unsafe { libc::close(self.read) };
        }
        if self.write >= 0 {
            unsafe { libc::close(self.write) };
        }
    }
}

/// Build a single data-trigger consumer subscribing the absolute external topic
/// `EXT_TOPIC` — one listener wake source, ready to co-exist with fd sources.
fn fd_ws_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_fd_source_test".to_string(),
        prefix: "wsfd".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "fd_ws_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("consumer".to_string(), Box::new(FdWsConsumerEntry::new()));
    (config, factories)
}

/// Build the graph over an isolated per-test SHM root.
fn build() -> GraphRuntime {
    let (config, factories) = fd_ws_graph();
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build fd-source graph")
}

/// Attach an out-of-graph publisher on the consumer's absolute external trigger
/// topic (provisioned `External`, so no single-writer cap blocks this attach).
fn external_publisher(runtime: &GraphRuntime, topic: &str) -> CerulionPublisher {
    let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
    mgr.create_publisher(topic, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to the absolute external topic")
}

/// Drain the build/attach connection-lifecycle events queued on the consumer's
/// trigger listener, so a later assertion is attributable to a specific wake and
/// not to connection noise.
fn prime_drain_connection_noise(runtime: &mut GraphRuntime) {
    let _ = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
}

/// Publish exactly ONE frame onto `pubr`'s topic (the proxy publishes on drop),
/// leaving a fresh `SentSample` notification on the consumer's trigger listener.
fn publish_one(pubr: &mut CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

// ---------------------------------------------------------------------------
// 1. A readable device fd wakes the reactor.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_source_fires_on_readable_pipe() {
    let mut runtime = build();
    // Drain the listener's build noise so the ONLY thing that can fire this cycle
    // is the pipe fd (the listener gets no fresh publish).
    prime_drain_connection_noise(&mut runtime);

    let pipe = Pipe::new();
    // Make the read end readable.
    pipe.write_byte();

    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe.read, "fd_node")],
        Duration::from_millis(200),
    );

    assert_eq!(
        outcome.constructed_ok,
        vec![true],
        "a valid pipe read fd must construct an FdSource"
    );
    assert!(
        outcome.blocked,
        "with >=1 attachable source the reactor must actually block on the WaitSet"
    );
    // Hand oracle: the listener was primed (quiet), so the ONLY fired source is
    // the readable pipe fd. Exact vector kills a spurious extra id / reorder.
    assert_eq!(
        outcome.fired,
        vec!["fd_node".to_string()],
        "the readable pipe fd must be the sole fired source; fired = {:?}",
        outcome.fired
    );
}

// ---------------------------------------------------------------------------
// 2. Listener + fd fire in the SAME cycle, reported in DECLARATION order.
//
// The seam lists listener sources FIRST, then the extra fds — so a listener that
// fires and a written pipe both appear, listener before fd. Pins the mixed-kind
// ordering contract (source index, not source kind or name).
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_and_listener_same_cycle_declaration_order() {
    let mut runtime = build();
    let mut pubr = external_publisher(&runtime, EXT_TOPIC);
    // Drain build + publisher-attach connection noise before the data publish.
    prime_drain_connection_noise(&mut runtime);

    // Both the listener (a real published frame) and the fd (a written pipe) are
    // pending for THIS cycle.
    publish_one(&mut pubr, 1.0);
    let pipe = Pipe::new();
    pipe.write_byte();

    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe.read, "fd_node")],
        Duration::from_millis(200),
    );

    assert_eq!(
        outcome.fired,
        vec!["consumer".to_string(), "fd_node".to_string()],
        "listener + fd fired the same cycle → fired-set is EXACTLY [consumer, fd_node] \
         in declaration order (listener sources precede fd sources); fired = {:?}",
        outcome.fired
    );
}

// ---------------------------------------------------------------------------
// 3. An unwritten fd + no publish → the reactor BLOCKS and returns empty.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_source_timeout_returns_empty_and_blocked() {
    let mut runtime = build();
    prime_drain_connection_noise(&mut runtime);

    // Unwritten pipe: the read end is NOT readable, so the fd never fires; the
    // listener was primed quiet. The reactor must block the full (short) timeout
    // and return an empty fired-set.
    let pipe = Pipe::new();

    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe.read, "fd_node")],
        Duration::from_millis(50),
    );

    assert_eq!(
        outcome.constructed_ok,
        vec![true],
        "the unwritten (but valid) pipe fd still constructs"
    );
    assert!(
        outcome.fired.is_empty(),
        "no source became readable within the timeout → empty fired-set; fired = {:?}",
        outcome.fired
    );
    assert!(
        outcome.blocked,
        "with >=1 attachable source the reactor must actually BLOCK on the WaitSet \
         (not short-circuit) even when nothing fires"
    );
}

// ---------------------------------------------------------------------------
// 4. The wrapped fd is NON-OWNING: a reactor cycle does not close it.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_source_non_owning_survives_detach() {
    let mut runtime = build();
    prime_drain_connection_noise(&mut runtime);

    let pipe = Pipe::new();
    pipe.write_byte();

    // Run one full cycle: the seam builds an FdSource over `pipe.read`, attaches
    // it, then drops the guard AND the FdSource when it returns. A non-owning
    // FdSource must NOT close `pipe.read` on drop.
    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe.read, "fd_node")],
        Duration::from_millis(200),
    );
    assert_eq!(
        outcome.fired,
        vec!["fd_node".to_string()],
        "sanity: the readable fd fired this cycle"
    );

    // The byte written above (never consumed by the reactor — record-only) must
    // still be readable through the pipe: proves `pipe.read` is still open (the
    // FdSource's non-owning Drop did NOT close it). A closed fd would return -1.
    let n = pipe.read_byte();
    assert_eq!(
        n, 1,
        "the pipe's read end must survive the reactor cycle (non-owning fd): \
         read(2) returned {n}, expected 1"
    );
}

// ---------------------------------------------------------------------------
// 5. Invalid fds are rejected at CONSTRUCTION — they never reach attach — while
// the FD_SETSIZE-1 boundary fd is ACCEPTED (the reject is `>=`, not `>`-1).
//
// `-1` (raw < 0), a value >= FD_SETSIZE (HAZARD 2 — out-of-bounds in iceoryx2's
// unchecked FD_SET), and a closed fd (FileDescriptor::non_owning_new probes
// F_GETFD) all fail FdSource::non_owning, so `constructed_ok` is false for each
// and none appear in the fired-set. A LIVE fd dup2'd to exactly FD_SETSIZE-1
// (1023, the last in-bounds value) constructs fine — pins the boundary as
// inclusive-reject at 1024, not an off-by-one at 1023. And `blocked == true`
// distinguishes "the invalid fds were rejected" from "the reactor never built /
// never attached anything": the graph's consumer listener (and the boundary fd)
// still attach, so a healthy reactor DID block on the WaitSet.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn invalid_fd_rejected_at_construction() {
    let mut runtime = build();
    prime_drain_connection_noise(&mut runtime);

    // Open ALL fixture fds BEFORE the close below: `Pipe::new`/`dup2` allocate
    // the lowest free fd numbers, so opening anything AFTER closing `closed_fd`
    // would REUSE its just-freed number and resurrect "closed" as a live fd
    // (observed: the boundary pipe landed on the closed number).
    let mut closed_pipe = Pipe::new();

    // ACCEPT arm: a LIVE fd at exactly FD_SETSIZE - 1 (1023) — the last in-bounds
    // value — must construct. dup2 a real pipe read end onto that number (1023 is
    // far above the test process's organically-open fds, so nothing is displaced).
    let boundary_pipe = Pipe::new();
    let boundary_fd: RawFd = (libc::FD_SETSIZE - 1) as RawFd;
    // LOUD precondition: fd 1023 must be FREE — dup2 silently close(2)s whatever
    // lives at its target, so a live fd there (e.g. an iceoryx2 internal fd)
    // would be clobbered and poison later #[serial] tests in this binary.
    // SAFETY: F_GETFD on an arbitrary fd number is a read-only liveness probe.
    assert_eq!(
        unsafe { libc::fcntl(boundary_fd, libc::F_GETFD) },
        -1,
        "FD_SETSIZE-1 must be free before dup2 else it silently clobbers a live fd"
    );
    // SAFETY: `boundary_pipe.read` is a valid open fd; `boundary_fd` (1023) is an
    // in-range fd number for dup2, asserted free above. The dup is closed
    // explicitly below.
    let rc = unsafe { libc::dup2(boundary_pipe.read, boundary_fd) };
    assert_eq!(rc, boundary_fd, "dup2 to FD_SETSIZE-1 must succeed");

    // A genuinely-closed fd: take `closed_pipe`'s read end and close it. The
    // number is now a closed (invalid) fd. Nothing opens an fd between here and
    // the seam call (the reactor is built inside the seam AFTER construction
    // probing), so it stays closed when `non_owning` probes it.
    let closed_fd = closed_pipe.take_read();
    // SAFETY: sole close of the fd we just relinquished from `closed_pipe`.
    unsafe { libc::close(closed_fd) };

    // Exactly at the FD_SETSIZE boundary: `non_owning` rejects `>= FD_SETSIZE`
    // regardless of whether the number happens to be open, so this pins the guard
    // (not fd-openness). `RawFd` is `i32`; `FD_SETSIZE` (1024) fits.
    let over_fd_setsize: RawFd = libc::FD_SETSIZE as RawFd;

    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[
            (-1, "neg"),
            (closed_fd, "closed"),
            (over_fd_setsize, "huge"),
            (boundary_fd, "boundary_ok"),
        ],
        Duration::from_millis(50),
    );
    // SAFETY: sole close of the dup2-created boundary fd (the FdSource over it
    // was non-owning; the original `boundary_pipe.read` is closed by Pipe::drop).
    unsafe { libc::close(boundary_fd) };

    assert_eq!(
        outcome.constructed_ok,
        vec![false, false, false, true],
        "the three invalid fds (-1, closed, >= FD_SETSIZE) must fail FdSource \
         construction while the live FD_SETSIZE-1 boundary fd constructs; \
         constructed_ok = {:?}",
        outcome.constructed_ok
    );
    for id in ["neg", "closed", "huge"] {
        assert!(
            !outcome.fired.iter().any(|f| f == id),
            "a rejected fd ({id}) never reaches attach, so it can never fire; \
             fired = {:?}",
            outcome.fired
        );
    }
    // The boundary pipe was never written, so its fd attached but did not fire.
    assert!(
        !outcome.fired.iter().any(|f| f == "boundary_ok"),
        "the unwritten boundary fd attaches but must not fire; fired = {:?}",
        outcome.fired
    );
    // The reactor was healthy: the consumer's listener (and the boundary fd)
    // attached, so the cycle actually BLOCKED — the rejections above are the fds'
    // failures, not a reactor-never-built artifact.
    assert!(
        outcome.blocked,
        "the graph's consumer listener still attaches, so a healthy reactor must \
         have blocked on the WaitSet this cycle"
    );
}

// ---------------------------------------------------------------------------
// 6. THE EBADF-guard pin: a fd valid at construction but stale at attach is
// SKIPPED (HAZARD 1 — no iceoryx2 `select` EBADF fatal abort), and the graph's
// listener source is still observed the same cycle.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn stale_fd_skipped_at_attach_no_abort() {
    let mut runtime = build();
    let mut pubr = external_publisher(&runtime, EXT_TOPIC);
    prime_drain_connection_noise(&mut runtime);
    // A real published frame so the listener has something to fire on this cycle.
    publish_one(&mut pubr, 1.0);

    // A valid pipe read fd; the seam takes ownership (it closes the fd itself to
    // inject the staleness), so relinquish it from the Pipe to avoid a
    // double-close in Drop.
    let mut pipe = Pipe::new();
    let stale = pipe.take_read();

    // The seam: build an FdSource from `stale` (valid now) → close `stale` →
    // run one reactor cycle. If the attach-time `is_valid` probe did NOT skip the
    // stale fd, iceoryx2's `select` would `fatal_panic` on EBADF and abort the
    // whole process — reaching the assertions below AT ALL proves the guard held.
    let (fired, built_valid) = runtime.run_waitset_reactor_stale_fd_for_test(
        stale,
        "stale_fd",
        Duration::from_millis(200),
    );

    assert!(
        built_valid,
        "precondition: the fd was valid when the FdSource was constructed"
    );
    assert!(
        !fired.iter().any(|f| f == "stale_fd"),
        "the stale fd must be SKIPPED at attach (never fires); fired = {fired:?}"
    );
    assert!(
        fired.iter().any(|f| f == "consumer"),
        "the graph's listener source must still be observed the same cycle (the \
         stale-fd skip must not disturb the rest of the reactor cycle); \
         fired = {fired:?}"
    );
}

// ---------------------------------------------------------------------------
// 7. TWO fds in one cycle fire in DECLARATION (index) order, NOT name order.
//
// The node ids are chosen REVERSE-alphabetical vs their index order ("zzz_fd"
// at index 0, "aaa_fd" at index 1), so a sort-by-NAME regression in the reactor
// would surface as ["aaa_fd", "zzz_fd"] instead of the contract ["zzz_fd",
// "aaa_fd"]. (Test 2's ids happen to be alphabetical in index order, so there a
// name sort and an index sort agree — this is the disambiguating pin.) Also
// covers the multi-fd-same-cycle gap: every earlier test attaches at most ONE fd.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn two_fds_reverse_alpha_ids_fire_in_declaration_order() {
    let mut runtime = build();
    // No publish: the consumer's listener stays quiet after priming, so the
    // fired-set is exactly the two pipe fds.
    prime_drain_connection_noise(&mut runtime);

    let pipe_first = Pipe::new();
    let pipe_second = Pipe::new();
    pipe_first.write_byte();
    pipe_second.write_byte();

    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe_first.read, "zzz_fd"), (pipe_second.read, "aaa_fd")],
        Duration::from_millis(200),
    );

    assert_eq!(
        outcome.constructed_ok,
        vec![true, true],
        "both valid pipe fds must construct"
    );
    assert_eq!(
        outcome.fired,
        vec!["zzz_fd".to_string(), "aaa_fd".to_string()],
        "both written fds fired the same cycle → fired-set is EXACTLY \
         [\"zzz_fd\", \"aaa_fd\"] in DECLARATION (index) order; a sort-by-name \
         regression would yield [\"aaa_fd\", \"zzz_fd\"]. fired = {:?}",
        outcome.fired
    );
}

// ---------------------------------------------------------------------------
// 8. A fd that becomes readable WHILE the reactor is parked wakes it.
//
// Every earlier positive test writes the pipe BEFORE run_once, so the fd is
// already readable at attach and the wait returns immediately — the realistic
// driver edge (a device interrupt arriving mid-park) was uncovered. A spawned
// thread writes the byte ~50ms into a generous (2s) wait: the reactor must wake
// on the write, not sit out the full timeout.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_written_during_wait_wakes_reactor() {
    let mut runtime = build();
    prime_drain_connection_noise(&mut runtime);

    let pipe = Pipe::new();
    let write_fd = pipe.write;
    // The delayed "device": write one byte ~50ms after the reactor parks. RawFd
    // is Copy + Send; the Pipe (and thus both fds) outlives the join below.
    let writer = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(50));
        let b: u8 = 1;
        // SAFETY: `write_fd` is the pipe's valid open write end (the Pipe is
        // alive in the test thread until after join); one byte from a live buffer.
        let n = unsafe { libc::write(write_fd, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
        assert_eq!(n, 1, "delayed pipe write must succeed");
    });

    // Generous timeout: a wake-on-write returns in ~50ms; only a broken
    // mid-park wake path would consume the full 2s (and then fail the asserts).
    let outcome = runtime.run_waitset_reactor_once_with_fds_for_test(
        &[(pipe.read, "fd_node")],
        Duration::from_secs(2),
    );
    writer.join().expect("writer thread must not panic");

    assert!(
        outcome.blocked,
        "the reactor must have actually parked on the WaitSet for this cycle"
    );
    assert!(
        outcome.fired.iter().any(|id| id == "fd_node"),
        "the mid-park pipe write must wake the reactor and record the fd source; \
         fired = {:?}",
        outcome.fired
    );
}

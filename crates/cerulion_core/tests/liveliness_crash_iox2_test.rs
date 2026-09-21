// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof of the TRUE-CRASH liveliness
//! path over real iceoryx2 — a forked child publisher that is SIGKILLed (NO
//! `Drop`), so its registered iceoryx2 port lingers in `number_of_publishers()`
//! until the runtime liveliness sweep's per-sweep
//! `LivelinessCleaner::cleanup_dead_nodes()`
//! reclaims it — at which point the probe count drops and the sweep mints a
//! `Lost` edge.
//!
//! This is the COMPLEMENT to `liveliness_sweep_iox2_test.rs`, which
//! covers the GRACEFUL disconnect path (a same-process publisher's `Drop`
//! decrements the dynamic count immediately, no cleanup needed). Here the child
//! is a SEPARATE PROCESS that crashes — its `Drop` never runs, so reclamation
//! is the ONLY path to `Lost`.
//!
//! ## Config sharing (the make-or-break fact)
//!
//! `cerulion_core::testing::iceoryx_test_config()` wraps iceoryx2's
//! `generate_isolated_config()`, which uses a SHARED root_path but a UNIQUE
//! `prefix` per call — and the prefix is baked into BOTH the service paths AND
//! the node-monitoring registry path. A child that called
//! `iceoryx_test_config()` itself would get a DIFFERENT prefix, so its
//! publisher would be invisible to the parent AND the parent's
//! `try_cleanup_dead_nodes` could not see the dead child's node. So the child
//! MUST join the parent's EXACT `Config` (root_path + prefix). The parent
//! serializes its config (via the test-only `TransportManager::iox_config()`
//! accessor) to a temp file with `serde_json`; the child re-deserializes it and
//! builds its `TransportManager` from it — same SHM region, same registry.
//!
//! ## Determinism contract (LIVE-ONLY)
//!
//! Crash timing + cleanup reclamation are NOT bit-reproducible (a crashed
//! process's port-removal latency is OS-scheduling dependent). So this test
//! asserts PRESENCE, NOT an exact sweep count or `changed_at_ns`: eventually
//! exactly one `Lost` edge fires, with `PublisherDisconnected`, count 0, and
//! the per-node disconnect counter ≥ 1. This is the documented live-only path.
//!
//! ## Hang / orphan safety (HARD CONSTRAINT)
//!
//! A hung cross-process test gating CI is strictly worse than nothing. Every
//! safety net here is mandatory:
//!   - `ChildGuard` is an RAII wrapper whose `Drop` calls `.kill()` then
//!     `.wait()`, so a parent panic / early-return still reaps the child (no
//!     orphan).
//!   - EVERY wait/poll loop has a HARD bounded cap (`for _ in 0..N`), never an
//!     unbounded `while` — a config-sharing failure fail-fasts with an assert
//!     instead of spinning forever.
//!   - After `kill()` the parent `wait()`s to reap the zombie.
//!
//! Known accepted residue: the crashed child's iceoryx2 node registration is
//! reclaimed only by the PARENT's per-sweep `cleanup_dead_nodes`. If the PARENT
//! itself is killed (SIGKILL / CI timeout) after spawning the child but before
//! the sweep reclaims it, the child's isolated SHM prefix can leak — harmless
//! (a small per-test prefix under the OS temp dir, OS-reclaimed), not worth a
//! panic-path cleanup guard for a test.
//!
//! No fake data (Principle #13): the child is a REAL iceoryx2 publisher on a
//! REAL SHM region; the parent observes its port via the REAL sweep probe.
//!
//! MUST run with `--test-threads=1` (serial iceoryx2). NEVER `--workspace`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Env var the parent sets to hand the child the path to the serialized
/// iceoryx2 `Config`. UNSET in a normal `cargo test` run → the child
/// entrypoint is a harmless no-op pass.
const CHILD_CONFIG_ENV: &str = "CER_LIVENESS_CRASH_CHILD_CONFIG";

/// The fixed absolute external topic the child publishes on and the parent
/// graph's single consumer watches. Producer-less in the parent graph (so it
/// takes the `External` provisioning arm — a foreign publisher can attach).
const CRASH_TOPIC: &str = "/crash/cam";

/// Marker the child prints to stdout once its publisher is attached, so a
/// human running the child manually can see it came up. The parent does NOT
/// depend on parsing this (it detects the publisher via the real sweep) — it's
/// purely a manual-run convenience.
const CHILD_READY_MARKER: &str = "CHILD_READY";

// ===========================================================================
// Child entrypoint — re-invoked as a subprocess via `current_exe`.
// ===========================================================================

/// When `CER_LIVENESS_CRASH_CHILD_CONFIG` is UNSET this is a harmless no-op
/// pass (so a normal `cargo test` run isn't disturbed). When SET (the parent
/// re-invokes THIS test binary with `--exact child_publisher_entrypoint`), it:
///   1. deserializes the parent's EXACT iceoryx2 `Config` from the file,
///   2. builds a `TransportManager` joined to that SHM region,
///   3. attaches a publisher on `/crash/cam` (the External topic the parent
///      watches), publishes one frame so the port is live,
///   4. prints `CHILD_READY` + flushes, then sleeps FOREVER.
///
/// The parent SIGKILLs it (`Child::kill`), so the publisher's `Drop` NEVER
/// runs — the crash path. The bounded sleep loop here is just a long-lived
/// idle; the parent's `ChildGuard` guarantees the child is reaped even if the
/// parent panics.
#[test]
fn child_publisher_entrypoint() {
    let config_path = match std::env::var(CHILD_CONFIG_ENV) {
        Ok(p) => p,
        // Not the child invocation — normal `cargo test` run. No-op pass.
        Err(_) => return,
    };

    // Deserialize the parent's EXACT iceoryx2 Config (root_path + prefix) so we
    // join the SAME SHM region + node-monitoring registry. serde_json because
    // iceoryx2 `Config` derives Serialize/Deserialize and serde_json is a
    // first-class dep (no need for iceoryx2's TOML `FilePath` machinery).
    let contents =
        std::fs::read_to_string(&config_path).expect("child: read serialized iceoryx2 config file");
    let ix_config: iceoryx2::config::Config =
        serde_json::from_str(&contents).expect("child: deserialize iceoryx2 config");

    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: "cerulion_crash_child".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
            .expect("child: init_for_test from parent config");

    // Attach a publisher on the External topic. NEVER stored anywhere that
    // would drop it gracefully — the parent SIGKILLs us, so its `Drop` never
    // runs (the whole point: a CRASHED publisher, not a graceful disconnect).
    let mut publisher = mgr
        .create_publisher(CRASH_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("child: create publisher on /crash/cam");

    // Publish one frame so the port is unambiguously live before we signal
    // readiness (loan-and-write-directly — zero copy).
    {
        let mut proxy = publisher
            .loan_proxy::<Vector3>()
            .expect("child: loan a frame");
        proxy.x = 1.0;
        // drop(proxy) publishes the frame (the publish happens on proxy Drop,
        // NOT on publisher Drop — the publisher port itself is what must NOT
        // be gracefully dropped).
    }

    println!("{CHILD_READY_MARKER}");
    use std::io::Write;
    let _ = std::io::stdout().flush();

    // Idle forever; the parent SIGKILLs us. Bounded loop (no unbounded `while`)
    // as belt-and-suspenders: even if the parent somehow never kills us, the
    // child self-terminates after ~5 minutes rather than orphaning forever.
    // `std::mem::forget` would also work but an explicit long idle is clearer.
    for _ in 0..30_000 {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    // If we ever fall out of the loop, leak the publisher so its Drop still
    // does not run (defensive — should be unreachable; the parent kills us).
    std::mem::forget(publisher);
}

// ===========================================================================
// RAII child guard — Drop kills + reaps so a parent panic never orphans.
// ===========================================================================

/// Owns the spawned child `std::process::Child`. `Drop` SIGKILLs then reaps it,
/// so a parent panic / early-return / assertion failure cannot leave an orphan
/// process behind. `kill()` is best-effort (the child may already be dead —
/// e.g. the test killed it explicitly first); `wait()` reaps the zombie.
struct ChildGuard {
    child: std::process::Child,
    reaped: bool,
}

impl ChildGuard {
    fn new(child: std::process::Child) -> Self {
        Self {
            child,
            reaped: false,
        }
    }

    /// Explicitly SIGKILL (no Drop in the child) then reap. Idempotent-safe:
    /// after this, the guard's `Drop` is a no-op.
    fn kill_and_reap(&mut self) {
        if self.reaped {
            return;
        }
        // `Child::kill` sends SIGKILL on Unix — the child gets NO chance to run
        // destructors (exactly the crash semantics we need). Ignore the error:
        // an already-exited child returns InvalidInput, which is fine.
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.reaped = true;
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        // Belt-and-suspenders reaping on any unwind / early return.
        self.kill_and_reap();
    }
}

// ===========================================================================
// Parent test scaffolding — a single-consumer graph watching /crash/cam.
// ===========================================================================

/// Shared observation surface the consumer's handler writes and the test reads.
#[derive(Default, cerulion_core::state::CerulionState)]
struct CrashObs {
    lost_fires: AtomicU64,
    alive_fires: AtomicU64,
    last_publisher_count: AtomicU64,
}

/// Periodic (5 ms) consumer of `/crash/cam` with a `LivelinessEvent` handler
/// that records (not asserts) each edge. Periodic on purpose: the `#[on_event]`
/// dispatch runs on the tick Ok-path, so the node must tick EVERY step to drain
/// a pending liveliness event — a data-trigger node would stop ticking once the
/// crashed publisher's data stops and never dispatch the `Lost` handler.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct CrashConsumer {
    #[input]
    inp: Vector3,
    sum: f64,
    obs: Arc<CrashObs>,
}

#[cerulion_node_impl]
impl CrashConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_live(&mut self, event: LivelinessEvent) {
        assert_eq!(
            event.input_name.as_ref(),
            "inp",
            "the event must carry its input name"
        );
        self.obs
            .last_publisher_count
            .store(event.publisher_count as u64, Ordering::Relaxed);
        match event.state {
            LivelinessState::Lost => {
                assert_eq!(
                    event.cause,
                    LivelinessCause::PublisherDisconnected,
                    "Lost must carry PublisherDisconnected"
                );
                assert_eq!(event.publisher_count, 0, "Lost means no publishers remain");
                self.obs.lost_fires.fetch_add(1, Ordering::Relaxed);
            }
            LivelinessState::Alive => {
                assert_eq!(
                    event.cause,
                    LivelinessCause::PublisherConnected,
                    "Alive must carry PublisherConnected"
                );
                assert!(
                    event.publisher_count >= 1,
                    "Alive means at least one publisher present"
                );
                self.obs.alive_fires.fetch_add(1, Ordering::Relaxed);
            }
            _ => unreachable!("only Lost/Alive exist"),
        }
    }
}

/// Build the watcher graph: a single periodic consumer whose ONLY input is the
/// producer-less absolute external topic `/crash/cam` (so it takes the
/// `External` provisioning arm and a foreign — the child — publisher can
/// attach). 5 ms sweep cadence so one `step_ms(5)` runs one sweep.
fn build_watcher(obs: Arc<CrashObs>) -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "liveliness_crash_test".to_string(),
        prefix: "lcr".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "cons".to_string(),
            node_type: "crash_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: CRASH_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cons".to_string(),
        Box::new(CrashConsumerEntry::with_state(CrashConsumer {
            obs,
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build crash-watcher graph");
    runtime.set_liveliness_sweep_period_for_test(5);
    runtime
}

/// Spawn the child by re-invoking THIS test binary with a filter that runs ONLY
/// `child_publisher_entrypoint`, handing it the serialized-config path via env.
/// `--exact <name>` + `--test-threads=1` runs exactly that one test fn; the env
/// var flips it from no-op into the publisher-crash mode.
fn spawn_child(config_path: &std::path::Path) -> std::io::Result<std::process::Child> {
    let exe = std::env::current_exe()?;
    std::process::Command::new(exe)
        .args([
            "--exact",
            "child_publisher_entrypoint",
            "--test-threads=1",
            "--nocapture",
        ])
        .env(CHILD_CONFIG_ENV, config_path)
        // Inherit stdout/stderr so CHILD_READY + any panic is visible under
        // --nocapture; the parent does not parse it.
        .stdout(std::process::Stdio::inherit())
        .stderr(std::process::Stdio::inherit())
        .spawn()
}

#[test]
#[serial]
fn crash_publisher_eventually_lost() {
    let obs = Arc::new(CrashObs::default());
    let mut runtime = build_watcher(Arc::clone(&obs));

    // Serialize the parent's EXACT iceoryx2 Config (root_path + prefix) to a
    // temp file. The child re-deserializes it to join the SAME SHM region — the
    // ONLY way the parent's sweep + cleanup can see the dead child's port.
    let ix_config = runtime
        .test_transport()
        .expect("test transport parked")
        .iox_config();
    let config_json = serde_json::to_string(&ix_config).expect("serialize iceoryx2 config");
    let tmp = tempfile::TempDir::new().expect("temp dir for child config");
    let config_path = tmp.path().join("child_iox_config.json");
    std::fs::write(&config_path, config_json).expect("write child config file");

    // Spawn the child publisher, wrapped in the RAII guard immediately so an
    // assert below still reaps it.
    let child = spawn_child(&config_path).expect("spawn child publisher process");
    let mut guard = ChildGuard::new(child);

    // (1) BOUNDED wait for the parent to OBSERVE the child's publisher (Alive
    // edge). This fail-fasts (assert, not hang) if config-sharing is broken —
    // the child would be on a different SHM region and never visible. The bound
    // is on ITERATIONS (APPEAR_SWEEPS), and each iteration does a real iceoryx2
    // sweep (the publisher-count probe) PLUS a 5 ms real sleep so the
    // freshly-spawned child has wall-clock time to attach — so the worst-case
    // fail-fast is ~2-3 s of real time, not the iteration count × sim step.
    // (Reclamation/attach latency is OS-dependent — this is the live-only path.)
    const APPEAR_SWEEPS: usize = 400;
    let mut appeared = false;
    for _ in 0..APPEAR_SWEEPS {
        runtime.step_ms(5);
        if obs.alive_fires.load(Ordering::Relaxed) >= 1 {
            appeared = true;
            break;
        }
        // Small real-time yield so the freshly-spawned child has wall-clock time
        // to attach its publisher (the sim clock alone does not wait for it).
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    assert!(
        appeared,
        "parent never observed the child publisher on {CRASH_TOPIC} within \
         {APPEAR_SWEEPS} sweeps — config-sharing likely failed (child on a \
         different SHM prefix). This is a fail-fast, not a hang."
    );
    assert_eq!(
        obs.last_publisher_count.load(Ordering::Relaxed),
        1,
        "the Alive edge reports exactly one live publisher (the child)"
    );
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "no Lost yet — the child is still alive"
    );

    // (2) SIGKILL the child (NO Drop → its publisher port lingers as a CRASHED
    // port) then reap the zombie.
    guard.kill_and_reap();
    // Snapshot the cleanup-call count at the moment of the crash so we can later
    // assert the reclaim thread's `cleanup_dead_nodes` actually RAN during the
    // reclamation window — i.e. that cleanup is the MECHANISM that dropped the
    // count (iceoryx2 does not self-reclaim a dead node's port on a mere probe
    // read), not just that a Lost happened to fire for some other reason.
    let cleanup_calls_before_crash = runtime.liveliness_cleanup_call_count();

    // (3) BOUNDED drive of the sweep. The reclaim thread's `cleanup_dead_nodes`
    // (its own cadence, `LIVELINESS_CLEANUP_PERIOD_MS` = 2 s) reclaims the dead
    // child's node, dropping `number_of_publishers()` to 0 → the next sweep
    // mints a `Lost` edge → the next tick dispatches the handler. Reclamation
    // latency is OS-dependent (live-only), so we assert PRESENCE within a
    // generous bound, not an exact sweep count: the 5 ms real sleep per
    // iteration makes this window >= 6 s of wall time, three reclaim periods.
    const LOST_SWEEPS: usize = 1_200;
    let mut lost = false;
    for _ in 0..LOST_SWEEPS {
        runtime.step_ms(5);
        if obs.lost_fires.load(Ordering::Relaxed) >= 1 {
            lost = true;
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    assert!(
        lost,
        "the crashed child's publisher was never reclaimed within {LOST_SWEEPS} \
         sweeps; the dead-node reclaim did not drop the count. This is a \
         fail-fast, not a hang."
    );

    // (4) Live-only contract: PRESENCE assertions only (crash timing is
    // non-deterministic — no exact sweep count / changed_at_ns).
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "exactly one Lost edge fires when the crashed publisher is reclaimed"
    );
    assert_eq!(
        obs.last_publisher_count.load(Ordering::Relaxed),
        0,
        "the Lost event reports zero live publishers after reclamation"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert!(
        handle.publisher_disconnects_observed_count() >= 1,
        "the per-node disconnect counter bumps on the crash-reclaim Lost edge"
    );
    // The reclaim thread's `cleanup_dead_nodes` ran during the kill→Lost window:
    // proves cleanup is the reclamation MECHANISM (not an incidental Lost). A
    // regression that never started the thread would leave this equal.
    assert!(
        runtime.liveliness_cleanup_call_count() > cleanup_calls_before_crash,
        "the reclaim thread must run cleanup_dead_nodes during the reclamation \
         window (before {cleanup_calls_before_crash}, after {})",
        runtime.liveliness_cleanup_call_count()
    );

    // ChildGuard::Drop is a no-op now (already reaped) — explicit drop for
    // clarity / to keep `tmp` alive until here.
    drop(guard);
    drop(tmp);
}

// SPDX-License-Identifier: AGPL-3.0-only
//! The PRODUCTION egress-plane path over REAL iceoryx2 + zenoh —
//! proof that `GatewayEgressPlane` is NOT inert (the "no inert shipping" rule).
//!
//! A local producer P + the desk daemon's SHARED network manager G (the ONE session
//! netd owns) live on the SAME per-test SHM root; a remote consumer B lives on a
//! DISTINCT root and reaches G ONLY over a real 127.0.0.1 TCP hop. Once an egress
//! plan registers on the plane (which boots the embedded gateway + pushes the topic
//! over the reg-channel), P's raw wire frames flow SHM → G's demand-driven tap →
//! zenoh TCP → B's re-injected local SHM, where B reads them BYTE-IDENTICAL to a
//! HAND oracle (each frame recomputed from its OWN wire `sequence` — never a
//! self-compare, Principle #7). The in-process byte-identity of the tap/forward
//! chain is `cerulion_core`'s `gateway_iox2_test`; this composes it through the
//! netd egress PLANE (which drives the gateway on its own thread — no manual
//! `drive_once`).
//!
//! Parallel-safe (per-test SHM roots + probed ports + isolated scouting-off
//! sessions + unique topics), so NOT `#[serial]` — the `network_ingress_e2e_test` /
//! `gateway_iox2_test` convention.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

use cerulion_core::{GatewayEgressPolicy, GatewayPlan, SchemaServing, GATEWAY_ZERO_DEMAND_IDLE};
use cerulion_netd::egress::{EgressError, EgressPlane, GatewayEgressPlane};

/// The wire schema hash the egress frames carry — B's ingress bridge validates every
/// inbound frame against it before re-injecting (a mismatch is silently dropped), so
/// P's frames + B's `register_ingress_topic` MUST agree. Any value exercises the path.
const EGRESS_HASH: u64 = 0x0837_C5A0_E9E5_0001;

/// A unique id so parallel tests never collide on topic / SHM names.
fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// Probe a free ephemeral TCP port on loopback (the gateway binds it; a
/// probe→rebind race is absorbed by the caller's retry).
fn probe_ephemeral_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral")
        .local_addr()
        .expect("local_addr")
        .port()
}

/// Hand-build the raw wire frame P publishes for logical sequence `seq` — the
/// oracle. Payload = three little-endian f64s `(seq, 2*seq, 3*seq)`; the header's
/// `sequence` is `seq` and `timestamp_ns` a deterministic function of it. Every
/// received frame is recomputed from ITS OWN wire sequence, so byte-equality is a
/// real cross-check.
fn oracle_frame(seq: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    payload.extend_from_slice(&(seq as f64).to_le_bytes());
    payload.extend_from_slice(&(2.0 * seq as f64).to_le_bytes());
    payload.extend_from_slice(&(3.0 * seq as f64).to_le_bytes());
    let header = WireHeader {
        schema_hash: EGRESS_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000 + seq as u64 * 7,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// The producer + shared gateway (machine A) + the booted egress plane, established
/// with a bounded port retry (the probe→rebind steal race).
struct MachineA {
    producer: Arc<TransportManager>,
    plane: Arc<GatewayEgressPlane>,
    port: u16,
}

/// Build machine A: a network-free producer P + the shared network manager G on one
/// root, then boot the egress plane and register the topic's egress plan (which
/// binds G's listen port inside the gateway boot — a bind failure retries).
fn establish_machine_a(tag: &str, id: &str, topic: &str) -> MachineA {
    establish_machine_a_gated(tag, id, topic, None)
}

/// [`establish_machine_a`] plus its optional lost-wakeup-window hook, which
/// MUST be installed before `register_egress` boots the gateway (the runtime is
/// moved onto the drive thread there and the plane keeps no handle on it).
fn establish_machine_a_gated(
    tag: &str,
    id: &str,
    topic: &str,
    idle_wait_gate: Option<Arc<dyn Fn() + Send + Sync>>,
) -> MachineA {
    let root = iceoryx_test_config();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let producer = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("egp_p_{tag}_{id}_{attempt}"),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init producer P");
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("egp_g_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    // The desk egress plane announces under a machine
                    // identity (the net.rs default; here an explicit test value).
                    robot_identity: Some("egpdesk".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init shared gateway G");
        // G is ROBOT-shaped (an identity plus a `tcp/` listen
        // endpoint), so the beacon decides `Advertise` — and the production
        // advertiser would publish a REAL, resolvable `_cerulion._tcp` record on
        // the LAN this test is running on. MEASURED before the suppression seam:
        // `cargo test -p cerulion_netd --test egress_plane_iox2_test` put
        // `egpdesk` on two interfaces for the length of the run, where a
        // concurrent `cerulion topic list` renders it as a live ROBOT and caches
        // its dead ephemeral port in `peers.json` for the 7-day TTL. The DECISION
        // path is unchanged, so the `*` oracles below still bind.
        let plane = Arc::new(GatewayEgressPlane::new_without_mdns_for_test(g));
        if let Some(gate) = idle_wait_gate.clone() {
            plane.set_idle_wait_gate_for_test(gate);
        }
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.to_string()],
            ingress: vec![],
        };
        // register_egress boots the embedded gateway (binds G's port). A bind steal
        // surfaces here → retry with a fresh port.
        match plane.register_egress(0, &plan, &SchemaServing::default(), None) {
            Ok(gateway_started) => {
                assert!(gateway_started, "the first egress plan boots the gateway");
                assert!(plane.gateway_running(), "the gateway thread is up");
                return MachineA {
                    producer,
                    plane,
                    port,
                };
            }
            Err(e) => eprintln!("attempt {attempt}: egress plane boot failed (port {port}): {e}"),
        }
    }
    panic!("could not boot the egress plane's listening gateway in 3 attempts");
}

/// Spawn a background thread on P that publishes `oracle_frame(seq)` for
/// seq=0,1,2,… over `publish_raw` until `stop` flips — the continuous producer the
/// async demand-driven tap needs (the tap has no history; only frames published
/// after it attaches are forwarded).
fn spawn_producer(
    producer: &Arc<TransportManager>,
    topic: &str,
    stop: Arc<AtomicBool>,
) -> JoinHandle<()> {
    let mut pubr = producer
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("P publisher");
    std::thread::spawn(move || {
        let mut seq: u32 = 0;
        while !stop.load(Ordering::Relaxed) {
            let frame = oracle_frame(seq);
            let _ = pubr.publish_raw(&frame);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    })
}

/// Block until `topic` has a bridge FLAG on the shared manager.
///
/// `register_egress` publishes the topic on the iceoryx2 runtime-registration
/// control channel; the gateway's drive thread calls `register_topic` only when it
/// DRAINS that channel, so the flag does not exist the instant
/// `establish_machine_a` returns. Flipping demand before then is a silent no-op
/// (`enable_bridge` returns `Ok(false)` for an unregistered topic), which would make
/// a demand arm assert against a gateway that never saw any demand at all. Bounded
/// in seconds — a generous liveness ceiling, not a cadence assertion.
fn await_registered(plane: &GatewayEgressPlane, topic: &str) {
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        if plane
            .manager()
            .bridge_manager()
            .is_registered(topic)
            .expect("read the registered set")
        {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("'{topic}' never reached the shared bridge manager's registered set");
}

/// This PROCESS's cumulative CPU (user + system).
///
/// PRINTED EVIDENCE ONLY — never asserted on. A CPU gate fails OPEN under load (a
/// starved spinner accumulates LESS CPU and so passes the very check meant to catch
/// it — the class the `topic_observer_iox2_test` flood arms record), and
/// this reading is process-wide, so it also carries zenoh's own threads AND every
/// SIBLING test running in parallel in this binary — the parked-drive-thread
/// arms that hold demand ON are 1 ms-tick loops, and they alone move this number
/// by tens of ms. So it is only meaningful when the arm is run ALONE:
///
/// ```text
/// cargo test -p cerulion_netd --test egress_plane_iox2_test \
///     a_zero_demand_gateway -- --nocapture
/// ```
///
/// The load-bearing oracle is the pass COUNT; this number is here so a human reading
/// the PR sees what the pass count buys on a real machine.
fn process_cpu() -> Duration {
    // SAFETY: `getrusage` fills a caller-owned, fully-initialised `rusage`.
    let mut usage: libc::rusage = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut usage) };
    assert_eq!(rc, 0, "getrusage(RUSAGE_SELF)");
    let secs = |tv: libc::timeval| {
        Duration::from_secs(tv.tv_sec as u64) + Duration::from_micros(tv.tv_usec as u64)
    };
    secs(usage.ru_utime) + secs(usage.ru_stime)
}

/// How long a zero-demand observation window runs. Nominal passes
/// over it are `WINDOW / GATEWAY_ZERO_DEMAND_IDLE` = 5; earlier it was ~1000.
const SAMPLE_WINDOW: Duration = Duration::from_secs(1);

/// A generous LIVENESS ceiling for every bounded wait in this file's wake arms.
///
/// Never a discriminating wall: each wake is now DETERMINED by the time
/// it is polled for, so load can only delay the observation, never change it. The
/// only thing this bounds is a hang.
const LIVENESS_CEILING: Duration = Duration::from_secs(20);

// ────────────────────────────────────────────────────────────────────────────
// The wake arms rendezvous with the drive loop's LOST-WAKEUP WINDOW
// instead of betting that it was parked.
//
// `GatewayRuntime::idle_wait` snapshots the demand generation, scans the egress
// flags, and — finding none ON — bumps `zero_demand_waits` and blocks. A stimulus
// raised while the thread is anywhere OTHER than inside that block is not merely
// "seen late": the scan observes demand as ALREADY ON, so the loop takes the paced
// arm and NEITHER counter moves. That signature is IDENTICAL to the one a gateway
// whose transition never bumped the signal produces, so an arm asserting
// `demand_wakes >= 1` after an unsynchronised flip is betting on a race whose loss
// it cannot even distinguish from the bug it exists to catch.
//
// Such an arm loses that bet on a loaded macOS runner, even on runs
// containing no Rust changes at all. A lost
// race's own diagnostic — `0 wake(s) after 20s (0 further waits)` — is that
// signature exactly: not a slow wake, a wake that could never be posted.
//
// The hook below closes it. It runs ON the drive thread AFTER the generation
// snapshot and AFTER the scan found no demand, and holds the thread there while the
// TEST thread raises the stimulus. `seen` therefore predates the bump on every
// scheduler, so `DemandSignal::wait_for_change` returns `true` whether it blocks
// first or returns immediately — and the wake is a CONSEQUENCE rather than an
// outcome. It also pins the window the production ordering comment calls "the
// lost-wakeup class this repo refuses to ship", which no test reached before.
//
// The stimulus still crosses a THREAD BOUNDARY, which is the production shape (a
// remote demand token arrives on a zenoh callback thread) and the reason the hook
// rendezvouses rather than raising the stimulus itself.
// ────────────────────────────────────────────────────────────────────────────

/// The drive-thread half of the rendezvous: parked in `idle_wait`'s window while
/// the test raises its stimulus.
struct IdleWindowGate {
    /// One-shot, and INERT until the test arms it — so every arm's precondition
    /// still observes the untouched production loop parking on its own.
    armed: AtomicBool,
    /// "I am in the window" → the test.
    reached: std::sync::mpsc::SyncSender<()>,
    /// "the stimulus is raised, carry on" ← the test.
    release: std::sync::Mutex<std::sync::mpsc::Receiver<()>>,
}

/// The test-thread half.
struct WindowRendezvous {
    gate: Arc<IdleWindowGate>,
    reached: std::sync::mpsc::Receiver<()>,
    release: std::sync::mpsc::SyncSender<()>,
}

impl WindowRendezvous {
    fn new() -> Self {
        // Both channels are buffered, never rendezvous channels: a `send` that
        // blocks on the far side being ready would let a panicking test wedge the
        // drive thread, and a wedged drive thread turns a FAILING assertion into a
        // HUNG `drop(plane)` join (the "every failure path takes this
        // route" lesson).
        let (reached_tx, reached_rx) = std::sync::mpsc::sync_channel(1);
        let (release_tx, release_rx) = std::sync::mpsc::sync_channel(1);
        Self {
            gate: Arc::new(IdleWindowGate {
                armed: AtomicBool::new(false),
                reached: reached_tx,
                release: std::sync::Mutex::new(release_rx),
            }),
            reached: reached_rx,
            release: release_tx,
        }
    }

    /// The hook to install on the plane before it boots.
    fn hook(&self) -> Arc<dyn Fn() + Send + Sync> {
        let gate = Arc::clone(&self.gate);
        Arc::new(move || {
            if !gate.armed.swap(false, Ordering::SeqCst) {
                return;
            }
            let _ = gate.reached.send(());
            // BOUNDED: a test that panics between `await_window` and `release`
            // must cost this thread one ceiling, not its life. Returning without
            // the stimulus cannot manufacture a PASS — the generation is then
            // unchanged, the wait times out, and the arm fails on its wake
            // assertion.
            let rx = gate
                .release
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let _ = rx.recv_timeout(LIVENESS_CEILING);
        })
    }

    /// Arm the one-shot: the drive loop's NEXT idle pass stops in the window.
    fn arm(&self) {
        self.gate.armed.store(true, Ordering::SeqCst);
    }

    /// Block until the drive thread is held in the window. Bounded by a liveness
    /// ceiling — at zero demand `idle_wait` runs every pass, so this is at most one
    /// `GATEWAY_ZERO_DEMAND_IDLE` of real waiting.
    fn await_window(&self) {
        self.reached.recv_timeout(LIVENESS_CEILING).expect(
            "the drive thread must reach idle_wait's window (it parks every pass at zero demand)",
        );
    }

    /// Let the held drive thread proceed into its wait.
    fn release(&self) {
        self.release
            .send(())
            .expect("the drive thread is holding the window, so it is still receiving");
    }
}

/// Block until the drive loop has ENTERED a zero-demand wait at least once — the
/// precondition every wake arm needs (a gateway that never parks cannot be woken).
fn await_zero_demand_park(stats: &cerulion_core::GatewayDriveStats) -> u64 {
    let deadline = Instant::now() + LIVENESS_CEILING;
    while Instant::now() < deadline {
        let waits = stats.zero_demand_waits();
        if waits > 0 {
            return waits;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("precondition: the drive thread never took the zero-demand blocking arm");
}

/// Block until `demand_wakes` moves past `before`, and return the delta.
///
/// A pure liveness ceiling: by the time this is called the wake is already
/// DETERMINED (the stimulus bumped the generation while the thread was held in the
/// window with an older snapshot), so contention can delay this observation but
/// cannot change its outcome.
fn await_demand_wake(stats: &cerulion_core::GatewayDriveStats, before: u64) -> u64 {
    let deadline = Instant::now() + LIVENESS_CEILING;
    while Instant::now() < deadline {
        let wakes = stats.demand_wakes();
        if wakes > before {
            return wakes - before;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    0
}

/// Block until the demand signal's generation moves past `before` — POSITIVE
/// evidence that a stimulus raised on another thread has actually landed, rather
/// than a sleep long enough that it probably has.
fn await_generation_past(signal: &cerulion_core::DemandSignal, before: u64) {
    let deadline = Instant::now() + LIVENESS_CEILING;
    while Instant::now() < deadline {
        if signal.generation() != before {
            return;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    panic!("the stimulus never bumped the demand signal's generation (was {before})");
}

/// The zero-demand pass CEILING over [`SAMPLE_WINDOW`].
///
/// A CEILING, never a band — contention can only make a paced loop run FEWER
/// passes, so load can delay this assertion but never invert it (the rate-floor
/// lesson). 60 sits ~12x above the nominal 5 (so scheduler jitter, a slow boot and
/// a couple of spurious condvar wakes are all absorbed) and ~17x BELOW the
/// earlier ~1000, which is the number an unpaced loop produces.
const ZERO_DEMAND_PASS_CEILING: u64 = 60;

/// The DEMANDED loop's pass ceiling, per
/// millisecond of the measured window.
///
/// The idle pacing lives in one place,
/// `GatewayRuntime::idle_wait`, so `sleep(GATEWAY_IDLE_POLL)` is the SINGLE line
/// pacing the demanded loop for `GatewayRuntime::run`, netd's egress drive thread and
/// `graph run-gateway` alike. Deleting it left **388 arms green** while the loop ran
/// at 17,699-44,452 passes/s — a full core, and a standing denial of deep C-states on
/// a Jetson, the exact cost the idle wait exists to remove, re-introduced on the path that
/// runs whenever Studio is attached.
///
/// A CEILING is load-SAFE by arithmetic and is the one direction a floor is not: a
/// 1 ms sleep caps the loop at ~1000 passes/s, so 5 passes/ms can only be exceeded by
/// running FASTER, which contention cannot cause. That is why this file carries no
/// pass-RATE floors (the load-inverted floor class — macOS
/// background QoS takes this loop to 45 passes/s) and still wants this ceiling.
///
/// 5/ms sits ~7x above the measured 655/s and ~9x below an unpaced variant's 44,452/s.
const DEMANDED_PASSES_PER_MS_CEILING: u64 = 5;

/// THE zero-demand pin — a gateway with registered-but-UNDEMANDED
/// egress topics stops re-scanning its flag map a thousand times a second and parks
/// on the demand signal instead.
///
/// This is the half `gateway_iox2_test`'s arms structurally cannot see: those drive
/// `idle_wait` by hand, while this observes a REAL drive thread the netd egress
/// plane owns and started. The oracle is `GatewayDriveStats` read through the
/// plane's `drive_stats()` — a wall cannot express this claim (a spinning loop and a
/// parked one both "take one second"), and the counters are load-safe in the only
/// direction that matters.
#[test]
fn a_zero_demand_gateway_stops_spinning_its_flag_map() {
    let id = unique_id();
    let topic = format!("/egp/idle/{id}");
    let a = establish_machine_a("wkidle", &id, &topic);
    let stats = a
        .plane
        .drive_stats()
        .expect("the gateway booted, so it reports pacing stats");

    // Let boot settle (the plan registration bumps the signal once, and the topic
    // reaches the bridge map only once the drive thread drains the reg channel),
    // then measure a clean window with NO demand anywhere.
    await_registered(&a.plane, &topic);
    std::thread::sleep(Duration::from_millis(300));
    assert!(
        !a.plane
            .manager()
            .bridge_manager()
            .is_enabled(&topic)
            .expect("read the demand flag"),
        "precondition: nothing has demanded this topic"
    );

    let passes_before = stats.passes();
    let waits_before = stats.zero_demand_waits();
    let cpu_before = process_cpu();
    std::thread::sleep(SAMPLE_WINDOW);
    let cpu = process_cpu() - cpu_before;
    let passes = stats.passes() - passes_before;
    let waits = stats.zero_demand_waits() - waits_before;

    println!(
        "{passes} drive passes / {waits} zero-demand waits in \
         {SAMPLE_WINDOW:?} at ZERO demand, process CPU {cpu:?} \
         (earlier: ~1000 passes)"
    );
    assert!(
        waits > 0,
        "anti-tautology: the loop must really be taking the BLOCKING arm (got {waits} \
         waits) — without this a wedged drive thread would satisfy the ceiling below"
    );
    assert!(
        passes <= ZERO_DEMAND_PASS_CEILING,
        "a gateway with NO demand must not spin its flag map: {passes} passes in \
         {SAMPLE_WINDOW:?} exceeds the {ZERO_DEMAND_PASS_CEILING} ceiling (the \
         earlier 1 ms tick produces ~1000)"
    );
}

/// A demand transition WAKES the parked drive thread — measured on
/// the plane's own counters, so a timeout cannot be mistaken for a wake.
///
/// The claim is stated as a COUNTER FLOOR plus a generous liveness ceiling in
/// seconds: `demand_wakes` can only be incremented by a wait that ended on the
/// signal, so load can delay it but never fabricate it (and never satisfy it by
/// accident, which a wall assertion in units of the 200 ms fallback would).
///
/// The counter is the right ORACLE; the STIMULUS is what has to be
/// SYNCHRONISED. An arm that sleeps 300 ms and flips demand lands at a
/// uniformly random point in the drive loop's park cycle — and a flip landing while
/// the thread is AWAKE is not a late wake but an unpostable one: the flag scan then
/// sees demand already ON, the loop takes the paced arm, and neither counter ever
/// moves again. That is why such an arm prints `0 wake(s) ... (0 further waits)`,
/// and why its assertion cannot tell its own lost race from the bug it guards.
/// The flip here happens while the thread is HELD in that window (see
/// `WindowRendezvous`), so the wake is determined rather than raced.
#[test]
fn a_demand_transition_wakes_the_parked_drive_thread() {
    let id = unique_id();
    let topic = format!("/egp/wake/{id}");
    let window = WindowRendezvous::new();
    let a = establish_machine_a_gated("wkwake", &id, &topic, Some(window.hook()));
    let stats = a.plane.drive_stats().expect("gateway booted");
    let signal = a.plane.manager().bridge_manager().demand_signal();

    await_registered(&a.plane, &topic);
    // The gate is still INERT here, so this precondition observes the untouched
    // production loop: it really is parking, with nothing demanded.
    await_zero_demand_park(&stats);
    assert!(
        !a.plane
            .manager()
            .bridge_manager()
            .is_enabled(&topic)
            .expect("read the demand flag"),
        "precondition: nothing has demanded this topic yet"
    );
    let wakes_before = stats.demand_wakes();
    let waits_before = stats.zero_demand_waits();
    let passes_before = stats.passes();

    // Hold the drive thread in the window: past the generation snapshot, past a
    // scan that found no demand, not yet blocked.
    window.arm();
    window.await_window();

    let started = Instant::now();
    let gen_before = signal.generation();
    assert!(
        a.plane
            .manager()
            .bridge_manager()
            .enable_bridge(&topic)
            .expect("enable demand"),
        "a genuine false→true transition (the only edge that bumps the signal)"
    );
    // The transition BUMPED the signal. Asserted directly, and before the counter
    // poll, so a gateway whose `enable_bridge` stopped signalling fails HERE — on
    // the cause — instead of 20 s later on the symptom.
    assert_ne!(
        signal.generation(),
        gen_before,
        "a false→true demand transition must bump the demand signal's generation — \
         without that bump a parked gateway waits out the whole \
         {GATEWAY_ZERO_DEMAND_IDLE:?} fallback before it attaches the tap"
    );
    window.release();

    // Bounded wait for the wake to be POSTED (the counter is written after the wait
    // returns, so a poll is the correct way to read it) — seconds, not fallback
    // units, and a ceiling on OBSERVING an outcome that is already settled.
    let wakes = await_demand_wake(&stats, wakes_before);
    let elapsed = started.elapsed();
    println!(
        "the demand transition posted {wakes} wake(s) after {elapsed:?} \
         ({} further waits)",
        stats.zero_demand_waits() - waits_before
    );
    assert!(
        wakes >= 1,
        "the parked drive thread must be WOKEN by the demand transition, not left to \
         time out — `demand_wakes` is only ever incremented by a wait that ended on \
         the signal, and the transition was raised while the thread was HELD in that \
         wait's own window, so this is not a race that can be lost (waited {elapsed:?})"
    );
    // And the wake did REAL work rather than merely posting a counter: from here on
    // the loop takes the DEMANDED arm and never parks again.
    //
    // Asserted as an EXACT ZERO on the park counter, never as a pass RATE. A rate
    // floor is the load-inverted class and it FIRED here: under `taskpolicy -b`
    // macOS coalesces the demanded loop's 1 ms sleep down to ~45 passes/s against
    // ~780 on an idle desk, so any floor separating "demanded" from "parked" (4/s)
    // is one throttled runner away from inverting. Load cannot inflate a zero.
    let waits_at_wake = stats.zero_demand_waits();
    let passes_at_wake = stats.passes();
    std::thread::sleep(SAMPLE_WINDOW);
    assert_eq!(
        stats.zero_demand_waits() - waits_at_wake,
        0,
        "once demand is ON the woken loop must never park again"
    );
    // Anti-vacuity: the drive thread is ALIVE. Deliberately a tiny floor — it
    // separates "running" from "dead", which is all the zero above needs, and not
    // "1 ms-paced" from "parked", which is what load can invert.
    assert!(
        stats.passes() - passes_at_wake >= 2,
        "the drive thread must still be running (else the zero above proves nothing)"
    );
    let _ = passes_before;
}

/// With demand ON the drive thread NEVER parks — the netd-side
/// twin of `gateway_iox2_test::with_demand_on_the_loop_never_takes_the_blocking_arm`,
/// observed on a real thread. The exact-zero delta is the load-safe assertion; a
/// pass FLOOR would be the load-inverted class (contention lowers it).
#[test]
fn a_demanded_gateway_keeps_the_unchanged_one_ms_pacing() {
    let id = unique_id();
    let topic = format!("/egp/on/{id}");
    let a = establish_machine_a("wkon", &id, &topic);
    let stats = a.plane.drive_stats().expect("gateway booted");

    await_registered(&a.plane, &topic);
    assert!(
        a.plane
            .manager()
            .bridge_manager()
            .enable_bridge(&topic)
            .expect("enable demand"),
        "a genuine false→true transition — an unregistered topic silently returns \
         false and this arm would then be asserting about a gateway with NO demand"
    );
    // Let the drive thread observe the flag (it may be mid-park when the flip
    // lands; the wake is what ends that park).
    std::thread::sleep(Duration::from_millis(300));

    let passes_before = stats.passes();
    let waits_before = stats.zero_demand_waits();
    let measured_from = Instant::now();
    std::thread::sleep(SAMPLE_WINDOW);
    // The ceiling is derived from the window the run ACTUALLY took, not the nominal
    // one: a coalesced sleep overshoots, and charging the loop for wall it was never
    // given is the load-inverted class in miniature (the wake_drain precedent).
    let elapsed = measured_from.elapsed();
    let passes = stats.passes() - passes_before;
    let waits = stats.zero_demand_waits() - waits_before;

    println!(
        "{passes} drive passes / {waits} zero-demand waits in \
         {elapsed:?} with demand ON"
    );
    assert_eq!(
        waits, 0,
        "a DEMANDED gateway must never park — the demand plane's ~1 kHz cadence is \
         what the liveness observer's `queue_emptied`, sustained bands and \
         rate window were all derived against"
    );
    // Anti-vacuity ONLY: the drive thread is alive. Deliberately NOT a rate FLOOR
    // separating "1 ms-paced" from "parked" — that is the load-inverted floor class and
    // it FIRED here, 20/20, under `taskpolicy -b`: macOS background-QoS coalescing
    // took this loop from ~780 passes/s to 45, which is on the wrong side of the
    // parked loop's own ceiling. The EXACT ZERO above is the whole claim, and load
    // cannot inflate a zero.
    assert!(
        passes >= 2,
        "the drive thread must still be running (else the zero above proves nothing)"
    );
    // And it must still be PACED. `sleep(GATEWAY_IDLE_POLL)` is the
    // single line pacing this loop for all three gateway shapes (they share
    // one `idle_wait`), and deleting it left 388 arms green at 44,452 passes/s. A
    // ceiling is the load-SAFE direction (contention makes a paced loop SLOWER, and
    // 5 passes/ms can only be beaten by running faster).
    let ceiling = (elapsed.as_millis() as u64) * DEMANDED_PASSES_PER_MS_CEILING;
    assert!(
        passes <= ceiling,
        "a DEMANDED gateway ran {passes} drive passes in {elapsed:?} (ceiling \
         {ceiling} = {DEMANDED_PASSES_PER_MS_CEILING}/ms) — the ON-path \
         `sleep(GATEWAY_IDLE_POLL)` is gone and every gateway shape is now a busy \
         loop the moment ANY topic is demanded, which is the moment Studio attaches"
    );
}

/// An IN-PROCESS runtime egress registration wakes the parked drive
/// thread, so the new topic is announced now rather than at the fallback tick.
///
/// This is the half of the wake story that is easy to ship inert: without the bump in
/// `TransportManager::register_dynamic_egress_topic` the topic STILL arrives — the
/// gateway drains the control channel on its next pass — so every existing arm, and
/// this file's own `await_registered`, stay green while every desk registration
/// silently costs a `GATEWAY_ZERO_DEMAND_IDLE` window. The `demand_wakes` counter is
/// the only thing that can see the difference.
///
/// Scope, stated: this covers the IN-PROCESS registration only. A registration
/// published from ANOTHER process cannot bump an in-process condvar and is served by
/// the fallback timeout by design — which is why the timeout is a real bound and not
/// a formality.
///
/// This arm carried the SAME exposure as its demand-transition sibling,
/// one order of magnitude smaller — a fresh-park rendezvous left the stimulus racing
/// the 200 ms fallback rather than the whole park cycle. It is gated for the same
/// reason and in the same way: a sibling fixed alone leaves the next slow runner to
/// pick the other (the loaded-runner lesson).
#[test]
fn an_in_process_runtime_registration_wakes_the_parked_drive_thread() {
    let id = unique_id();
    let topic = format!("/egp/reg/{id}");
    let window = WindowRendezvous::new();
    let a = establish_machine_a_gated("wkreg", &id, &topic, Some(window.hook()));
    let stats = a.plane.drive_stats().expect("gateway booted");
    let signal = a.plane.manager().bridge_manager().demand_signal();
    await_registered(&a.plane, &topic);
    await_zero_demand_park(&stats);
    let wakes_before = stats.demand_wakes();

    // Hold the drive thread in the lost-wakeup window, then register.
    window.arm();
    window.await_window();

    // A SECOND graph registers its produced topic on the same live plane.
    let second = format!("/egp/reg2/{id}");
    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![second.clone()],
        ingress: vec![],
    };
    let started = Instant::now();
    let gen_before = signal.generation();
    assert!(
        !a.plane
            .register_egress(7, &plan2, &SchemaServing::default(), None)
            .expect("second egress plan"),
        "the gateway is already running, so this joins rather than boots"
    );
    assert_ne!(
        signal.generation(),
        gen_before,
        "an in-process runtime egress registration must bump the demand signal — \
         this is the bump `TransportManager::register_dynamic_egress_topic` makes, \
         and the one whose absence costs every desk registration a \
         {GATEWAY_ZERO_DEMAND_IDLE:?} window while every other arm stays green"
    );
    window.release();

    let wakes = await_demand_wake(&stats, wakes_before);
    println!(
        "an in-process runtime registration posted {wakes} wake(s) \
         after {:?}",
        started.elapsed()
    );
    assert!(
        wakes >= 1,
        "an in-process runtime egress registration must WAKE the parked drive thread \
         — without it the topic waits out a {GATEWAY_ZERO_DEMAND_IDLE:?} fallback \
         before it is announced, and every other arm stays green"
    );
    // And the wake did real work: the new topic really joined the registered set.
    await_registered(&a.plane, &second);
}

/// Tearing the plane down WAKES the parked drive thread instead of
/// waiting out the fallback.
///
/// `exit` is an `AtomicBool` the idle wait cannot be woken by, so without
/// `RunningGateway::drop` poking the demand signal every netd shutdown — and every
/// `ensure_gateway_booted` re-boot of a dead slot — would pay the full
/// `GATEWAY_ZERO_DEMAND_IDLE`. The claim is asserted on the `demand_wakes` COUNTER,
/// not a wall: a wall tight enough to separate "woken" from "timed out" is ~200 ms,
/// which is exactly the margin macOS background-QoS timer coalescing can eat (the
/// timer-coalescing class). A timeout cannot increment this counter, so load can delay the
/// assertion but never invert it.
///
/// The park was previously observed FRESH before the drop, which left the
/// poke racing the 200 ms fallback — the same class as its two siblings, and the
/// same fix. The drop is held until the thread is in the wait's own window; because
/// `RunningGateway::drop` also JOINS that thread, it runs on a helper and is
/// released only once its poke has LANDED, observed on the signal's own generation
/// rather than on a timer.
#[test]
fn tearing_down_an_idle_plane_wakes_its_parked_drive_thread() {
    let id = unique_id();
    let topic = format!("/egp/drop/{id}");
    let window = WindowRendezvous::new();
    let a = establish_machine_a_gated("wkdrop", &id, &topic, Some(window.hook()));
    let stats = a.plane.drive_stats().expect("gateway booted");
    let signal = a.plane.manager().bridge_manager().demand_signal();
    await_registered(&a.plane, &topic);
    await_zero_demand_park(&stats);
    let wakes_before = stats.demand_wakes();

    // Hold the drive thread in the lost-wakeup window, then tear the plane down.
    window.arm();
    window.await_window();

    // `drop` sets `exit`, pokes the signal, and JOINS — and the thread it joins is
    // the one held in the window, so the drop MUST run off this thread or the
    // release below could never be sent.
    let MachineA { plane, .. } = a;
    let started = Instant::now();
    let gen_before = signal.generation();
    let dropper = std::thread::spawn(move || drop(plane));
    // POSITIVE evidence the teardown poke landed — not a sleep long enough that it
    // probably has. Only then is the held thread let go.
    await_generation_past(&signal, gen_before);
    window.release();
    dropper.join().expect("the teardown thread must not panic");
    let elapsed = started.elapsed();

    let wakes = stats.demand_wakes() - wakes_before;
    println!("idle-plane teardown joined in {elapsed:?} ({wakes} wake(s))");
    assert!(
        wakes >= 1,
        "the teardown must WAKE the parked thread, not wait out the \
         {GATEWAY_ZERO_DEMAND_IDLE:?} fallback — `demand_wakes` is only ever \
         incremented by a wait that ended on the signal (join took {elapsed:?})"
    );
    // Generous liveness ceiling only: a hung join is the failure this catches, and
    // the counter above is what carries the claim.
    assert!(
        elapsed < Duration::from_secs(20),
        "liveness: the teardown join must not hang (took {elapsed:?})"
    );
}

/// The headline pin: the production egress plane is NOT inert — a local producer's
/// frames, once the plan registers, are observable over the REAL zenoh hop by a
/// remote consumer B, BYTE-IDENTICAL to the hand oracle.
#[test]
fn egress_plane_forwards_producer_frames_over_zenoh_to_a_remote_consumer() {
    let id = unique_id();
    let topic = format!("/egp/data/{id}");
    let a = establish_machine_a("fwd", &id, &topic);

    // Start the continuous producer.
    let stop = Arc::new(AtomicBool::new(false));
    let producer_handle = spawn_producer(&a.producer, &topic, Arc::clone(&stop));

    // Machine B (a DISTINCT root, reaching G only over the TCP hop) demands the topic
    // and asserts P's frames arrive byte-identical over the zenoh hop.
    receive_and_verify_over_hop(a.port, &topic, &format!("{id}_b"));

    stop.store(true, Ordering::Relaxed);
    let _ = producer_handle.join();
    // Teardown: dropping the plane stops + joins the gateway drive thread (no hang).
    drop(a.plane);
}

/// Machine B: a DISTINCT root reaching the gateway ONLY over the TCP hop on `port`.
/// B declares the demand token (`register_ingress_topic`) that drives the gateway's
/// egress ON, then collects the re-injected frames and asserts DELIVERY (> 0) +
/// BYTE-IDENTITY to each frame's own-sequence oracle. The gateway's demand watch +
/// reconciler belt flip the egress flag from B's real demand over the hop, then the
/// plane's drive thread taps + forwards — no manual drive.
fn receive_and_verify_over_hop(port: u16, topic: &str, node_suffix: &str) {
    let b = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("egp_b_{node_suffix}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                ..Default::default()
            }),
            ..Default::default()
        },
        iceoryx_test_config(),
    )
    .expect("init B");
    // Create the reader FIRST (the consumer-first order), then the demand.
    let b_sub = b.create_subscriber(topic).expect("B subscriber");
    b.register_ingress_topic(topic, EGRESS_HASH, MaxSliceLen::const_new(256))
        .expect("B register_ingress_topic (the demand token)");

    const WANT: usize = 5;
    let mut got: Vec<(u32, Vec<u8>)> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    while got.len() < WANT && Instant::now() < deadline {
        b_sub
            .try_receive(|msg| {
                let h = msg.header();
                let payload = msg.payload().to_vec();
                let mut full = vec![0u8; WireHeader::SIZE + payload.len()];
                h.write_to_buf(&mut full[..WireHeader::SIZE]);
                full[WireHeader::SIZE..].copy_from_slice(&payload);
                got.push((h.sequence, full));
            })
            .expect("B try_receive");
        if got.len() < WANT {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    // NOT inert: frames crossed the hop.
    assert!(
        got.len() >= WANT,
        "the production egress plane forwarded {}/{WANT} frames over zenoh for '{topic}' (got {}); \
         the plane is inert if 0",
        got.len(),
        got.len()
    );
    // Byte-identical to the hand oracle (recomputed from each frame's OWN sequence).
    for (seq, frame) in &got {
        assert_eq!(
            frame,
            &oracle_frame(*seq),
            "frame seq {seq} on '{topic}' must arrive byte-identical to its own-sequence oracle"
        );
    }
}

/// A SECOND egress plan on the LIVE production plane does
/// NOT re-boot the shared gateway (gateway_started == false) and the new topic flows
/// e2e over the same session — the multi-graph desk shape.
#[test]
fn second_plan_on_the_live_plane_does_not_reboot_and_the_new_topic_flows() {
    let id = unique_id();
    let topic1 = format!("/egp/first/{id}");
    // The FIRST plan boots the embedded gateway.
    let a = establish_machine_a("second", &id, &topic1);

    // A SECOND plan for a DIFFERENT topic on the ALREADY-RUNNING plane → no re-boot.
    let topic2 = format!("/egp/second/{id}");
    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic2.clone()],
        ingress: vec![],
    };
    let booted2 = a
        .plane
        .register_egress(1, &plan2, &SchemaServing::default(), None)
        .expect("second register_egress");
    assert!(
        !booted2,
        "the second plan joins the RUNNING gateway — no second boot"
    );
    assert!(a.plane.gateway_running());

    // topic2 (the second plan's topic) flows e2e over the SAME session.
    let stop = Arc::new(AtomicBool::new(false));
    let producer_handle = spawn_producer(&a.producer, &topic2, Arc::clone(&stop));
    receive_and_verify_over_hop(a.port, &topic2, &format!("{id}_b2"));
    stop.store(true, Ordering::Relaxed);
    let _ = producer_handle.join();
    drop(a.plane);
}

/// A crashed gateway drive thread must not wedge the
/// plane — a subsequent register_egress RE-BOOTS a fresh gateway (rather than silently
/// pushing topics into the dead reg-channel) and the new topic flows e2e.
#[test]
fn a_dead_drive_thread_reboots_on_the_next_register_and_the_topic_flows() {
    let id = unique_id();
    let topic1 = format!("/egp/dead1/{id}");
    let a = establish_machine_a("dead", &id, &topic1);
    assert!(
        a.plane.gateway_running(),
        "the first plan booted the gateway"
    );

    // Simulate the drive thread CRASHING (run() returned Err) via the fault seam.
    a.plane.fault_inject_drive_failure_for_test();
    // Wait for the thread to actually die + RECORD it (the dead flag).
    let deadline = Instant::now() + Duration::from_secs(3);
    while !a.plane.gateway_drive_died_for_test() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        a.plane.gateway_drive_died_for_test(),
        "the faulted drive thread marks the slot dead"
    );

    // A SECOND register RE-boots the gateway (gateway_started true again) — NOT a
    // silent no-op that leaves the dead gateway wedged in place.
    let topic2 = format!("/egp/dead2/{id}");
    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic2.clone()],
        ingress: vec![],
    };
    let rebooted = a
        .plane
        .register_egress(1, &plan2, &SchemaServing::default(), None)
        .expect("re-register after the drive thread died");
    assert!(
        rebooted,
        "a dead gateway RE-BOOTS on the next register (gateway_started true)"
    );
    assert!(a.plane.gateway_running());
    assert!(
        !a.plane.gateway_drive_died_for_test(),
        "the re-booted gateway is healthy"
    );

    // topic2 flows e2e over the RE-BOOTED gateway (proving it is not a dead channel).
    let stop = Arc::new(AtomicBool::new(false));
    let producer_handle = spawn_producer(&a.producer, &topic2, Arc::clone(&stop));
    receive_and_verify_over_hop(a.port, &topic2, &format!("{id}_bd"));
    stop.store(true, Ordering::Relaxed);
    let _ = producer_handle.join();
    drop(a.plane);
}

/// A register_egress carrying a MATCHING forwarded iceoryx2 namespace
/// (the plane's OWN shared-session config) is VERIFIED + accepted, and the topic flows
/// e2e — the multi-process desk path. A same-machine run resolves the DEFAULT `iox2_` data
/// plane, == netd's shared session, so the forward matches and egresses.
#[test]
fn register_with_a_matching_forwarded_config_is_accepted_and_the_topic_flows() {
    let id = unique_id();
    let topic1 = format!("/egp/cfgmatch1/{id}");
    let a = establish_machine_a("cfgmatch", &id, &topic1);

    // The plane's shared-session config, serialized — the EXACT namespace a same-box
    // run forwards. A MATCHING forward passes verification and joins the running
    // gateway (no re-boot), then egresses.
    let matching = serde_json::to_string(&a.plane.manager().iox_config())
        .expect("serialize the plane manager config");
    let topic2 = format!("/egp/cfgmatch2/{id}");
    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic2.clone()],
        ingress: vec![],
    };
    let booted2 = a
        .plane
        .register_egress(2, &plan2, &SchemaServing::default(), Some(&matching))
        .expect("a matching forwarded config is accepted (verification passes)");
    assert!(
        !booted2,
        "a matching-namespace register joins the running gateway (no re-boot)"
    );

    let stop = Arc::new(AtomicBool::new(false));
    let producer_handle = spawn_producer(&a.producer, &topic2, Arc::clone(&stop));
    receive_and_verify_over_hop(a.port, &topic2, &format!("{id}_bcm"));
    stop.store(true, Ordering::Relaxed);
    let _ = producer_handle.join();
    drop(a.plane);
}

/// A register_egress carrying a MISMATCHING namespace (a DIFFERENT
/// prefix — a run that resolved a divergent `global_config()`) is REFUSED loudly with
/// `NamespaceMismatch` naming BOTH identities. netd's gateway taps its OWN namespace,
/// so a foreign-namespace run is un-tappable here; the CLI degrades it to a per-run
/// gateway child on the run's own namespace rather than being silently tapped on the
/// wrong one (egressing nothing).
#[test]
fn register_with_a_mismatching_forwarded_config_is_refused() {
    let id = unique_id();
    let topic1 = format!("/egp/cfgmis1/{id}");
    let a = establish_machine_a("cfgmis", &id, &topic1);

    // Start from the plane's REAL config JSON, then flip ONLY the prefix to a distinct
    // namespace the plane's manager cannot see (FileName serializes as a plain string,
    // so a Value swap re-parses into a valid Config).
    let mut v: serde_json::Value =
        serde_json::from_str(&serde_json::to_string(&a.plane.manager().iox_config()).unwrap())
            .unwrap();
    let real_prefix = v["global"]["prefix"].as_str().unwrap().to_string();
    v["global"]["prefix"] = serde_json::json!("cer_mismatch_");
    let mismatch = serde_json::to_string(&v).unwrap();

    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/egp/cfgmis2/{id}")],
        ingress: vec![],
    };
    let err = a
        .plane
        .register_egress(3, &plan2, &SchemaServing::default(), Some(&mismatch))
        .expect_err("a mismatching forwarded config is refused");
    match &err {
        EgressError::NamespaceMismatch { run, netd } => {
            assert_eq!(
                run.prefix, "cer_mismatch_",
                "the refusal carries the RUN's prefix"
            );
            assert_eq!(
                netd.prefix, real_prefix,
                "and netd's own shared-session prefix"
            );
        }
        other => panic!("expected NamespaceMismatch, got {other:?}"),
    }
    // The LOUD diagnostic names both namespaces + the divergent-config hint.
    let msg = err.to_string();
    assert!(
        msg.contains("does NOT match"),
        "loud mismatch diagnostic: {msg}"
    );
    assert!(
        msg.contains("cer_mismatch_") && msg.contains(&real_prefix),
        "names BOTH the run and netd prefixes: {msg}"
    );
    assert!(
        msg.contains("IOX2_CONFIG_FILE"),
        "names the usual cause (divergent config): {msg}"
    );
    drop(a.plane);
}

/// A register_egress whose forwarded config shares
/// netd's `(root_path, prefix)` but flips ONLY `global.service.directory` is
/// REFUSED — a `(root_path, prefix)`-only identity would wrongly MATCH here,
/// so netd would tap its OWN (empty) service directory and the run would silently
/// egress nothing. iceoryx2 discovers a service's static config under
/// `root_path + service.directory`, so a divergent directory is a distinct
/// discovery namespace. Refusing it degrades the run to a per-run gateway child on
/// its own namespace (the CLI's `establish_network_egress` warn-but-run fallback).
#[test]
fn register_with_a_divergent_service_directory_is_refused() {
    let id = unique_id();
    let topic1 = format!("/egp/svcdirmis1/{id}");
    let a = establish_machine_a("svcdirmis", &id, &topic1);

    // Start from the plane's REAL config JSON, keep (root_path, prefix) IDENTICAL,
    // flip ONLY `global.service.directory` (Path serializes as a plain string, so a
    // Value swap re-parses into a valid Config). The sanity assert on the default
    // value ALSO validates the JSON key path — a wrong key would be Null.
    let mut v: serde_json::Value =
        serde_json::from_str(&serde_json::to_string(&a.plane.manager().iox_config()).unwrap())
            .unwrap();
    let real_prefix = v["global"]["prefix"].as_str().unwrap().to_string();
    let real_service_dir = v["global"]["service"]["directory"]
        .as_str()
        .expect("service.directory is a string (JSON key path)")
        .to_string();
    v["global"]["service"]["directory"] = serde_json::json!("cer_other_services");
    let mismatch = serde_json::to_string(&v).unwrap();

    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/egp/svcdirmis2/{id}")],
        ingress: vec![],
    };
    let err = a
        .plane
        .register_egress(3, &plan2, &SchemaServing::default(), Some(&mismatch))
        .expect_err("a divergent service.directory is refused");
    match &err {
        EgressError::NamespaceMismatch { run, netd } => {
            // The prefix MATCHES — the refusal came from the directory divergence,
            // which a `(root_path, prefix)`-only identity could not see.
            assert_eq!(run.prefix, real_prefix, "the run's prefix matches netd's");
            assert_eq!(netd.prefix, real_prefix, "netd's prefix");
            assert_eq!(
                run.service_dir, "cer_other_services",
                "the run's divergent service directory"
            );
            assert_eq!(
                netd.service_dir, real_service_dir,
                "netd's own service directory"
            );
            assert_ne!(
                run.service_dir, netd.service_dir,
                "the identities differ ONLY on service.directory"
            );
        }
        other => panic!("expected NamespaceMismatch, got {other:?}"),
    }
    // The LOUD diagnostic names both service directories + the divergent-config hint.
    let msg = err.to_string();
    assert!(
        msg.contains("does NOT match") && msg.contains("cer_other_services"),
        "loud mismatch diagnostic names the divergent directory: {msg}"
    );
    drop(a.plane);
}

/// The same refusal for a divergent
/// `global.node.directory` — a distinct node-discovery directory is a distinct
/// namespace even with `(root_path, prefix, service.directory)` all identical.
#[test]
fn register_with_a_divergent_node_directory_is_refused() {
    let id = unique_id();
    let topic1 = format!("/egp/nodedirmis1/{id}");
    let a = establish_machine_a("nodedirmis", &id, &topic1);

    let mut v: serde_json::Value =
        serde_json::from_str(&serde_json::to_string(&a.plane.manager().iox_config()).unwrap())
            .unwrap();
    assert_eq!(
        v["global"]["node"]["directory"], "nodes",
        "sanity: default node directory (JSON key path)"
    );
    v["global"]["node"]["directory"] = serde_json::json!("cer_other_nodes");
    let mismatch = serde_json::to_string(&v).unwrap();

    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/egp/nodedirmis2/{id}")],
        ingress: vec![],
    };
    let err = a
        .plane
        .register_egress(3, &plan2, &SchemaServing::default(), Some(&mismatch))
        .expect_err("a divergent node.directory is refused");
    match &err {
        EgressError::NamespaceMismatch { run, netd } => {
            assert_eq!(run.node_dir, "cer_other_nodes", "the run's node directory");
            assert_eq!(netd.node_dir, "nodes", "netd's default node directory");
            assert_eq!(
                run.prefix, netd.prefix,
                "prefix matches — only node_dir differs"
            );
        }
        other => panic!("expected NamespaceMismatch, got {other:?}"),
    }
    drop(a.plane);
}

/// An UNPARSEABLE forwarded config is refused LOUDLY (a corrupt/wrong
/// forward never silently taps on an unverified namespace).
#[test]
fn register_with_a_garbage_forwarded_config_is_refused() {
    let id = unique_id();
    let topic1 = format!("/egp/cfggarbage1/{id}");
    let a = establish_machine_a("cfggarbage", &id, &topic1);
    let plan2 = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/egp/cfggarbage2/{id}")],
        ingress: vec![],
    };
    let err = a
        .plane
        .register_egress(4, &plan2, &SchemaServing::default(), Some("not a config"))
        .expect_err("a garbage forwarded config is refused");
    assert!(
        matches!(err, EgressError::Gateway { .. }),
        "an unparseable config is a loud Gateway error, got {err:?}"
    );
    assert!(
        err.to_string().contains("not a valid Config"),
        "the error names the parse failure: {err}"
    );
    drop(a.plane);
}

// ────────────────────────────────────────────────────────────────────────────
// The `_cerulion._tcp` mDNS BEACON is raised by the netd-hosted
// serving plane.
//
// The advertise was first put in the standalone `graph run-gateway` child; a later
// change folded the permissive gateway plane into netd and that path spawns NO such
// child, so a robot serving topics through netd advertised nothing at all
// (Measured on the Go2: 86 topics served, `dns-sd -B _cerulion._tcp local.` empty from
// a desk on the same /24). These two arms pin the wiring at the one seam that
// fixes it — the embedded gateway's boot.
//
// The oracle is `mdns_beacon_decision()`, NOT `is_advertising_mdns()`, and that
// is the whole point: a DESK correctly declines and reports `false`, which is
// byte-indistinguishable from a plane that never consulted the beacon at all. So
// a `false` reading proves nothing and the recorded DECISION proves everything.
//
// WHAT KEEPS THESE ARMS OFF THE MULTICAST SOCKET is the suppression seam, NOT
// the decision oracle. The decision being
// "recorded whether or not `ServiceDaemon::new()` succeeds" is NOT enough,
// and fails in the direction that matters: `ensure_raised` records the
// decision AND THEN advertises, so every robot-shaped fixture in this file
// published a real, resolvable `_cerulion._tcp` record on the developer's or
// runner's LAN (MEASURED: `egpdesk`, two interfaces, visible to `dns-sd -B`).
// `establish_machine_a` therefore builds its plane with
// `GatewayEgressPlane::new_without_mdns_for_test`, which changes NOTHING about
// the decision and only declines to open the socket.
//
// The DESK arm below deliberately keeps the PRODUCTION constructor — a desk
// declines before the advertiser is reached, so it is safe there AND it is the
// only place that can prove production still installs the real advertiser
// (`mdns_advertiser_is_real()`). The ADVERTISING side needs a real LAN and lives
// in the `#[ignore]`d `cerulion_cli_engine/tests/mdns_live_test.rs`.
// ────────────────────────────────────────────────────────────────────────────

/// THE headline: booting the embedded egress gateway raises this machine's
/// beacon, carrying the SHARED SESSION's own identity and bound listen port.
///
/// `establish_machine_a` builds G the ROBOT way — an explicit `robot_identity`
/// plus a `tcp/127.0.0.1:<probed>` listen endpoint, exactly the shape
/// `CERULION_NETD_LISTEN` produces on a real robot — so the expected decision is
/// a HAND oracle over values this test chose, never a self-compare: a plane that
/// read a hardcoded `None` would report `NoNetwork`, one that read the wrong
/// config would report the wrong port.
#[test]
fn the_egress_gateway_boot_raises_this_machines_mdns_beacon() {
    let id = unique_id();
    let topic = format!("/egp/beacon/{id}");

    // ANTI-TAUTOLOGY: a plane that has booted no gateway has consulted no
    // beacon. Without this the assertion below could be satisfied by a decision
    // taken at construction — which would advertise a port nothing is bound to,
    // since netd's zenoh session is LAZY.
    let idle = GatewayEgressPlane::new(
        TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("egp_beacon_idle_{id}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec!["tcp/127.0.0.1:1".to_string()],
                    robot_identity: Some("egpdesk".to_string()),
                    ..Default::default()
                }),
                ..Default::default()
            },
            iceoryx_test_config(),
        )
        .expect("init an un-booted plane's manager"),
    );
    assert_eq!(
        idle.mdns_beacon_decision(),
        None,
        "a plane whose gateway never booted must not have decided anything — the \
         beacon is raised at BOOT, when the session's listener is really bound"
    );
    assert!(!idle.is_advertising_mdns());
    drop(idle);

    let a = establish_machine_a("beacon", &id, &topic);
    assert_eq!(
        a.plane.mdns_beacon_decision(),
        Some(cerulion_netd::beacon::BeaconDecision::Advertise {
            robot: "egpdesk".to_string(),
            port: a.port,
        }),
        "the embedded gateway's boot must raise the mDNS beacon under the \
         SHARED session's identity and its bound listen port"
    );
    drop(a.plane);
}

/// The DESK control — the anti-tautology half, and the reason the gate is the
/// LISTEN endpoint rather than the identity.
///
/// netd stamps `robot_identity` (the hostname) on EVERY machine at init
/// (the egress plane needs it to announce), so a beacon gated on identity
/// would make every laptop running a desk graph announce itself as a robot in
/// `cerulion topic list`'s ROBOTS section — and publish an SRV record pointing at
/// a port nothing is listening on. A desk netd is given no `CERULION_NETD_LISTEN`,
/// so it has no dialable port; here G carries an identity and NO listen endpoint,
/// which is exactly what `desk_network_config` builds.
#[test]
fn a_desk_shaped_netd_boots_its_gateway_and_advertises_nothing() {
    let id = unique_id();
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("egp_beacon_desk_{id}"),
            network: Some(NetworkConfig {
                // The DESK shape: an identity (netd always stamps one) and NO
                // listen endpoint.
                listen_endpoints: vec![],
                robot_identity: Some("egpdesk".to_string()),
                ..Default::default()
            }),
            ..Default::default()
        },
        iceoryx_test_config(),
    )
    .expect("init a desk-shaped shared manager");
    // The PRODUCTION constructor, deliberately — a desk declines before the
    // advertiser is ever reached, so this arm opens no socket AND is the one
    // place that can prove production still installs the real advertiser.
    let plane = GatewayEgressPlane::new(g);
    assert!(
        plane.mdns_advertiser_is_real(),
        "GatewayEgressPlane::new must install the REAL mDNS advertiser — a plane \
         built with the test-only suppressed one would take every decision, log \
         every line and advertise NOTHING, which is exactly the original beacon defect"
    );
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![format!("/egp/beacondesk/{id}")],
        ingress: vec![],
    };
    assert!(
        plane
            .register_egress(0, &plan, &SchemaServing::default(), None)
            .expect("a desk egress plan registers"),
        "the first egress plan boots the gateway on a desk too"
    );
    assert!(plane.gateway_running(), "the gateway thread is up");

    // It CONSULTED the beacon (so the wiring is live on this path too) and
    // DECLINED, naming the premise it lacked.
    assert_eq!(
        plane.mdns_beacon_decision(),
        Some(cerulion_netd::beacon::BeaconDecision::NoListenPort { listen: vec![] }),
        "a desk consults the beacon and declines for want of a dialable port"
    );
    assert!(
        !plane.is_advertising_mdns(),
        "a desk must NEVER advertise itself as a robot over `_cerulion._tcp`"
    );
    drop(plane);
}

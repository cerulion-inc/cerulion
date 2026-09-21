// SPDX-License-Identifier: AGPL-3.0-only
//! Consumer-first spawn acceptance: a CONSUMER-ONLY worker spawning FIRST does not kill the
//! deployment — the supervisor pre-creates every graph-owned topic service at
//! the FULL finalized config BEFORE any worker spawns (in
//! `graph_run_supervisor`), so the early consumer can only OPEN, never win the
//! create race and under-provision.
//!
//! # The regression shape (verified in-source)
//!
//! One producer topic `/{prefix}/prod/cmd`, consumed by FIVE inputs across two
//! groups:
//!
//! * group `early` (listed FIRST in `process_groups:`) holds ONLY `c_snap`
//!   (`test_node_macro_period_input_cdylib`: `period_ms = 10` + a NON-trigger
//!   `#[input] inp`). Levelization derives from TRIGGER edges only
//!   (`build_trigger_edges`), and a plain `#[input]` contributes none — so
//!   `c_snap` sits at global level 0, tying `prod`'s level.
//! * group `main` (listed second) holds `prod`
//!   (`test_node_macro_period_cdylib`, output `cmd`) + FOUR data-trigger
//!   forwarders `dt0..dt3` (`test_node_macro_data_trigger_cdylib`, level 1).
//!
//! `spawn_order` sorts by (min owned global level, rank); both groups' min
//! level is 0, and rank = `process_groups:` listing order — so the tie-break
//! spawns the CONSUMER-ONLY `early` worker FIRST. That is the consumer-first shape:
//! without pre-creation, `early`'s build reaches `/{prefix}/prod/cmd` as a producer-less
//! absolute external source (the cross-group subgraph rewrite) and its
//! create-or-open leg CREATES the service at iceoryx2's port-cap defaults
//! (`max_subscribers` unset ⇒ create-default 8); the producer worker then opens
//! it requiring MORE and dies pre-READY.
//!
//! # The ceiling arithmetic (in-source: `runtime.rs` `in_graph_subs` +
//! `TopicServiceConfig::for_topology`)
//!
//! Every number below is DERIVED from
//! [`INTROSPECTION_SUBSCRIBER_HEADROOM`](cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM)
//! and [`IOX2_DEFAULT_MAX_SUBSCRIBERS`], never hard-coded: the whole point of
//! this test is a CROSSING of the create-default, and a literal would go on
//! passing while silently describing (and proving) something else. That is
//! exactly what happened when the introspection headroom was raised 4 → 5: the local view
//! grew to 9 and crossed the default ON ITS OWN, so the stamped-union
//! attribution below stopped holding while the test stayed green.
//!
//! `DT_CONSUMERS` is pinned at `IOX2_DEFAULT_MAX_SUBSCRIBERS - HEADROOM` so that,
//! for `/{prefix}/prod/cmd`:
//!
//! * the producer worker's LOCAL subgraph view is `DT_CONSUMERS + HEADROOM`
//!   = EXACTLY `IOX2_DEFAULT_MAX_SUBSCRIBERS` — it would open a default-created
//!   service by equality, so it cannot be what forces the crossing;
//! * the FULL (monolith planning) build sees `DT_CONSUMERS + 1` body
//!   subscribers (c_snap.inp non-trigger + `dt*.trigger_in`), so
//!   `max_subscribers` = `DT_CONSUMERS + 1 + HEADROOM`
//!   = `IOX2_DEFAULT_MAX_SUBSCRIBERS + 1` — ONE over the default.
//!
//! `IOX2_DEFAULT_MAX_SUBSCRIBERS` is itself a hand-written MIRROR of an iceoryx2
//! create-default (there is no public constant for it), so `DT_CONSUMERS` is only
//! as accurate as that mirror: `assert_default_subscriber_cap_mirror_is_live` reads
//! the LIVE value off the resolved iceoryx2 config and fails loudly if the two
//! ever part company. The two inequalities above are invariants BY CONSTRUCTION
//! of `DT_CONSUMERS`'s definition, NOT independently checkable claims — see the
//! note on the `const _` block below.
//!
//! So it is the stamped cross-group union (carrying `c_snap`'s edge)
//! that pushes the requirement over the consumer-created default, exactly the
//! humanoid per-node-baseline death class. (Trigger-drain subscribers are 0 —
//! the data-trigger fixture exports the drain symbol, so every input is
//! UNIFIED and counts +1 `extra_event_listeners`, not a second subscriber. Under
//! a forced `CERULION_DRAIN_DISCIPLINE=separate` run the count only grows, so
//! the crossing holds either way; the LOCAL-view equality does not, which is why
//! that seam is left at its default here.)
//!
//! # What this test asserts (POST-fix behavior only)
//!
//! 1. The pre-create breadcrumb appears BEFORE any "spawned worker
//!    process" evidence in the same supervisor log stream.
//! 2. The GO breadcrumb appears (both workers READY — the deployment survived
//!    the consumer-first spawn).
//! 3. The consumer-first DEATH SIGNATURE is ABSENT — asserted explicitly and FIRST so
//!    the mutation check (neutering the pre-create block) fails HERE,
//!    naming the regression: "before signaling READY" (the supervisor's
//!    pre-READY worker-death abort) and "fewer subscriber slots" (the
//!    ordering-edge open refusal in `transport/mod.rs`).
//! 4. Clean SIGINT shutdown, exit 0.
//! 5. Delivery evidence via the `CERULION_MP_TRACE_DIR` fire-trace seam (JSON,
//!    no ANSI/format concerns): some `dt*` consumer fired > 0 (a data-trigger
//!    node fires ONLY on a delivered frame — real cross-topic delivery through
//!    the pre-created service) and `c_snap` fired > 0 in ITS worker's trace
//!    (the early worker genuinely ran).
//!
//! No failing arm is built in — the death shape is reached by neutering the
//! pre-create block, which is the mutation check for this file.
//!
//! A SECOND `#[serial]` arm
//! (`precreate_failure_aborts_deployment_before_any_spawn`) pins the pre-create
//! FAILURE contract behaviorally: a pre-existing INCOMPATIBLE service on a
//! graph-owned topic — hand-minted from the test process on the default
//! namespace and HELD across the launch — aborts the deployment NONZERO with
//! the `supervisor_precreate_error` operator surface (topic + likely cause +
//! `cerulion clean` remedy) and ZERO worker spawn evidence, making
//! "Deployment aborted BEFORE any worker spawned" a live promise, not prose.
//!
//! # Harness
//!
//! Crib of `mp_default_ns_e2e_test.rs` / the shared `mp_support` module:
//! tempdir workspace, PREBUILT fixture cdylibs (no in-test cargo build),
//! real-binary subprocess `graph run`, bounded waits + `ChildGuard`
//! process-GROUP teardown on drop, file-redirected stdio (no TTY ⇒ tracing emits no ANSI, so
//! plain substring greps are color-safe). Prerequisites (the helpers PANIC
//! with the instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_period_input_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]` (the mp supervisor is real on macOS too).
//! `#[serial]`: the mp data plane is the global default iceoryx2 namespace
//! (the unique `mp666` prefix keeps topic names apart from sibling suites).

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

/// The supervisor's deployment-live breadcrumb (the same pin the sibling mp
/// suites use).
const GO_MARKER: &str = "GO signaled; deployment live";

/// The pre-create success breadcrumb (a stable substring of the supervisor's
/// `info!` message).
const PRECREATE_MARKER: &str = "pre-created every graph-owned topic service";

/// The supervisor's per-worker spawn breadcrumb — the "worker spawn evidence"
/// the pre-create breadcrumb must precede.
const SPAWN_MARKER: &str = "spawned worker process";

/// Consumer-first death signature, half 1: the supervisor's pre-READY worker-death
/// abort (`wait_for_ready_or_exit`).
const DEATH_PRE_READY: &str = "before signaling READY";

/// Consumer-first death signature, half 2: the ordering-edge subscriber-slot open
/// refusal hint (`transport/mod.rs`, the
/// `DoesNotSupportRequestedAmountOfSubscribers` arm) the producer worker dies
/// with when a consumer-created default-cap service already occupies the topic.
const DEATH_SLOT_HINT: &str = "fewer subscriber slots";

/// iceoryx2 0.9.1's create-default `max_subscribers` — the cap a service gets
/// when it is created with no explicit subscriber requirement, which is exactly
/// what a consumer-first worker's create-or-open leg mints without pre-creation
/// (the consumer-first death). Not derivable from a public iceoryx2 constant, so it is stated here
/// and cross-checked against the LIVE value by
/// [`assert_default_subscriber_cap_mirror_is_live`] rather than assumed silently.
const IOX2_DEFAULT_MAX_SUBSCRIBERS: usize = 8;

/// Data-trigger consumers on `prod/cmd`, DERIVED so the discriminator survives a
/// headroom change: the producer worker's LOCAL view
/// (`DT_CONSUMERS + HEADROOM`) lands EXACTLY on the create-default (opens by
/// equality), while the full stamped union (`DT_CONSUMERS + 1 + HEADROOM`, the
/// `c_snap` edge included) is ONE over it. See the module-doc arithmetic.
const DT_CONSUMERS: usize =
    IOX2_DEFAULT_MAX_SUBSCRIBERS - cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;

/// The one compile-time condition on [`DT_CONSUMERS`] that can actually FAIL.
///
/// The two inequalities the discriminator rests on — the producer worker's LOCAL
/// view `DT_CONSUMERS + HEADROOM` landing AT the create-default, and the stamped
/// union `DT_CONSUMERS + 1 + HEADROOM` landing ONE OVER it — are invariants BY
/// CONSTRUCTION of [`DT_CONSUMERS`]'s definition (substituting it reduces them to
/// `MAX <= MAX` and `MAX + 1 > MAX`), so asserting them here would be a
/// tautology that can never fire and would only LOOK like a guard. The genuinely
/// unguarded input is [`IOX2_DEFAULT_MAX_SUBSCRIBERS`] — a hard-coded MIRROR of
/// an iceoryx2 create-default that nothing in the build checks; that one is
/// guarded at RUNTIME by [`assert_default_subscriber_cap_mirror_is_live`].
///
/// What remains here is real: raise the headroom to (or past) the create-default
/// and `DT_CONSUMERS` reaches 0 — no data-trigger consumer, no delivery oracle,
/// and (usize) a subtraction overflow — so the build fails instead of the suite
/// silently degrading.
const _: () = {
    assert!(
        DT_CONSUMERS >= 1,
        "at least one data-trigger consumer is needed for the delivery oracle"
    );
};

/// Guard the ONE unguarded input to the discriminator arithmetic:
/// [`IOX2_DEFAULT_MAX_SUBSCRIBERS`] is a hard-coded mirror of iceoryx2's
/// publish-subscribe create-default `max_subscribers`, and nothing else pins it.
/// If iceoryx2 changes that default (or a local/CI iceoryx2 config file
/// overrides it), the mirror silently goes stale, the derived [`DT_CONSUMERS`]
/// lands somewhere else, and BOTH tests below keep passing while no longer
/// isolating the stamped cross-group union they exist to guard.
///
/// The LIVE value is read from the same resolved iceoryx2 config the mp data
/// plane uses (`Config::global_config()` — the default namespace both this test
/// process's squatter and the `graph run` subprocess resolve), via
/// `cerulion_core`'s `test-helpers` accessor. Cheap: a config field read, no
/// service, no SHM.
fn assert_default_subscriber_cap_mirror_is_live() {
    let live = cerulion_core::transport::iceoryx2_default_max_subscribers();
    assert_eq!(
        live, IOX2_DEFAULT_MAX_SUBSCRIBERS,
        "IOX2_DEFAULT_MAX_SUBSCRIBERS ({IOX2_DEFAULT_MAX_SUBSCRIBERS}) \
         no longer mirrors iceoryx2's live create-default max_subscribers ({live}). \
         DT_CONSUMERS is derived from it, so this file's whole discriminator (LOCAL \
         view AT the default, stamped union ONE OVER) is void until the mirror is \
         corrected — update the constant and re-check the module-doc arithmetic"
    );
}

/// The mirror guard as its own `#[test]`, so a stale mirror turns the suite RED
/// even when the two e2e arms are filtered out (they each call the same guard
/// first, so a filtered single-test run is guarded too).
#[test]
#[serial]
fn iox2_default_max_subscribers_mirror_matches_the_live_iceoryx2_default() {
    assert_default_subscriber_cap_mirror_is_live();
}

/// Hand-build the workspace in `root`: `[workspace]` Cargo.toml +
/// `graphs/mp666.yaml` + per-type fixture-source copies (for the metadata
/// walkers) + prebuilt fixture cdylibs under `target/debug/` (no in-test cargo
/// build — the `mp_support::build_mp_workspace` recipe with this file's node
/// set).
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    // prod = period source (output `cmd`); c_snap = period node with a
    // NON-trigger `#[input] inp` (the level-0 consumer-only group's node);
    // dt = the data-trigger forwarder (input `trigger_in`, output `cmd`) —
    // one TYPE, four instances.
    let sources = [
        ("prod", "test_node_macro_period_cdylib"),
        ("c_snap", "test_node_macro_period_input_cdylib"),
        ("dt", "test_node_macro_data_trigger_cdylib"),
    ];
    for (node_type, fixture) in sources {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    // The rank-tie mechanics: `early` is listed FIRST ⇒ rank 0; both groups'
    // min owned global level is 0 (c_snap has no incoming TRIGGER edge, so it
    // levelizes at 0 beside prod) ⇒ `spawn_order`'s (level, rank) tie-break
    // spawns the consumer-only worker FIRST, which is the shape under test.
    let mut yaml = format!(
        "name: mp666\n\
         prefix: {prefix}\n\
         process_groups:\n\
         \x20 early:\n\
         \x20 - c_snap\n\
         \x20 main:\n\
         \x20 - prod\n"
    );
    for i in 0..DT_CONSUMERS {
        yaml.push_str(&format!("\x20 - dt{i}\n"));
    }
    yaml.push_str(
        "nodes:\n\
         - id: c_snap\n\
         \x20 type: c_snap\n\
         \x20 inputs:\n\
         \x20 - name: inp\n\
         \x20\x20\x20 source: prod/cmd\n\
         \x20 outputs:\n\
         \x20 - name: out\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: prod\n\
         \x20 type: prod\n\
         \x20 inputs: []\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n",
    );
    for i in 0..DT_CONSUMERS {
        yaml.push_str(&format!(
            "- id: dt{i}\n\
             \x20 type: dt\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20\x20\x20 source: prod/cmd\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n"
        ));
    }
    std::fs::write(root.join("graphs/mp666.yaml"), yaml).unwrap();
}

/// Spawn `cerulion graph run mp666 --no-validate` (NO `--record`, NO
/// `--single-process` — a REAL default multi-process run), stdio redirected to
/// files, with the `CERULION_MP_TRACE_DIR` fire-trace seam armed (workers
/// inherit the env and dump `trace_{group}.json` on clean exit — the delivery
/// oracle).
fn spawn_graph_run(root: &Path, trace_dir: &Path) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", "mp666", "--no-validate"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .env("CERULION_MP_TRACE_DIR", trace_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    let guard =
        ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run (multi-process)");
    (guard, stdout_path, stderr_path)
}

/// stdout + stderr merged (tracing's writer choice must not decide a pin).
fn merged_log(stdout_path: &Path, stderr_path: &Path) -> String {
    format!("{}\n{}", read_file(stdout_path), read_file(stderr_path))
}

/// Bounded poll until either log file contains `needle` OR the child exits
/// (one final drain read after an exit, so a fast mutation-run death returns
/// promptly instead of burning the whole deadline). Returns whether the needle
/// appeared.
fn wait_for_log_or_exit(
    guard: &mut ChildGuard,
    stdout_path: &Path,
    stderr_path: &Path,
    needle: &str,
    deadline: Duration,
) -> bool {
    let start = Instant::now();
    loop {
        if read_file(stdout_path).contains(needle) || read_file(stderr_path).contains(needle) {
            return true;
        }
        if guard.try_wait_noting().expect("try_wait").is_some() {
            // Exited: the redirected files are flushed by process exit — one
            // final read decides.
            return read_file(stdout_path).contains(needle)
                || read_file(stderr_path).contains(needle);
        }
        if start.elapsed() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The `run-worker` children of a supervisor pid (crib of
/// `mp_default_ns_e2e_test.rs::worker_pids`).
fn worker_pids(supervisor_pid: u32) -> Vec<u32> {
    let out = Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", "run-worker"])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Bounded poll until the supervisor has >= `n` `run-worker` children.
fn wait_for_workers(supervisor_pid: u32, n: usize, deadline: Duration) -> Vec<u32> {
    let start = Instant::now();
    loop {
        let pids = worker_pids(supervisor_pid);
        if pids.len() >= n || start.elapsed() >= deadline {
            return pids;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Count fire-trace entries for nodes whose id starts with `node_prefix` in a
/// worker's `CERULION_MP_TRACE_DIR` dump (`[{node_id, step, fire_time_ns,
/// global_level}, ..]` — structured JSON, no ANSI/formatter concerns).
fn fires_in_trace(trace_path: &Path, node_prefix: &str) -> usize {
    let raw = read_file(trace_path);
    assert!(
        !raw.is_empty(),
        "worker fire-trace dump `{}` must exist after a clean exit (the \
         CERULION_MP_TRACE_DIR seam writes it on both clean and poisoned paths)",
        trace_path.display()
    );
    let v: serde_json::Value = serde_json::from_str(&raw).expect("trace dump parses as JSON");
    v.as_array()
        .expect("trace dump is a JSON array")
        .iter()
        .filter(|e| {
            e["node_id"]
                .as_str()
                .is_some_and(|id| id.starts_with(node_prefix))
        })
        .count()
}

/// THE pin: a consumer-only worker spawning first (rank-tie by listing
/// order at level 0) does not under-provision-create the producer topic —
/// the supervisor pre-created it at the full finalized config before any
/// spawn, the deployment goes live, delivers, and exits 0 on Ctrl-C.
///
/// Mutation contract: with the
/// pre-create block neutered, this test fails at the
/// death-signature ABSENCE asserts below — naming the pre-READY worker death +
/// the subscriber-slot open refusal — because `early` creates `prod/cmd` at
/// the iceoryx2 default cap 8 and `main`'s stamped-union open requires 9.
#[test]
#[serial]
fn consumer_first_spawn_survives_via_supervisor_precreation() {
    assert_default_subscriber_cap_mirror_is_live();
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "mp666";
    build_workspace(tmp.path(), prefix);
    let trace_dir = tmp.path().join("traces");
    std::fs::create_dir_all(&trace_dir).unwrap();

    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &trace_dir);
    let sup_pid = guard.id();

    // Deployment live, OR an early death (the mutation run exits fast — the
    // or-exit wait keeps the failure prompt instead of burning the deadline).
    let went_live = wait_for_log_or_exit(
        &mut guard,
        &stdout_path,
        &stderr_path,
        GO_MARKER,
        Duration::from_secs(120),
    );
    let log = merged_log(&stdout_path, &stderr_path);

    // (3) FIRST, the death-signature ABSENCE pins — ordered before the GO
    // assert so the mutation check (neutered pre-create) fails HERE,
    // attributably naming the regression rather than a generic no-GO timeout.
    assert!(
        !log.contains(DEATH_PRE_READY),
        "consumer-first regression: a worker died BEFORE signaling READY — the \
         consumer-first create race is back (supervisor pre-creation missing \
         or inert); log:\n{log}"
    );
    assert!(
        !log.contains(DEATH_SLOT_HINT),
        "consumer-first regression: a worker was refused at open against an \
         under-provisioned service ('{DEATH_SLOT_HINT}') — the consumer-only \
         early worker won the create race at the iceoryx2 default subscriber \
         cap (8) against the producer's stamped-union requirement (9); log:\n{log}"
    );

    // (2) The deployment went live (both workers READY despite the
    // consumer-only worker spawning first).
    assert!(
        went_live,
        "the GO breadcrumb proves the deployment survived the consumer-first \
         spawn; log:\n{log}"
    );

    // (1) The pre-create breadcrumb appears BEFORE any worker spawn evidence,
    // in the SAME log stream (both lines come from the supervisor's tracing
    // subscriber, so they share one file; file-redirected stdio means tracing
    // emits no ANSI and byte offsets order the events).
    let mut ordering_checked = false;
    for path in [&stdout_path, &stderr_path] {
        let stream = read_file(path);
        if let Some(pre_idx) = stream.find(PRECREATE_MARKER) {
            let spawn_idx = stream.find(SPAWN_MARKER).unwrap_or_else(|| {
                panic!(
                    "the stream carrying the pre-create breadcrumb must also \
                     carry the worker spawn evidence; stream:\n{stream}"
                )
            });
            assert!(
                pre_idx < spawn_idx,
                "the supervisor must pre-create ALL graph-owned topic \
                 services BEFORE spawning any worker (breadcrumb at byte \
                 {pre_idx} vs first spawn at byte {spawn_idx}); stream:\n{stream}"
            );
            // SHAPE self-check: the FIRST spawned worker is the CONSUMER-ONLY
            // `early` group (the (level, rank) tie-break this whole regression
            // shape rests on). If a future levelization change reorders the
            // spawn, this fails LOUDLY instead of silently degrading the pin
            // (and the mutation check would stop failing).
            let line_start = stream[..spawn_idx].rfind('\n').map_or(0, |i| i + 1);
            let line_end = stream[spawn_idx..]
                .find('\n')
                .map_or(stream.len(), |i| spawn_idx + i);
            let first_spawn_line = &stream[line_start..line_end];
            assert!(
                first_spawn_line.contains("early"),
                "the FIRST spawned worker must be the consumer-only `early` \
                 group (spawn_order (level, rank) tie-break by listing order) \
                 — the regression shape requires consumer-first; first \
                 spawn line was: {first_spawn_line}"
            );
            ordering_checked = true;
        }
    }
    assert!(
        ordering_checked,
        "the pre-create breadcrumb ('{PRECREATE_MARKER}') must appear \
         in the supervisor log; log:\n{log}"
    );

    // Belt-and-suspenders "really mp": 2 live `run-worker` children.
    let pids = wait_for_workers(sup_pid, 2, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "a 2-group deployment must have >= 2 run-worker children, saw {pids:?}"
    );

    // A data window: prod ticks at 50ms, so ~30 fires reach the dt consumers
    // through the pre-created service before shutdown.
    std::thread::sleep(Duration::from_millis(1500));

    // (4) Clean shutdown: directed SIGINT to the supervisor (the production
    // Ctrl-C; the supervisor fans it out to the workers) -> exit 0.
    send_signal(sup_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "mp run must exit 0 on Ctrl-C, got {status:?}\nlog:\n{}",
        merged_log(&stdout_path, &stderr_path)
    );

    // (5) Delivery evidence via the fire-trace seam: a data-trigger node fires
    // ONLY on a delivered frame, so dt fires > 0 proves real frames flowed
    // from prod's publisher through the PRE-CREATED service to its consumers.
    let dt_fires = fires_in_trace(&trace_dir.join("trace_main.json"), "dt");
    assert!(
        dt_fires > 0,
        "the data-trigger consumers must have fired (> 0) — delivery through \
         the pre-created topic service; trace_main.json had 0 dt entries"
    );
    // The early (consumer-only, first-spawned) worker genuinely ran: c_snap is
    // period-driven (10ms), so a live worker records fires.
    let snap_fires = fires_in_trace(&trace_dir.join("trace_early.json"), "c_snap");
    assert!(
        snap_fires > 0,
        "the early worker's c_snap must have fired (> 0) — the consumer-only \
         first-spawned worker ran to shutdown; trace_early.json had 0 entries"
    );
}

/// The pre-create FAILURE contract, behaviorally: a pre-existing INCOMPATIBLE
/// service occupying a graph-owned topic on the data plane aborts the
/// deployment NONZERO — carrying the `supervisor_precreate_error` operator
/// surface — BEFORE any worker spawns.
///
/// The conflicting service is hand-minted from THIS test process on the
/// DEFAULT iceoryx2 namespace (the mp data plane — the same resolved
/// global config `mint_deployment_ix_config` snapshots) via the singleton
/// `TransportManager::get_or_init()` + a default-opener `create_subscriber`,
/// and HELD across the subprocess launch (`_squatter` lives to the end of the
/// test; iceoryx2 tears a service down only when the last Node handle drops).
/// A default-opener creation under-provides on TWO axes the supervisor's
/// pre-create open leg REQUIRES for `/{prefix}/prod/cmd`: `max_subscribers` unset
/// ⇒ `IOX2_DEFAULT_MAX_SUBSCRIBERS` < the stamped union
/// `DT_CONSUMERS + 1 + HEADROOM` (the module-doc arithmetic, whose mirror term is
/// guarded live by [`assert_default_subscriber_cap_mirror_is_live`]), and
/// `subscriber_max_borrowed_samples` unset ⇒ 2 < the held-input
/// snapshot raise's Some(3). Whichever axis iceoryx2 verifies first refuses
/// the open; both map through `supervisor_precreate_error` — the asserts are
/// axis-independent.
///
/// Orphan discipline: the abort precedes the spawn loop, so no worker process
/// ever exists (pinned via zero SPAWN_MARKER lines); the `ChildGuard` covers
/// the supervisor itself on any panic path.
#[test]
#[serial]
fn precreate_failure_aborts_deployment_before_any_spawn() {
    assert_default_subscriber_cap_mirror_is_live();
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "mp666f";
    build_workspace(tmp.path(), prefix);
    let trace_dir = tmp.path().join("traces");
    std::fs::create_dir_all(&trace_dir).unwrap();

    // Mint + HOLD the squatter: a default-namespace manager (the singleton —
    // this is the test PROCESS's handle, independent of the deployment's
    // subprocesses) creates the owned topic's service at default-opener caps.
    let mgr = cerulion_core::TransportManager::get_or_init()
        .expect("default-namespace transport for the squatter service");
    let topic = format!("/{prefix}/prod/cmd");
    let _squatter = mgr
        .create_subscriber(&topic)
        .expect("create the under-provisioned squatter service (default-opener caps)");

    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &trace_dir);
    let status = guard
        .wait_bounded(Duration::from_secs(120))
        .expect("the deployment must exit (refused at the pre-create) within the bounded window");
    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        !status.success(),
        "a deployment whose owned topic is squatted by an incompatible service \
         must exit NONZERO, got {status:?}\nlog:\n{log}"
    );

    // The supervisor_precreate_error operator surface, end-to-end across the
    // process boundary (the binary's `Error:` print — RUST_LOG-independent).
    // Topic needle is lead-slash-agnostic (matches the derived absolute name).
    for needle in [
        "mp666f/prod/cmd",
        "another deployment",
        "stale",
        "cerulion clean",
        "BEFORE any worker spawned",
    ] {
        assert!(
            log.contains(needle),
            "the pre-create abort surface must reach the operator ('{needle}'); log:\n{log}"
        );
    }

    // The behavioral half of the promise: ZERO worker spawn evidence — the
    // abort happened before the spawn loop, so no worker process ever existed
    // and there is nothing to orphan.
    assert!(
        !log.contains(SPAWN_MARKER),
        "the pre-create failure must abort BEFORE any worker spawns; log:\n{log}"
    );
    // And the refused deployment never went live.
    assert!(
        !log.contains(GO_MARKER),
        "a refused deployment must never reach GO; log:\n{log}"
    );
}

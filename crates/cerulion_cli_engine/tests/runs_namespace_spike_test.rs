// SPDX-License-Identifier: AGPL-3.0-only
//! THE NAMESPACE PROOF behind the `runs` verb's serve
//! side.
//!
//! The `runs` verb answers "which runs are live on this machine" by GATHERING
//! `/__cerulion/runs` at serve time, in whichever process hosts the gateway. The
//! run registry is keyed by **iceoryx2 namespace**, so the whole design rests on
//! one claim: *the gateway host shares the namespace the run's registry
//! writer publishes on.* This file proves it — or refutes it loudly.
//!
//! # What the production seams actually are
//!
//! | Role | Namespace it resolves | Evidence |
//! |---|---|---|
//! | The run's registry WRITER | `Config::global_config()`, **always** | `run_dir.rs` calls `RunHandle::publish` — ONE call site, above the deployment dispatch, so monolith and multi-process share it |
//! | netd's shared gateway (host A) | `TransportManager::init` ⇒ the global config | `cerulion_netd/src/main.rs` |
//! | The per-run gateway CHILD (host B) | `TransportManager::detached_with_config(cfg, ix)` where `ix` is the handoff's `ix_config_json`, else the global config | `graph_cmd::graph_run_gateway` |
//! | The mp supervisor's OWN manager | the THROWAWAY planning namespace `cer_p_{hex}` | `multiprocess::planning_ix_config` |
//!
//! So "thread the manager's config" is right for a gateway HOST and wrong for
//! the supervisor — see
//! [`the_supervisors_planning_namespace_is_not_where_the_run_is_announced`].
//!
//! # Why a multi-process run takes the same route
//!
//! One might expect a multi-process run to reach the registry "by a
//! different route": netd REFUSES on a namespace mismatch and the run falls back
//! to a per-run gateway child on its own namespace. **That mechanism does not
//! exist.** The multi-process DATA PLANE runs on
//! the resolved global config (`multiprocess::mint_deployment_ix_config` returns
//! `Config::global_config().clone()`; `graph` and `nonce` are ignored), and
//! `IceoryxShmIdentity` deliberately excludes the dead-node-cleanup flags — the
//! only field where netd's `resolve_singleton_node_config` and a raw
//! `global_config()` clone still differ. So a multi-process run's forwarded
//! config MATCHES netd's, netd ACCEPTS, and the SHARED gateway serves both
//! shapes.
//!
//! The conclusion (reachable) holds, for that reason. Two consequences
//! worth stating:
//!
//! 1. The per-run gateway child is still a real production shape — it is reached
//!    by a Strict `network:` posture, by netd being unavailable, or by netd and
//!    the run genuinely resolving different `global_config()`s (a divergent
//!    `IOX2_CONFIG_FILE`). It is NOT reached by "the run is multi-process".
//! 2. **The obvious substitution is inert.** "Thread the global config
//!    instead of the manager's" cannot fail a multi-process arm, because on the
//!    shipping code they are the SAME config. The arm that kills it is
//!    [`only_a_gather_threading_the_hosts_own_config_sees_that_hosts_run`],
//!    where the host is on a namespace that is not the global one — exactly the
//!    per-run-child-after-a-real-mismatch shape.
//!
//! # How the arms model production, and what that costs
//!
//! Production puts the writer and both gateway hosts on `global_config()`. A
//! test that used the global namespace would be neither parallel-safe nor
//! hermetic: it would see every other run on the machine and every sibling test's.
//! So each arm mints an ISOLATED namespace and puts the writer and the host on
//! it — preserving the RELATIONSHIP under test (identity equality) while
//! dropping the shared-global accident.
//!
//! That substitution is the arms' one weakness, and it is closed by TWO arms
//! that have no model in them at all, rather than waved at:
//!
//! - [`the_shipping_deployment_config_resolves_the_registry_writers_namespace`]
//!   asserts over the REAL production VALUES that the config a multi-process
//!   supervisor hands its workers and its gateway child resolves the same SHM
//!   identity the registry writer publishes on. If the design ever reverts to a
//!   run-scoped data plane, that arm fails and takes the multi-process
//!   reachability claim down with it, loudly.
//! - [`a_production_minted_run_record_is_gatherable_by_a_netd_shaped_host`]
//!   drives the REAL `run_dir::start_run_descriptor` — production's own minted
//!   `run_id`, its own directory writer, its own `RunHandle::publish` — and
//!   gathers the result from a netd-shaped host. It is the arm that proves the
//!   serve side sees what production actually WRITES, not only what this file
//!   crafts.
//!
//! # On the crafted records the namespace arms publish
//!
//! The namespace arms publish a hand-built [`RunRecord`] through the REAL
//! `RunHandle::publish_on_config`, over the real registry service, gathered by
//! the real gather. The record is INPUT to that transport, in the same sense as
//! the hand-built wire frames in `bag_record_run_attach_test`, and every claim
//! the arms make about it — which namespace can see it — is provably independent
//! of its field values: `gather_from_node_checked` decodes the record, dedupes
//! by `run_id`, and never reads `run_started_at_ns` or stats `run_dir`.
//!
//! It is crafted for a structural reason and not a convenience one:
//! `start_run_descriptor` takes no config parameter, so the production path can
//! ONLY publish on the global namespace — the one namespace on which two hosts
//! cannot be told apart, which is exactly the discrimination
//! [`only_a_gather_threading_the_hosts_own_config_sees_that_hosts_run`] exists
//! to make. Using it in the namespace arms would delete the property under test.
//! The `RunFixture` still writes a REAL run directory holding the artifacts the
//! verb reads, because a `run_dir` pointing at nothing is a record no production
//! writer can emit — see [`RunFixture`].
//!
//! # Discipline
//!
//! Isolated per-test SHM roots and unique run ids ⇒ parallel-safe. The single
//! exception is the production-path arm, which is `#[serial]` because
//! `start_run_descriptor` publishes on the process-global namespace and mutates
//! `CERULION_HOME`; it asserts only about its own `run_id` and never that the
//! namespace is otherwise empty. Every wait is a bounded CONDITION with a
//! seconds-scale ceiling and a panic naming what never happened; nothing is
//! asserted in units of the registry's republish interval (macOS
//! background-QoS coalescing charges sleep slack per wakeup).

#![cfg(unix)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{
    gather_runs_on_config, RunGather, RunHandle, RunRecord, RunState,
};
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, RealClock, SchemaServing, TransportConfig, TransportManager,
};

use cerulion_cli_engine::multiprocess::{mint_deployment_ix_config, planning_ix_config};
use cerulion_netd::egress::{EgressError, EgressPlane, GatewayEgressPlane};

/// The gather window every arm uses. Generous relative to the registry's
/// republish interval, and never the thing being asserted — a gather EARLY-EXITS
/// the instant it has heard from every live writer, so a healthy arm returns in
/// milliseconds and this is only the ceiling on an unhealthy one.
const WINDOW: Duration = Duration::from_millis(600);

/// How long an arm waits for a just-published run to become gatherable before
/// reporting that it never did. A liveness ceiling against milliseconds of work:
/// load can delay the record, it cannot make it arrive wrong.
const APPEAR_BUDGET: Duration = Duration::from_secs(10);

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn nanos() -> u128 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos()
}

/// RAII guard: set `CERULION_HOME` to `dir` and restore the prior value on drop
/// (panic-safe — a failed assert never leaks the override into a sibling test).
/// Used only by the production-path arm, which is `#[serial]` for that reason.
struct HomeGuard(Option<String>);
impl HomeGuard {
    fn set(dir: &std::path::Path) -> Self {
        let prev = std::env::var("CERULION_HOME").ok();
        std::env::set_var("CERULION_HOME", dir);
        HomeGuard(prev)
    }
}
impl Drop for HomeGuard {
    fn drop(&mut self) {
        match &self.0 {
            Some(v) => std::env::set_var("CERULION_HOME", v),
            None => std::env::remove_var("CERULION_HOME"),
        }
    }
}

/// A run id unique to this call, so an arm's assertions are about ITS run even
/// when a real `graph run` is live on the developer's box.
fn fresh_run_id() -> u128 {
    nanos() ^ (u128::from(COUNTER.fetch_add(1, Ordering::Relaxed)) << 96) ^ 0x1238_0000_0000_0000
}

/// A run's REAL on-disk directory, plus the record that points at it.
///
/// The record is hand-built, and the directory it names really exists and really
/// holds the artifacts the verb reads. That combination is deliberate, and the
/// reasoning is worth stating because it is the line between a legitimate
/// crafted input and a fabricated one:
///
/// - The record is **input** to the transport, exactly as the hand-built frames
///   in `bag_record_run_attach_test` are. Everything the arms assert about it —
///   which namespace can gather it — is provably independent of its field
///   values: `gather_from_node_checked` decodes the wire record, dedupes by
///   `run_id`, and never reads `run_started_at_ns` or stats `run_dir`.
/// - The **directory is not input, it is a claim.** A `run_dir` naming a path
///   that does not exist is a record no production writer can emit
///   (`start_run_descriptor` creates the directory FIRST and publishes the
///   pointer SECOND, precisely so a fast gatherer cannot read a half-written
///   run), and arm 1 justifies its `run_dir` assertion by what the verb reads out
///   of it. A dangling pointer cannot support that claim, so the fixture makes
///   it real and arm 1 reads it back.
///
/// The production path itself — `start_run_descriptor`, minting its own
/// `run_id`, writing its own directory and publishing through
/// `RunHandle::publish` — is driven end to end by
/// [`a_production_minted_run_record_is_gatherable_by_a_netd_shaped_host`].
struct RunFixture {
    dir: tempfile::TempDir,
    record: RunRecord,
}

impl RunFixture {
    /// Write a run directory holding the four artifacts a real run writes, and
    /// mint the record that points at it.
    fn new(run_id: u128, graph: &str) -> Self {
        let dir = tempfile::tempdir().expect("run directory");
        let path = dir.path();
        std::fs::write(
            path.join("graph.yaml"),
            format!("name: {graph}\nprefix: {graph}\nnodes: []\n"),
        )
        .expect("graph.yaml");
        std::fs::write(
            path.join("run.json"),
            serde_json::to_vec_pretty(&serde_json::json!({
                "run_id": format!("{run_id:032x}"),
                "graph_name": graph,
            }))
            .expect("render run.json"),
        )
        .expect("run.json");
        std::fs::write(path.join("env.json"), b"{}").expect("env.json");
        std::fs::write(path.join("recorder.json"), b"{}").expect("recorder.json");

        let record = RunRecord {
            run_id,
            supervisor_pid: std::process::id(),
            // Wall clock, like the production spec's `run_started_at_ns`.
            run_started_at_ns: u64::try_from(nanos()).expect("ns fits u64"),
            state: RunState::Live,
            graph_name: graph.to_string(),
            run_dir: path.display().to_string(),
        };
        Self { dir, record }
    }

    fn record(&self) -> RunRecord {
        self.record.clone()
    }

    fn path(&self) -> &std::path::Path {
        self.dir.path()
    }
}

fn transport_config(node_name: &str) -> TransportConfig {
    TransportConfig {
        node_name: node_name.to_string(),
        clock: Arc::new(RealClock),
        subscriber_buffer_size: 16,
        network: None,
    }
}

/// A gateway host built the way **netd** builds its shared manager — one
/// manager, one namespace, nothing handed in from a run.
///
/// (`init_for_test` rather than `init`: `init` is the process singleton and
/// resolves the GLOBAL namespace, which is what the module docs explain these
/// arms deliberately do not use. The property under test — that a gather on
/// `manager.iox_config()` reaches a writer on the same namespace — is identical.)
fn netd_shaped_host(node_name: &str, ns: &iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(transport_config(node_name), ns.clone())
        .expect("netd-shaped gateway host manager")
}

/// A gateway host built the way the **per-run gateway child** builds its manager:
/// `TransportManager::detached_with_config`, on the namespace the handoff carried
/// (`graph_cmd::graph_run_gateway`). The PRODUCTION constructor, not a stand-in.
fn child_shaped_host(node_name: &str, ns: &iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::detached_with_config(transport_config(node_name), ns.clone())
        .expect("per-run gateway child manager")
}

/// **The serve side, reduced to its one load-bearing
/// decision: WHICH namespace the gather runs on.**
///
/// The serve side threads the gateway host manager's own config. The regression
/// is reverting to a fixed config instead — `iceoryx2::config::Config::global_config()`.
/// Flipping this one line is how the kill claims in this file
/// were measured.
fn serve_side_gather(host: &TransportManager, window: Duration) -> RunGather {
    gather_runs_on_config(&host.iox_config(), window).expect("run-registry gather")
}

/// Poll the serve side until it reports `run_id`, or fail naming what never
/// happened.
fn gather_until_run(host: &TransportManager, run_id: u128, what: &str) -> RunRecord {
    let start = Instant::now();
    loop {
        let gather = serve_side_gather(host, WINDOW);
        if let Some(r) = gather.records.iter().find(|r| r.run_id == run_id) {
            return r.clone();
        }
        assert!(
            start.elapsed() < APPEAR_BUDGET,
            "timed out after {APPEAR_BUDGET:?} waiting for: {what} (last gather: {} record(s), \
             completeness {:?})",
            gather.records.len(),
            gather.completeness,
        );
        std::thread::sleep(Duration::from_millis(25));
    }
}

/// The SHM-discovery identity of a config, via the same public entry point
/// netd's namespace check uses on a forwarded run config
/// (`cerulion_core::ix_config_shm_identity_from_json`). There is no public
/// `IceoryxShmIdentity::from_config`, and going through the JSON is the more
/// faithful route anyway — it is literally what crosses the netd control seam.
fn identity_of(config: &iceoryx2::config::Config) -> cerulion_core::IceoryxShmIdentity {
    let json = serde_json::to_string(config).expect("serialize an iceoryx2 Config");
    cerulion_core::ix_config_shm_identity_from_json(&json).expect("SHM identity from config JSON")
}

fn config_json(config: &iceoryx2::config::Config) -> String {
    serde_json::to_string(config).expect("serialize an iceoryx2 Config")
}

// ===========================================================================
// SHAPE 1 — monolith `graph run` + netd's shared gateway
// ===========================================================================

/// A monolith run publishes its record on the namespace netd's shared gateway is
/// already on, so the serve side sees EXACTLY that run.
///
/// In production both sides are `Config::global_config()`: the writer via
/// `RunHandle::publish`, netd's manager via `TransportManager::init`. netd's namespace
/// check is not even consulted — a monolith forwards `ix_config_json: None`,
/// which short-circuits it ("shares netd's default namespace by construction").
#[test]
fn monolith_run_is_reachable_from_the_netd_hosted_gateway() {
    let ns = iceoryx_test_config();
    let host = netd_shaped_host("netd_host", &ns);

    let run_id = fresh_run_id();
    let fixture = RunFixture::new(run_id, "perception");
    let _run = RunHandle::publish_on_config(&ns, fixture.record()).expect("publish the run record");

    let seen = gather_until_run(
        &host,
        run_id,
        "the monolith run to appear to netd's gateway",
    );

    assert_eq!(seen.run_id, run_id, "the gather must report OUR run");
    assert_eq!(seen.graph_name, "perception");
    assert_eq!(seen.state, RunState::Live);
    assert_eq!(seen.supervisor_pid, std::process::id());
    assert_eq!(
        seen.run_dir,
        fixture.path().display().to_string(),
        "the run-dir POINTER must survive the gather"
    );
    // …and the pointer must be USABLE, not merely intact. The serve side reads
    // `graph.yaml` + `run.json` out of exactly this path, so the arm performs
    // that read rather than asserting a string and asserting the read works.
    let graph_yaml =
        std::fs::read_to_string(std::path::Path::new(&seen.run_dir).join("graph.yaml"))
            .expect("the verb reads `graph.yaml` through the gathered run_dir pointer");
    assert!(
        graph_yaml.contains("name: perception"),
        "the gathered pointer must lead to THIS run's effective graph, got: {graph_yaml:?}"
    );
    assert!(
        std::path::Path::new(&seen.run_dir)
            .join("run.json")
            .is_file(),
        "the gathered pointer must lead to a run directory carrying `run.json`"
    );

    // Exactly one, and settled: an isolated namespace carries this run and
    // nothing else, so the answer is evidence rather than a snapshot.
    let gather = serve_side_gather(&host, WINDOW);
    assert_eq!(
        gather.records.len(),
        1,
        "an isolated namespace with one live run must gather exactly one record, got {:?}",
        gather.records,
    );
    assert_eq!(
        gather.completeness,
        GatherCompleteness::Settled,
        "every live writer was heard, so the answer settles the question"
    );
}

/// The **production** run-announcement path, end to end: a record minted, a
/// directory written and a writer published by `run_dir::start_run_descriptor`
/// itself is gathered by a netd-shaped host.
///
/// Every other arm publishes a hand-built record through
/// `RunHandle::publish_on_config`, because that is the ONLY config-parametrized
/// publish seam and the isolation the namespace arms need depends on it —
/// `start_run_descriptor` calls `RunHandle::publish`, which takes no config and
/// resolves `Config::global_config()`. So this arm is the one that pays for
/// production parity, and it pays in full: it runs on the GLOBAL namespace,
/// which is why it is `#[serial]` and why it asserts only about ITS OWN
/// `run_id`, never that the namespace is otherwise empty (a developer box may
/// have a real `graph run` live).
///
/// What it adds over arm 1, precisely: arm 1 proves a record on the host's
/// namespace is reachable; this proves the record PRODUCTION ACTUALLY EMITS —
/// its own minted `run_id`, its own `run_dir` string, its own `RunState`, its
/// own `supervisor_pid`, written by the real directory writer — survives the
/// same gather and lands on a directory the verb can read. Nothing here is modelled.
///
/// It is deliberately NOT sensitive to a namespace mismatch (its host is on the
/// global namespace, which is where the writer is): that discrimination is arm
/// 3's job. This arm answers a different question — does the serve side see what
/// production writes?
///
/// `CERULION_HOME` is redirected to a tempdir so the run directory lands there
/// instead of the developer's `~/.cerulion/runs`; the SHM side cannot be
/// contained the same way, and that residual is stated rather than hidden.
#[test]
#[serial_test::serial]
fn a_production_minted_run_record_is_gatherable_by_a_netd_shaped_host() {
    let home = tempfile::tempdir().expect("CERULION_HOME");
    let _home_guard = HomeGuard::set(home.path());

    // The host netd builds — on the SAME namespace `RunHandle::publish` resolves.
    let host = netd_shaped_host(
        "prod_host",
        &iceoryx2::config::Config::global_config().clone(),
    );

    let started_at = u64::try_from(nanos()).expect("ns fits u64");
    let descriptor = cerulion_cli_engine::run_dir::start_run_descriptor(
        cerulion_cli_engine::run_dir::RunDescriptorSpec {
            // The run IDENTITY is minted by the caller, before and
            // independently of this write — the capture plane is named from it,
            // and descriptor creation is deliberately never-fatal, so an identity
            // that only existed on the success path took the always-on black box
            // down with any failed bookkeeping. `descriptor.run_id()` below reads
            // back exactly this value, so the gather oracle is unchanged.
            run_id: cerulion_cli_engine::run_dir::mint_run_id(),
            graph_name: "prod",
            run_started_at_ns: started_at,
            network: cerulion_cli_engine::run_dir::NetworkPostureLabel::Permissive,
            partition: cerulion_cli_engine::run_dir::PartitionProvenance::Declared,
            process_groups: false,
            shm: Vec::new(),
            gating: cerulion_cli_engine::run_dir::GatingClock::Wall,
            graph_yaml: "name: prod\nprefix: prod\nnodes: []\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        },
    )
    .expect("the production run-descriptor path");

    assert!(
        descriptor.is_discoverable(),
        "the production path must have published a registry writer — without one this arm proves \
         nothing about the gather"
    );

    let seen = gather_until_run(
        &host,
        descriptor.run_id(),
        "the PRODUCTION-minted run record to appear to a netd-shaped host",
    );

    // Every field compared against what PRODUCTION put there, not against a
    // constant this test also wrote.
    assert_eq!(seen.run_id, descriptor.run_id());
    assert_eq!(seen.graph_name, "prod");
    assert_eq!(seen.state, RunState::Live);
    assert_eq!(seen.supervisor_pid, std::process::id());
    assert_eq!(
        seen.run_started_at_ns, started_at,
        "the production writer carries the spec's start timestamp onto the wire"
    );
    assert_eq!(
        seen.run_dir,
        descriptor.path().display().to_string(),
        "the gathered pointer must be the directory the production writer created"
    );
    assert!(
        descriptor.path().starts_with(home.path()),
        "the run directory must have landed under the redirected CERULION_HOME, not the \
         developer's real one"
    );

    // The verb's read, over a directory nothing in this test wrote.
    let graph_yaml =
        std::fs::read_to_string(std::path::Path::new(&seen.run_dir).join("graph.yaml"))
            .expect("the verb reads `graph.yaml` from the production-written run directory");
    assert!(graph_yaml.contains("name: prod"), "got: {graph_yaml:?}");
    assert!(
        std::path::Path::new(&seen.run_dir)
            .join("run.json")
            .is_file(),
        "the production run directory must carry `run.json`"
    );

    // And the production teardown: the record goes, and so does the directory.
    let run_dir_path = descriptor.path().to_path_buf();
    drop(descriptor);
    let after = serve_side_gather(&host, WINDOW);
    assert!(
        after.records.iter().all(|r| r.run_id != seen.run_id),
        "a dropped RunDescriptor must leave no record: {:?}",
        after.records
    );
    assert!(
        !run_dir_path.exists(),
        "a dropped RunDescriptor must remove its run directory (`{}` survived)",
        run_dir_path.display()
    );
}

// ===========================================================================
// SHAPE 2 — the per-run gateway CHILD, on the namespace its handoff carried
// ===========================================================================

/// The per-run gateway child reaches the run on the namespace it was HANDED.
///
/// This is the same `TransportManager::detached_with_config` construction
/// `graph_cmd::graph_run_gateway` performs on the handoff's `ix_config_json`. The
/// arm also pins the equality the whole design rests on, in the child's own
/// terms: the host's `iox_shm_identity()` IS the writer's namespace identity.
#[test]
fn a_per_run_gateway_child_reaches_the_run_on_the_namespace_it_was_handed() {
    let ns = iceoryx_test_config();
    let host = child_shaped_host("gw_child", &ns);

    assert_eq!(
        host.iox_shm_identity(),
        identity_of(&ns),
        "the child's manager must be on the namespace its handoff carried — if this drifts, the \
         serve side is gathering somewhere the run never published"
    );

    let run_id = fresh_run_id();
    let fixture = RunFixture::new(run_id, "nav");
    let _run = RunHandle::publish_on_config(&ns, fixture.record()).expect("publish the run");

    let seen = gather_until_run(
        &host,
        run_id,
        "the run to appear to the per-run gateway child",
    );
    assert_eq!(seen.run_id, run_id);
    assert_eq!(seen.graph_name, "nav");
    assert_eq!(seen.state, RunState::Live);
}

// ===========================================================================
// THE HOST-CONFIG ARM — the serve side must thread the HOST's config
// ===========================================================================

/// A gather sees a run if and only if it runs on the host's OWN namespace.
///
/// **This is the arm that catches a wrong-config substitution**, and it exists in this shape
/// because the obvious alternative cannot: "thread the global config
/// instead" is indistinguishable from correct on a multi-process run, since
/// that run's data plane sits ON the global config (see the module docs).
/// A host that is NOT on the global namespace is the discriminating shape — and
/// it is a real one: it is what a per-run gateway child looks like after a
/// genuine `IOX2_CONFIG_FILE` divergence, which is the only thing that still
/// produces netd's `NamespaceMismatch`.
///
/// Two independent oracles, so neither half can carry the arm alone:
///
/// 1. **The isolation matrix.** Two hosts, two namespaces, one run on each. Each
///    host sees its own run and NOT its neighbour's. A serve side threading any
///    single fixed config answers identically for both hosts and fails here.
/// 2. **The literal substitution.** A gather on `Config::global_config()` — the memo's
///    named substitution — does not carry either run. Asserted as "does not contain
///    these run ids" rather than "is empty", so a developer box with a live
///    `graph run` cannot make it flake.
#[test]
fn only_a_gather_threading_the_hosts_own_config_sees_that_hosts_run() {
    let ns_a = iceoryx_test_config();
    let ns_b = iceoryx_test_config();
    assert_ne!(
        identity_of(&ns_a),
        identity_of(&ns_b),
        "the two isolated namespaces must actually differ, or this arm proves nothing"
    );

    let host_a = child_shaped_host("host_a", &ns_a);
    let host_b = child_shaped_host("host_b", &ns_b);

    let run_a = fresh_run_id();
    let run_b = fresh_run_id();
    let fix_a = RunFixture::new(run_a, "graph_a");
    let fix_b = RunFixture::new(run_b, "graph_b");
    let _a = RunHandle::publish_on_config(&ns_a, fix_a.record()).expect("publish A");
    let _b = RunHandle::publish_on_config(&ns_b, fix_b.record()).expect("publish B");

    let seen_a = gather_until_run(&host_a, run_a, "host A's own run");
    let seen_b = gather_until_run(&host_b, run_b, "host B's own run");
    assert_eq!(seen_a.graph_name, "graph_a");
    assert_eq!(seen_b.graph_name, "graph_b");

    // Oracle 1: neither host can see the other's run.
    let gather_a = serve_side_gather(&host_a, WINDOW);
    let gather_b = serve_side_gather(&host_b, WINDOW);
    assert!(
        gather_a.records.iter().all(|r| r.run_id != run_b),
        "host A gathered host B's run — the serve side is not threading A's own namespace: {:?}",
        gather_a.records,
    );
    assert!(
        gather_b.records.iter().all(|r| r.run_id != run_a),
        "host B gathered host A's run — the serve side is not threading B's own namespace: {:?}",
        gather_b.records,
    );
    assert_eq!(gather_a.records.len(), 1, "host A sees exactly its own run");
    assert_eq!(gather_b.records.len(), 1, "host B sees exactly its own run");

    // Oracle 2: the literal substitution. Neither run is reachable from the global
    // namespace, so a serve side that threaded it would answer a confident
    // "no runs" to a robot that is running one.
    let global = gather_runs_on_config(iceoryx2::config::Config::global_config(), WINDOW)
        .expect("gather on the global namespace");
    assert!(
        global
            .records
            .iter()
            .all(|r| r.run_id != run_a && r.run_id != run_b),
        "a gather on the GLOBAL namespace reported a run published on an isolated one — the \
         namespace keying this whole design depends on does not hold: {:?}",
        global.records,
    );
}

// ===========================================================================
// THE PRECONDITION — over PRODUCTION values, not a model
// ===========================================================================

/// The config a multi-process supervisor hands its workers and its gateway child
/// resolves the SAME SHM namespace the run registry writer publishes on.
///
/// Every other arm in this file substitutes an isolated namespace for the global
/// one, which preserves the relationship under test but not the production
/// VALUES. This arm has no model in it: it reads
/// `multiprocess::mint_deployment_ix_config` — the real function
/// `graph_cmd`'s supervisor calls — and compares its SHM identity against
/// `Config::global_config()`, which is where `RunHandle::publish` puts the
/// record on every path.
///
/// It is therefore the drift guard for the multi-process reachability claim. If
/// the mp data plane ever goes run-scoped again, the
/// gateway child is handed a namespace the registry writer never published on,
/// and shape 2 breaks silently — a confident "no runs" on a robot that is
/// running one. This arm fails first, and says so.
#[test]
fn the_shipping_deployment_config_resolves_the_registry_writers_namespace() {
    let deployment = mint_deployment_ix_config("perception", "1700000000_4242")
        .expect("mint the multi-process deployment config");
    let writer_ns = iceoryx2::config::Config::global_config();

    assert_eq!(
        identity_of(&deployment),
        identity_of(writer_ns),
        "the multi-process deployment namespace must resolve the SAME SHM identity as \
         `RunHandle::publish`'s `Config::global_config()`. It does not, so a per-run gateway \
         child — which is built on the deployment config — would gather where the run never \
         published and report a confident `no runs`. (An earlier change made these equal by moving the mp \
         data plane onto the resolved global config; if that was reverted, the `runs` \
         verb needs the writer's namespace threaded explicitly instead.)"
    );

    // netd's namespace check is exactly this comparison, run over the JSON the control
    // seam carries — so a multi-process run is ACCEPTED by
    // netd's shared gateway rather than routed to a child. This is the assertion
    // that pins it.
    let forwarded = cerulion_core::ix_config_shm_identity_from_json(&config_json(&deployment))
        .expect("the forwarded config's identity");
    assert_eq!(
        forwarded,
        identity_of(writer_ns),
        "a multi-process run forwards this config to netd's `register_egress`; netd compares its \
         identity against its own (global) manager's. Equal ⇒ netd ACCEPTS and the SHARED gateway \
         serves the run"
    );
}

// ===========================================================================
// THE ROUTING PREMISE — netd's real namespace gate
// ===========================================================================

/// netd REFUSES a run whose namespace it cannot tap, which is what routes that
/// run to a per-run gateway child.
///
/// Driven through the PRODUCTION `EgressPlane::register_egress` — the refusal
/// returns before `ensure_gateway_booted`, so nothing is spawned and no session
/// opens. Paired in the same body with the anti-tautology control: handed a
/// MATCHING config the same plane does NOT answer `NamespaceMismatch` (it fails
/// later and for a different, named reason — this test's manager is
/// network-less), so the refusal is attributable to the namespace rather than to
/// a plane that refuses everything.
#[test]
fn netd_refuses_a_run_whose_namespace_it_cannot_tap() {
    let netd_ns = iceoryx_test_config();
    let run_ns = iceoryx_test_config();
    let host = netd_shaped_host("c6b", &netd_ns);
    let plane = GatewayEgressPlane::new_without_mdns_for_test(Arc::clone(&host));

    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(Vec::new()),
        announce: Vec::new(),
        ingress: Vec::new(),
    };
    let serving = SchemaServing::default();

    let refused = plane.register_egress(1, &plan, &serving, Some(&config_json(&run_ns)));
    match refused {
        Err(EgressError::NamespaceMismatch { run, netd }) => {
            assert_eq!(
                *run,
                identity_of(&run_ns),
                "the refusal names the RUN's identity"
            );
            assert_eq!(
                *netd,
                identity_of(&netd_ns),
                "the refusal names NETD's identity"
            );
        }
        other => panic!(
            "a run on a namespace netd is not on must be REFUSED so it falls back to a per-run \
             gateway child; got {other:?}"
        ),
    }

    // Anti-tautology: the SAME plane, handed its OWN namespace, does not reach
    // the namespace arm at all.
    let matching = plane.register_egress(2, &plan, &serving, Some(&config_json(&netd_ns)));
    assert!(
        !matches!(matching, Err(EgressError::NamespaceMismatch { .. })),
        "a MATCHING config must pass the namespace gate — a plane that refused everything would satisfy \
         the refusal arm above for the wrong reason; got {matching:?}"
    );
}

// ===========================================================================
// THE ABSENCE FLOOR
// ===========================================================================

/// A machine with no live run answers `Settled` empty, and does it without
/// paying the window.
///
/// This is the property that makes the `runs` verb's absence EXACT: the gather's
/// zero-publisher fast path reads the registry service's live publisher count
/// and returns immediately, so an empty answer is backed by the service itself
/// reporting that nobody is publishing — not by a window that heard nothing.
///
/// The wall assertion is deliberately coarse: a 10 s window against a 5 s
/// ceiling. An implementation that paid the window takes at least 10 s, the fast
/// path takes milliseconds, and no amount of load closes a gap that wide.
#[test]
fn a_machine_with_no_live_run_settles_empty_without_paying_the_window() {
    let ns = iceoryx_test_config();
    let host = netd_shaped_host("empty", &ns);

    let started = Instant::now();
    let gather = serve_side_gather(&host, Duration::from_secs(10));
    let elapsed = started.elapsed();

    assert!(
        gather.records.is_empty(),
        "a namespace with no live run must gather nothing, got {:?}",
        gather.records
    );
    assert_eq!(
        gather.completeness,
        GatherCompleteness::Settled,
        "the empty must be EVIDENCE — the `runs` verb renders `Incomplete` as `could not \
         establish the running graph`, never as `no runs`"
    );
    assert!(
        elapsed < Duration::from_secs(5),
        "the zero-publisher fast path must return without paying the window (10 s window, took \
         {elapsed:?}) — otherwise every desk poll of the `runs` verb costs a full gather"
    );
}

/// A run that ENDED leaves no record: the writer stops republishing, so the very
/// next gather does not carry it.
///
/// This is the other half of "absence is exact by construction", and it is what
/// lets the `runs` verb serve a live gather with no retraction protocol, no
/// staleness window and no vanish grace. (The run DIRECTORY's removal on the
/// same drop is pinned first-party by `run_dir`'s own
/// `dropping_the_descriptor_removes_the_run_directory`; it is not duplicated
/// here, and it could not be — `RunDescriptor` publishes on the process-global
/// namespace, which these arms deliberately do not use.)
#[test]
fn a_run_that_ended_leaves_no_record() {
    let ns = iceoryx_test_config();
    let host = netd_shaped_host("ended", &ns);

    let run_id = fresh_run_id();
    let fixture = RunFixture::new(run_id, "ending");
    let run = RunHandle::publish_on_config(&ns, fixture.record()).expect("publish the run");
    gather_until_run(&host, run_id, "the run to be live before it ends");

    run.set_ending();
    drop(run);

    let after = serve_side_gather(&host, WINDOW);
    assert!(
        after.records.iter().all(|r| r.run_id != run_id),
        "a dropped run has no writer, so it must not be gathered: {:?}",
        after.records
    );
    assert_eq!(
        after.completeness,
        GatherCompleteness::Settled,
        "with the only writer gone the namespace has zero publishers, so the empty is evidence"
    );
}

// ===========================================================================
// THE WRONG MANAGER — where "thread the manager's config" does not apply
// ===========================================================================

/// The multi-process supervisor's own manager is NOT a place to gather from.
///
/// "Thread the manager's config" is the rule for a gateway HOST. It is not a
/// universal one: on the multi-process path the supervisor's own
/// `TransportManager` lives on the throwaway planning namespace
/// (`multiprocess::planning_ix_config` — `cer_p_{hex}`, deliberately disjoint
/// from the data plane so its planning build's single-writer publishers cannot
/// collide with the workers'). The run record is published on the global config,
/// not there.
///
/// So a future serve side that ran inside the supervisor and reached for "the
/// manager" would gather on the planning namespace and find nothing. Pinned here
/// so the verb does not discover it: the planning namespace is a DIFFERENT identity,
/// and a run announced the way `run_dir` announces it is not visible from it.
#[test]
fn the_supervisors_planning_namespace_is_not_where_the_run_is_announced() {
    let planning = planning_ix_config("perception", &format!("{}", nanos()))
        .expect("mint the supervisor's planning config");

    assert_ne!(
        identity_of(&planning),
        identity_of(iceoryx2::config::Config::global_config()),
        "the planning namespace must stay disjoint from the data plane — the whole reason \
         for it is that the supervisor's planning build must not collide with the workers'"
    );

    // And behaviourally: a run announced on the writer's namespace is not
    // visible from the planning one.
    let writer_ns = iceoryx_test_config();
    let run_id = fresh_run_id();
    let fixture = RunFixture::new(run_id, "perception");
    let _run = RunHandle::publish_on_config(&writer_ns, fixture.record()).expect("publish the run");

    let host = netd_shaped_host("writer_ns", &writer_ns);
    gather_until_run(&host, run_id, "the run on the writer's namespace");

    let from_planning = gather_runs_on_config(&planning, WINDOW).expect("gather on planning");
    assert!(
        from_planning.records.iter().all(|r| r.run_id != run_id),
        "the run must NOT be visible from the supervisor's planning namespace — a serve side \
         reaching for `the manager` there would report a confident `no runs`: {:?}",
        from_planning.records
    );
}

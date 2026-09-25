// SPDX-License-Identifier: AGPL-3.0-only
//! The `runs` serve driven against a run directory PRODUCTION wrote.
//!
//! `cerulion_core/tests/runs_serve_iox2_test.rs` owns the verb's behaviour — the
//! absence taxonomy, the refusal, the withheld-run demotion, the namespace. Every
//! arm there builds its run directory with the module's own constants, so the one
//! thing it structurally cannot check is whether the serve reads what a real
//! `cerulion graph run` actually WRITES: rename `GRAPH_YAML_FILE` and both sides
//! of those arms rename together and stay green, while every robot in the field
//! serves an empty runs list.
//!
//! This file closes that loop, and it has to live HERE: the production writer is
//! `cerulion_cli_engine::run_dir`, which depends on `cerulion_core`, so
//! `cerulion_core`'s own tests cannot reach it. The namespace proof
//! (`runs_namespace_spike_test.rs`) establishes the shape — drive
//! `start_run_descriptor`, gather the record it publishes — and stops at the
//! gather; this carries the same run through the SERVE.
//!
//! # What is production here
//!
//! The minted `run_id`, the directory, its four artifacts, the registry record
//! and the publish are all `run_dir`'s. The gather, the fold, the run-directory
//! read, the authorizer gate and the wire codec are all the serve's. Nothing in the file
//! writes a file the serve then reads, which is the property the crafted arms give up.
//!
//! The file names are deliberately spelled as LITERALS rather than through
//! `run_artifacts`' constants — an assertion routed through the constant under
//! test cannot see it move.
//!
//! # Discipline
//!
//! `#[serial]`: `start_run_descriptor` publishes on the process-GLOBAL iceoryx2
//! namespace (it takes no config) and mutates `CERULION_HOME`.
//! Assertions are scoped to this run's own `run_id`, never to the namespace being
//! otherwise empty, so a developer's live `graph run` cannot fail them. Every wait
//! is a bounded CONDITION with a seconds-scale ceiling, never a wall in units of
//! the registry's republish interval.

#![cfg(unix)]

use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_cli_engine::run_dir::{
    mint_run_id, start_run_descriptor, NetworkPostureLabel, PartitionProvenance, RunDescriptorSpec,
};
use cerulion_core::transport::cerulion_q::{decode_runs_reply, encode_runs_reply, format_run_id};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, RealClock, TransportConfig, TransportManager,
};
use serial_test::serial;

/// The robot identity the gateway serves under.
const ROBOT: &str = "prodsrv";

/// Liveness ceiling for the one wait: the registry republishes every 150 ms, so a
/// healthy run is heard in milliseconds and this is only the unhealthy bound.
const HEARD_CEILING: Duration = Duration::from_secs(20);

/// Restores `CERULION_HOME` on drop, so a panicking arm cannot leave the
/// developer's real home redirected (crib: `runs_namespace_spike_test.rs`).
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

/// A gateway on the PROCESS-GLOBAL namespace — the one `RunHandle::publish`
/// resolves, and therefore the only one that can see a production-written run.
fn gateway_on_the_global_namespace(tag: &str) -> GatewayRuntime {
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("prodsrv_{tag}"),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 16,
            network: Some(NetworkConfig {
                robot_identity: Some(ROBOT.to_string()),
                ..NetworkConfig::default()
            }),
        },
        iceoryx2::config::Config::global_config().clone(),
    )
    .expect("gateway host manager on the global namespace");
    GatewayRuntime::new(
        manager,
        GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: Vec::new(),
            ingress: Vec::new(),
        },
    )
    .expect("gateway boot")
}

/// THE production-path loop: a run written by `cerulion graph run`'s own
/// descriptor writer is SERVED by the `runs` verb, artifacts verbatim.
///
/// The load-bearing claim is that the serve reads the directory production WRITES —
/// which file names, which contents. Every other arm in the serve's suite
/// builds the directory itself through the same constants the serve reads, so a rename
/// on either side moves both and nothing fails; here the writer is
/// `run_dir::start_run_descriptor` and the reader is the serve, with no shared
/// constant between them.
#[test]
#[serial]
fn a_run_written_by_the_production_descriptor_is_served_with_its_artifacts_verbatim() {
    let home = tempfile::tempdir().expect("CERULION_HOME");
    let _home_guard = HomeGuard::set(home.path());

    let gateway = gateway_on_the_global_namespace("served");

    let started_at = u64::try_from(
        SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .expect("clock")
            .as_nanos(),
    )
    .expect("ns fits u64");
    // The graph document production is asked to render. It is compared back out
    // of the SERVED reply, so the assertion is "the wire carries what the run
    // executes", not "the wire carries what this test wrote to disk".
    let graph_yaml = "name: served\nprefix: served\nnodes: []\n";

    let descriptor = start_run_descriptor(RunDescriptorSpec {
        chains: None,
        run_id: mint_run_id(),
        graph_name: "served",
        run_started_at_ns: started_at,
        network: NetworkPostureLabel::Permissive,
        partition: PartitionProvenance::Declared,
        process_groups: false,
        shm: Vec::new(),
        gating: cerulion_cli_engine::run_dir::GatingClock::Wall,
        graph_yaml: graph_yaml.to_string(),
        env_json: b"{}".to_vec(),
        recorder_json: b"{}".to_vec(),
    })
    .expect("the production run-descriptor path");
    assert!(
        descriptor.is_discoverable(),
        "the production path must have published a registry writer — without one this arm \
         proves nothing about the serve"
    );

    // Wait on a CONDITION: this run's own id appearing in the served list.
    let want = format_run_id(descriptor.run_id());
    let deadline = Instant::now() + HEARD_CEILING;
    let mut reply = gateway.runs_serve_for_test(ROBOT);
    while !reply.runs.iter().any(|r| r.run_id == want) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        reply = gateway.runs_serve_for_test(ROBOT);
    }
    let entry = reply
        .runs
        .iter()
        .find(|r| r.run_id == want)
        .unwrap_or_else(|| {
            panic!(
                "the PRODUCTION-written run {want} was never served within {HEARD_CEILING:?} \
                 — completeness {:?}, withheld {:?}, error {:?}",
                reply.completeness, reply.undescribable, reply.error
            )
        });

    // Identity + metadata, against what PRODUCTION minted.
    assert_eq!(entry.graph_name, "served");
    assert_eq!(entry.run_started_at_ns, started_at);
    assert_eq!(
        entry.state,
        cerulion_core::transport::cerulion_q::RunEntryState::Live
    );

    // THE artifact claims. `graph_yaml` is compared against the document handed
    // to the spec — so the whole chain (render → write → read → wire) is pinned
    // end to end rather than at either end.
    assert_eq!(
        entry.graph_yaml, graph_yaml,
        "the served graph.yaml must be the effective document the run executes, VERBATIM"
    );
    // …and against the bytes on disk, read through a LITERAL filename: an
    // assertion routed through `run_artifacts::GRAPH_YAML_FILE` could not see
    // that constant move away from what production writes.
    let on_disk = std::fs::read_to_string(descriptor.path().join("graph.yaml"))
        .expect("production wrote graph.yaml");
    assert_eq!(entry.graph_yaml, on_disk);

    let run_json_on_disk = std::fs::read_to_string(descriptor.path().join("run.json"))
        .expect("production wrote run.json");
    assert_eq!(
        entry.run_json, run_json_on_disk,
        "the served run.json must be the manifest production wrote, VERBATIM"
    );
    // A structural check on the manifest, so "verbatim" cannot be satisfied by
    // two identically-empty strings.
    let manifest: serde_json::Value =
        serde_json::from_str(&entry.run_json).expect("the served manifest parses as JSON");
    assert_eq!(
        manifest.get("run_id").and_then(serde_json::Value::as_str),
        Some(want.as_str()),
        "the manifest names the same run the wire does: {}",
        entry.run_json
    );

    // The answer is well formed and survives its own decode gate.
    assert_eq!(reply.error, None);
    assert!(
        reply.undescribable.is_empty(),
        "a production-written run directory is describable: {:?}",
        reply.undescribable
    );
    reply.validate().expect("a served reply satisfies the gate");
    assert_eq!(
        decode_runs_reply(&encode_runs_reply(&reply)).expect("round trip"),
        reply
    );
}

/// A run that ENDED is gone from the answer — the absence half, over the
/// production lifecycle rather than a hand-dropped writer.
///
/// `RunDescriptor::drop` announces `Ending` and REMOVES the directory, which is
/// exactly what makes the serve-time gather exact for free (a
/// dead run has no registry writer AND no run dir). That claim is the reason the
/// verb gathers instead of caching, and nothing had exercised it through the real
/// teardown.
///
/// Its PRECONDITION is the same run being served first, in the same body —
/// without it "absent" is satisfied by a serve that never found the run at all.
#[test]
#[serial]
fn a_run_that_ended_leaves_no_trace_in_the_served_answer() {
    let home = tempfile::tempdir().expect("CERULION_HOME");
    let _home_guard = HomeGuard::set(home.path());

    let gateway = gateway_on_the_global_namespace("ended");

    let descriptor = start_run_descriptor(RunDescriptorSpec {
        chains: None,
        run_id: mint_run_id(),
        graph_name: "ending",
        run_started_at_ns: 42,
        network: NetworkPostureLabel::Permissive,
        partition: PartitionProvenance::Declared,
        process_groups: false,
        shm: Vec::new(),
        gating: cerulion_cli_engine::run_dir::GatingClock::Wall,
        graph_yaml: "name: ending\nprefix: ending\nnodes: []\n".to_string(),
        env_json: b"{}".to_vec(),
        recorder_json: b"{}".to_vec(),
    })
    .expect("the production run-descriptor path");
    let want = format_run_id(descriptor.run_id());
    let dir = descriptor.path().to_path_buf();

    // PRECONDITION: it really is served while live.
    let deadline = Instant::now() + HEARD_CEILING;
    let mut reply = gateway.runs_serve_for_test(ROBOT);
    while !reply.runs.iter().any(|r| r.run_id == want) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        reply = gateway.runs_serve_for_test(ROBOT);
    }
    assert!(
        reply.runs.iter().any(|r| r.run_id == want),
        "the run must be served while LIVE, or its later absence proves nothing"
    );

    // The PRODUCTION teardown: announces `Ending`, then removes the directory.
    drop(descriptor);
    assert!(
        !dir.exists(),
        "the production teardown removes the run directory — the other half of why \
         serve-time absence is exact"
    );

    let deadline = Instant::now() + HEARD_CEILING;
    let mut after = gateway.runs_serve_for_test(ROBOT);
    while after.runs.iter().any(|r| r.run_id == want) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        after = gateway.runs_serve_for_test(ROBOT);
    }
    assert!(
        !after.runs.iter().any(|r| r.run_id == want),
        "an ENDED run must leave the answer — no retraction protocol, no staleness window"
    );
    // And it must not linger as a WITHHELD row either: the run is gone, not
    // undescribable, so nothing about it belongs on the wire at all.
    assert!(
        !after.undescribable.iter().any(|w| w.run_id == want),
        "an ended run is ABSENT, not withheld: {:?}",
        after.undescribable
    );
}

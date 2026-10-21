// SPDX-License-Identifier: AGPL-3.0-only
//! A gateway start builds its run while LAN access stays paired and opens it
//! only at the commit; a stop or a failed start returns the process manager
//! to the quiescent baseline. Own binary: the transport manager and the
//! runtime's process-lifetime transport latch are process-global, so every
//! test here uses the same listen list and robot and runs under one lock.

use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};

use cerulion_android::runtime::gateway_owned_for_test;
use cerulion_android::{EmbedError, EmbeddedRuntime};
use cerulion_core::transport::TransportManager;
use cerulion_core::LanAccess;
use serde_json::json;
use tempfile::{tempdir, TempDir};

const ROBOT: &str = "gateway_restart_baseline";

static SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|error| error.into_inner())
}

fn topic(name: &str) -> String {
    format!("/{ROBOT}/{name}")
}

fn config(topic: &str, lan_access: &str) -> String {
    json!({
        "listen": ["tcp/127.0.0.1:0"],
        "lan_access": lan_access,
        "mdns": false,
        "publish": [{"topic": topic, "schema": "std_msgs/String"}]
    })
    .to_string()
}

fn runtime() -> (TempDir, EmbeddedRuntime) {
    let files = tempdir().expect("app files directory");
    let runtime =
        EmbeddedRuntime::new(PathBuf::from(files.path()), ROBOT.to_string()).expect("runtime");
    (files, runtime)
}

fn registered(manager: &TransportManager) -> Vec<String> {
    let mut topics: Vec<String> = manager
        .bridge_manager()
        .registered_topics()
        .expect("bridge registry")
        .into_iter()
        .map(|(topic, _flag)| topic)
        .collect();
    topics.sort();
    topics
}

fn announced(manager: &TransportManager) -> Vec<String> {
    let mut topics = manager.network().expect("network").announced_topics();
    topics.sort();
    topics
}

/// The quiescent baseline: paired, no egress announce, no bridge flag, the
/// latched serving empty and the gateway lease free.
fn assert_baseline(manager: &TransportManager) {
    let network = manager.network().expect("network");
    assert_eq!(network.lan_access(), LanAccess::Paired);
    assert_eq!(announced(manager), Vec::<String>::new());
    assert_eq!(registered(manager), Vec::<String>::new());
    let live = network
        .query_surface_serving_handles()
        .expect("a gateway start latched the query surface");
    assert!(live.is_serving_nothing_for_test());
    assert!(!gateway_owned_for_test());
}

#[test]
fn stop_returns_the_manager_to_the_quiescent_baseline() {
    let _serial = serial();
    let (_files, mut runtime) = runtime();
    let a = topic("stop_a");
    runtime.start(&config(&a, "open")).expect("start open");
    let manager = TransportManager::get().expect("transport manager");
    assert_eq!(announced(&manager), vec![a.clone()]);
    assert_eq!(registered(&manager), vec![a]);
    assert!(gateway_owned_for_test());

    runtime.stop().expect("stop");

    assert_baseline(&manager);
}

/// A restart never pairs the new run's schema snapshot with the stopped
/// run's topics: while the restart builds, the posture is paired, the
/// registry holds only the new topic and the serving names no old topic.
#[test]
fn a_restart_never_pairs_the_new_snapshot_with_prior_run_topics() {
    let _serial = serial();
    let (_files, mut runtime) = runtime();
    let a = topic("restart_a");
    let b = topic("restart_b");
    runtime.start(&config(&a, "open")).expect("start /a");
    runtime.stop().expect("stop /a");

    let seen = Arc::new(Mutex::new(None));
    let hook_seen = Arc::clone(&seen);
    runtime.set_before_commit_hook_for_test(move |manager| {
        let network = manager.network().expect("network");
        let live = network
            .query_surface_serving_handles()
            .expect("latched query surface");
        *hook_seen.lock().unwrap() = Some((
            network.lan_access(),
            registered(manager),
            live.served_topics_for_test(),
        ));
    });
    runtime.start(&config(&b, "open")).expect("start /b");

    let (posture, registry, (boot, bound)) = seen.lock().unwrap().take().expect("the hook ran");
    assert_eq!(posture, LanAccess::Paired);
    assert_eq!(registry, vec![b.clone()]);
    assert_eq!(boot, vec![b.clone()]);
    assert_eq!(bound, vec![b.clone()]);
    let manager = TransportManager::get().expect("transport manager");
    assert_eq!(announced(&manager), vec![b]);
    runtime.stop().expect("stop /b");
}

/// An open start keeps LAN access paired, with nothing announced, until the
/// commit; the commit opens it and announces every registered topic.
#[test]
fn posture_stays_paired_until_commit() {
    let _serial = serial();
    let (_files, mut runtime) = runtime();
    let open = topic("until_commit");
    let seen = Arc::new(Mutex::new(None));
    let hook_seen = Arc::clone(&seen);
    runtime.set_before_commit_hook_for_test(move |manager| {
        let network = manager.network().expect("network");
        *hook_seen.lock().unwrap() = Some((network.lan_access(), announced(manager)));
    });

    runtime.start(&config(&open, "open")).expect("start open");

    let (posture, before) = seen.lock().unwrap().take().expect("the hook ran");
    assert_eq!(posture, LanAccess::Paired);
    assert_eq!(before, Vec::<String>::new());
    let manager = TransportManager::get().expect("transport manager");
    assert_eq!(
        manager.network().expect("network").lan_access(),
        LanAccess::Open
    );
    assert_eq!(announced(&manager), registered(&manager));
    assert_eq!(announced(&manager), vec![open]);
    runtime.stop().expect("stop");
}

/// The commit's announce replay fails: the start returns the error, the
/// manager is back at the baseline with the lease free, and the next start
/// succeeds.
#[test]
fn a_commit_announce_failure_rolls_back_to_baseline() {
    let _serial = serial();
    let (_files, mut runtime) = runtime();
    let failing = topic("commit_fails");
    runtime.set_before_commit_hook_for_test(|manager| {
        manager
            .network()
            .expect("network")
            .fault_inject_announce_egress_once();
    });

    let error = runtime
        .start(&config(&failing, "open"))
        .expect_err("a failed announce replay fails the start");

    assert!(matches!(error, EmbedError::Network(_)), "{error}");
    let manager = TransportManager::get().expect("transport manager");
    assert_baseline(&manager);
    runtime
        .start(&config(&failing, "open"))
        .expect("the next start succeeds");
    assert_eq!(announced(&manager), vec![failing]);
    runtime.stop().expect("stop");
    assert_baseline(&manager);
}

/// An announce fault armed before the start fires inside the gateway
/// constructor (the build), not the commit: the start fails and the
/// manager is back at the baseline.
#[test]
fn an_armed_announce_fault_fails_the_build_and_rolls_back() {
    let _serial = serial();
    let (_files, mut runtime) = runtime();
    let built = topic("build_fails");
    // A first start creates the process manager the fault is armed on.
    runtime.start(&config(&built, "paired")).expect("start");
    runtime.stop().expect("stop");
    let manager = TransportManager::get().expect("transport manager");
    let reached_commit = Arc::new(Mutex::new(false));
    let hook_reached = Arc::clone(&reached_commit);
    runtime.set_before_commit_hook_for_test(move |_| *hook_reached.lock().unwrap() = true);
    manager
        .network()
        .expect("network")
        .fault_inject_announce_egress_once();

    let error = runtime
        .start(&config(&built, "open"))
        .expect_err("the constructor's announce fails the build");

    assert!(matches!(error, EmbedError::Network(_)), "{error}");
    assert!(
        !*reached_commit.lock().unwrap(),
        "the fault must fail the build"
    );
    assert_baseline(&manager);
}

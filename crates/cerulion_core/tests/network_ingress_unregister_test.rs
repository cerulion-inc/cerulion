// SPDX-License-Identifier: AGPL-3.0-only
//! `TransportManager::unregister_ingress_topic` — the
//! core-level teardown of a network→local INGRESS bridge (the inverse of
//! `register_ingress_topic`), the primitive `cerulion-netd`'s refcount-0 path
//! drives when the last consumer of a mirror leaves.
//!
//! Pins over REAL iceoryx2 + a lazy scouting-off zenoh session (per-test SHM
//! roots + isolated sessions ⇒ parallel-safe, NO `#[serial]`), against HAND
//! oracles (never a self-compare):
//!
//! - teardown RELEASES the bridge — `is_ingress_registered`/`ingress_stats` go
//!   away AND the mirror's local SHM data service is released (a fresh
//!   `create_subscriber_open_only` errs), so the single iceoryx2 publisher slot
//!   is free for a clean re-register;
//! - the topic leaves the `self_ingress` exclusion set
//!   (`is_self_ingress` flips false) — a produced topic of the same name is once
//!   again a legitimate egress candidate;
//! - the FULL CYCLE — register → unregister → register again SUCCEEDS (a
//!   non-torn-down bridge would hit the "already registered" refusal; the second
//!   register succeeding IS the mutation-kill for a teardown that forgot to
//!   remove the ingress entry);
//! - unregister of a NEVER-registered topic (and a double-unregister) errors
//!   LOUDLY, never silently;
//! - a network-less manager errors (mirrors `register_ingress_topic`);
//! - unregistering ONE of two leaves the OTHER intact (no cross-talk).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;

/// An arbitrary wire schema hash — registration stores it to validate INBOUND
/// frames (none arrive here), so any value exercises the path.
const HASH: u64 = 0x0BAD_F00D_DEAD_BEEF;
const SLICE: MaxSliceLen = MaxSliceLen::const_new(256);

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// A per-test manager WITH a lazy (scouting-off, ingress-only) network — netd's
/// desk shape — plus a unique canonical topic. Isolated SHM root + session.
fn setup_with_network(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("unreg_{base}_{id}"),
            network: Some(NetworkConfig::default()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test with network");
    (transport, format!("/unreg/{base}/{id}"))
}

/// A per-test manager with NO network (the `CERULION_NETD_NETWORK=off` shape).
fn setup_no_network(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("unreg_nonet_{base}_{id}"),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test no network");
    (transport, format!("/unreg/{base}/{id}"))
}

#[test]
fn register_then_unregister_releases_the_bridge_and_clears_self_ingress() {
    let (t, topic) = setup_with_network("release");
    let net = || t.network().expect("has network");

    t.register_ingress_topic(&topic, HASH, SLICE)
        .expect("register the ingress bridge");

    // Registered: the bridge is in the ingress map, the topic is in the
    // self_ingress exclusion set, stats exist, and the mirror's local SHM data
    // service exists (a consumer can open it).
    assert!(net().is_ingress_registered(&topic), "ingress registered");
    assert!(net().is_self_ingress(&topic), "in the self_ingress set");
    assert!(net().ingress_stats(&topic).is_some(), "stats live");
    assert!(
        t.create_subscriber_open_only(&topic).is_ok(),
        "the mirror's data service exists after register"
    );

    // Tear it down.
    t.unregister_ingress_topic(&topic)
        .expect("unregister the ingress bridge");

    // Released on every observable: the ingress map, the self_ingress set (RIDER
    // 1), the stats, AND the local SHM data service (opening it now errs — the
    // publisher slot is free for a clean re-register).
    assert!(
        !net().is_ingress_registered(&topic),
        "ingress removed after unregister"
    );
    assert!(
        !net().is_self_ingress(&topic),
        "the topic left the self_ingress exclusion set"
    );
    assert!(net().ingress_stats(&topic).is_none(), "stats gone");
    assert!(
        t.create_subscriber_open_only(&topic).is_err(),
        "the mirror's data service was released — no phantom service survives teardown"
    );
}

#[test]
fn register_unregister_register_full_cycle_succeeds() {
    // THE acceptance: a full demand→mirror-up→teardown→re-demand→mirror-up-again
    // cycle at the core layer. Pre-teardown a second register would REFUSE
    // ("already has a registered network ingress bridge"); the second register
    // SUCCEEDING is the mutation-kill for an unregister that forgot to remove the
    // ingress entry (or failed to release the SHM publisher slot).
    let (t, topic) = setup_with_network("cycle");

    t.register_ingress_topic(&topic, HASH, SLICE)
        .expect("first register");
    assert!(t.create_subscriber_open_only(&topic).is_ok());

    t.unregister_ingress_topic(&topic).expect("teardown");
    assert!(
        t.create_subscriber_open_only(&topic).is_err(),
        "teardown released the mirror before the re-register"
    );

    // Re-register the SAME topic — succeeds cleanly (NOT the double-register
    // refusal, because the first bridge was genuinely torn down).
    t.register_ingress_topic(&topic, HASH, SLICE)
        .expect("re-register after teardown succeeds (the full cycle)");
    assert!(
        t.network().expect("net").is_ingress_registered(&topic),
        "the mirror is back up"
    );
    assert!(
        t.create_subscriber_open_only(&topic).is_ok(),
        "the re-created mirror's data service exists again"
    );
}

#[test]
fn unregister_of_a_never_registered_topic_errors_loudly() {
    let (t, topic) = setup_with_network("never");
    // Never registered → LOUD error, not a silent no-op.
    let err = t
        .unregister_ingress_topic(&topic)
        .expect_err("unregister of a never-registered topic errors");
    let msg = err.to_string();
    assert!(
        msg.contains(&topic),
        "the error names the offending topic: {msg}"
    );
    assert!(
        msg.contains("no registered network ingress bridge"),
        "the error explains the cause: {msg}"
    );
}

#[test]
fn double_unregister_second_is_a_loud_error() {
    let (t, topic) = setup_with_network("double");
    t.register_ingress_topic(&topic, HASH, SLICE)
        .expect("register");
    t.unregister_ingress_topic(&topic)
        .expect("first unregister");
    // The bridge is already gone — a second teardown is a loud error, never a
    // silent success (the idempotency contract is deliberately loud).
    assert!(
        t.unregister_ingress_topic(&topic).is_err(),
        "a second unregister of an already-torn-down bridge errors"
    );
}

#[test]
fn unregister_without_network_errors() {
    let (t, topic) = setup_no_network("nonet");
    assert!(t.network().is_none(), "no network configured");
    let err = t
        .unregister_ingress_topic(&topic)
        .expect_err("no network → unregister errors");
    let msg = err.to_string();
    assert!(
        msg.contains("configured network transport"),
        "names the no-network cause: {msg}"
    );
    assert!(msg.contains(&topic), "names the topic: {msg}");
}

#[test]
fn unregister_one_of_two_leaves_the_other_intact() {
    // No cross-talk: tearing down one ingress topic must not
    // disturb another's bridge OR its self_ingress membership.
    let (t, base) = setup_with_network("two");
    let t1 = format!("{base}/a");
    let t2 = format!("{base}/b");
    let net = || t.network().expect("net");

    t.register_ingress_topic(&t1, HASH, SLICE)
        .expect("register t1");
    t.register_ingress_topic(&t2, HASH, SLICE)
        .expect("register t2");
    assert!(net().is_self_ingress(&t1) && net().is_self_ingress(&t2));
    assert!(net().is_ingress_registered(&t1) && net().is_ingress_registered(&t2));

    t.unregister_ingress_topic(&t1).expect("unregister t1");

    // t1 fully gone; t2 completely untouched.
    assert!(!net().is_ingress_registered(&t1) && !net().is_self_ingress(&t1));
    assert!(
        net().is_ingress_registered(&t2) && net().is_self_ingress(&t2),
        "the sibling bridge and its self_ingress entry survive"
    );
    assert!(
        t.create_subscriber_open_only(&t2).is_ok(),
        "t2's mirror data service is still live"
    );
}

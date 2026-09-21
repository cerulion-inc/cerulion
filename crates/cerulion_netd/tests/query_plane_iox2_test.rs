// SPDX-License-Identifier: AGPL-3.0-only
//! The PRODUCTION query-plane path over REAL iceoryx2 + zenoh —
//! proof that `GatewayQueryPlane` is not inert (the "no inert shipping" rule).
//!
//! Builds a network-configured `TransportManager` over an ISOLATED per-test SHM root
//! (`init_for_test` + `iceoryx_test_config`) with a scouting-OFF, `robot_identity:
//! None` desk `NetworkConfig` — the `mirror_plane_iox2_test.rs` hermetic pattern. No
//! remote peer is present, so a `query_catalog` / `query_schema` gather over the ONE
//! lazy session returns an EMPTY reply list, NOT an error — the exact answer the desk
//! uses without falling back to its own transient session. The plane's session-wiring
//! and the no-network refusal are the two behaviors this pins; the per-robot GET decode
//! itself is covered end-to-end over a REAL serving gateway in
//! `cerulion_cli_engine/tests/topic_network_live_test.rs` (the query plane wraps the
//! SAME `discovery::query_robot_catalog(s)` / `query_robot_schema(s)` functions).
//!
//! ** an empty answer is authoritative only from a plane that has completed
//! a DISCOVERY PASS. A peerless test manager never completes one, so every gather here
//! is marked `DiscoveryState::NotConverged` — and the plane RE-TRIES for its
//! cold-start budget before saying so. The arms that are not about the grace inject a
//! ZERO budget (one harvest, no retries) so they stay fast; the grace itself gets its
//! own arm at the SHIPPED constant.
//!
//! Parallel-safe (per-test SHM roots + scouting-off local sessions), so NOT
//! `#[serial]` — mirrors the `mirror_plane_iox2_test.rs` convention.

use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};

use cerulion_netd::query::{
    DiscoveryState, GatewayQueryPlane, QueryError, QueryPlane, COLD_START_DISCOVERY_BUDGET,
};

/// Build a network-configured (scouting-off, ingress-only) manager over an isolated
/// per-test SHM root — netd's desk shape.
fn networked_test_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            network: Some(NetworkConfig::default()),
            ..TransportConfig::default()
        },
        iceoryx_test_config(),
    )
    .expect("init networked test manager")
}

/// Build a LOCAL-ONLY manager (no network) — the `CERULION_NETD_NETWORK=off` shape.
fn local_only_test_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            network: None,
            ..TransportConfig::default()
        },
        iceoryx_test_config(),
    )
    .expect("init local-only test manager")
}

#[test]
fn query_over_a_networked_manager_opens_the_session_and_returns_authoritatively_empty() {
    let manager = networked_test_manager("netd_query_ok");
    // Before any query the lazy session has not opened.
    assert!(
        !manager.network().expect("has network").is_active(),
        "the zenoh session is LAZY — nothing opened before the first query"
    );

    // A ZERO cold-start budget = exactly one harvest per query, no retries,
    // so these four arms (which are about session WIRING, not the grace) stay fast.
    // The grace itself is pinned at the shipped constant by the arm below.
    let plane = GatewayQueryPlane::with_cold_start_budget(Arc::clone(&manager), Duration::ZERO);

    // A LAN catalog gather with no robots present → the session opens, the announce
    // harvest finds nobody, and the plane returns an EMPTY list (not an error — the
    // desk uses this directly, no transient fallback), marked as coming from
    // a plane that has discovered nothing.
    let gather = plane.query_catalog(None).expect("catalog query runs");
    assert!(
        gather.catalogs.is_empty(),
        "no robot answered → an empty catalog list, not an error"
    );
    assert_eq!(
        gather.discovery,
        DiscoveryState::NotConverged,
        "a peerless plane has completed NO discovery pass — its empty answer \
         must not claim absence"
    );
    assert!(
        !plane.is_discovery_settled(),
        "an empty gather must not latch the plane settled"
    );
    // The plane opened the ONE shared session (proving it is wired to the manager).
    assert!(
        manager.network().expect("has network").is_active(),
        "the query opened the lazy session"
    );

    // A single-robot catalog query for an absent robot is also empty.
    let one = plane
        .query_catalog(Some("ubuntu"))
        .expect("single-robot catalog query runs");
    assert!(
        one.catalogs.is_empty(),
        "no such robot → empty (not an error)"
    );
    assert_eq!(one.discovery, DiscoveryState::NotConverged);

    // A schema fetch (both fan-out and single-robot) is empty too.
    assert!(plane
        .query_schema(None, "tf2_msgs/TFMessage")
        .expect("schema fan-out runs")
        .replies
        .is_empty());
    assert!(plane
        .query_schema(Some("ubuntu"), "tf2_msgs/TFMessage")
        .expect("single-robot schema runs")
        .replies
        .is_empty());

    // The `manager()` accessor points back at the shared manager (Arc identity).
    assert!(Arc::ptr_eq(plane.manager(), &manager));
}

#[test]
fn a_cold_plane_spends_its_grace_before_reporting_no_robots_discovered() {
    // The cold-start grace over the PRODUCTION plane, at the SHIPPED budget: netd
    // answering an empty catalog the instant its just-opened session found
    // nothing is what the desk renders as a terminal "topic not found". This
    // peerless manager is exactly that situation, permanently — so the plane must
    // (a) actually SPEND its cold-start budget re-harvesting rather than answering at
    // once, and (b) report `NotConverged`, never a confident empty.
    //
    // The wall-clock lower bound is what makes this a REGRESSION pin: `query_announce
    // _entries` breaks out as soon as its liveliness reply channel closes, which on a
    // peerless scouting-off session is microseconds — so without the grace this call
    // returns essentially instantly.
    let manager = networked_test_manager("netd_query_cold");
    let plane = GatewayQueryPlane::new(Arc::clone(&manager));

    let started = Instant::now();
    let gather = plane.query_catalog(None).expect("catalog query runs");
    let elapsed = started.elapsed();

    assert!(gather.catalogs.is_empty(), "no peer exists to answer");
    assert_eq!(
        gather.discovery,
        DiscoveryState::NotConverged,
        "the not-converged marker, not a claim of absence"
    );
    assert!(
        elapsed >= COLD_START_DISCOVERY_BUDGET,
        "the plane must RE-HARVEST across its cold-start budget before giving up \
         (took {elapsed:?}, budget {COLD_START_DISCOVERY_BUDGET:?})"
    );
    // BOUNDED — the whole point is that it never hangs a CLI verb.
    assert!(
        elapsed < COLD_START_DISCOVERY_BUDGET * 3,
        "and it stays bounded ({elapsed:?})"
    );
    assert!(!plane.is_discovery_settled());
    assert!(
        plane.is_cold_start_grace_spent(),
        "R1: spending the budget latches the daemon's one grace"
    );

    // R1 — THE per-daemon scoping, over the PRODUCTION plane. This peerless manager
    // NEVER settles (`is_discovery_settled` stays false above), so before R1 every
    // subsequent query re-paid the whole budget: `topic echo`/`hz`/`info`, `schema
    // info` and every vizd sidebar refresh each stared for ~3 s on a robot-less desk,
    // and vizd's single mutex-held `NetdClient` serialised its whole demand plane
    // behind it. The grace bridges the SESSION-ESTABLISHMENT window, which happens
    // ONCE per daemon — so it is spent once. Pinned on BOTH query verbs, since they
    // share the one plane-level grace.
    //
    // R2: the wall alone cannot distinguish "answered after ONE harvest" from
    // "short-circuited without harvesting at all" — both are instant, both empty, both
    // NotConverged. The second is a REAL hazard (it would permanently blind the desk to
    // a robot that appears later), so each post-grace query must also be shown to have
    // RUN a harvest, via the `harvests_run()` observable.
    let harvests_after_grace = plane.harvests_run();
    assert!(
        harvests_after_grace >= 2,
        "the graced first query really re-harvested (got {harvests_after_grace})"
    );

    let started = Instant::now();
    let second = plane
        .query_catalog(None)
        .expect("second catalog query runs");
    let second_wall = started.elapsed();
    assert!(second.catalogs.is_empty());
    assert_eq!(
        second.discovery,
        DiscoveryState::NotConverged,
        "still NotConverged — spending the grace once does not change the verdict"
    );
    assert!(
        second_wall < COLD_START_DISCOVERY_BUDGET,
        "a plane whose one grace is spent must answer after ONE harvest, not re-pay the \
         budget on every query (took {second_wall:?}, budget {COLD_START_DISCOVERY_BUDGET:?})"
    );
    assert_eq!(
        plane.harvests_run(),
        harvests_after_grace + 1,
        "…and it must answer after ONE harvest, not ZERO: a spent grace suppresses the \
         RETRIES, never the harvest itself — this is what keeps a robot that appears \
         AFTER the grace discoverable (it settles the plane on the next query)"
    );

    let started = Instant::now();
    let schema = plane
        .query_schema(None, "tf2_msgs/TFMessage")
        .expect("schema query runs");
    let schema_wall = started.elapsed();
    assert!(schema.replies.is_empty());
    assert_eq!(schema.discovery, DiscoveryState::NotConverged);
    assert_eq!(
        plane.harvests_run(),
        harvests_after_grace + 2,
        "the schema verb harvests exactly once too"
    );
    assert!(
        schema_wall < COLD_START_DISCOVERY_BUDGET,
        "the grace is per-PLANE, so the schema verb does not get a fresh one \
         (took {schema_wall:?})"
    );
}

#[test]
fn query_on_a_local_only_manager_is_a_loud_no_network_error() {
    // A local-only (`CERULION_NETD_NETWORK=off`) daemon has no NetworkManager, so a
    // query is an explicit QueryError::NoNetwork — NOT an empty list. The desk maps this
    // to its transient-session fallback (never a silent swallow).
    let manager = local_only_test_manager("netd_query_local");
    assert!(
        manager.network().is_none(),
        "a local-only manager has no network"
    );
    let plane = GatewayQueryPlane::new(manager);

    assert!(matches!(
        plane.query_catalog(None),
        Err(QueryError::NoNetwork)
    ));
    assert!(matches!(
        plane.query_catalog(Some("go2")),
        Err(QueryError::NoNetwork)
    ));
    assert!(matches!(
        plane.query_schema(None, "pkg/Type"),
        Err(QueryError::NoNetwork)
    ));
    assert!(matches!(
        plane.query_schema(Some("go2"), "pkg/Type"),
        Err(QueryError::NoNetwork)
    ));
    // B3: the runs verb refuses the same way — an empty list here would be
    // indistinguishable from "netd asked the LAN and no robot answered".
    assert!(matches!(plane.query_runs(None), Err(QueryError::NoNetwork)));
    assert!(matches!(
        plane.query_runs(Some("go2")),
        Err(QueryError::NoNetwork)
    ));
}

/// The DAEMON half of the first-contact gate — the plane really does
/// produce a growing un-settled age.
///
/// The whole "first contact, not every command" scoping rests on this number, and
/// before this arm NOTHING in the repository observed a REAL plane producing it: the
/// 15 client e2e arms drive a scripted `UnixListener` daemon that hand-writes the
/// field, and both `SpyQueryPlane`s hard-code `None`. So deleting the single
/// `maturity.note_attempt()` call — `first_attempt_at` is a `OnceLock` that is only
/// ever `.get()`, so a never-`set` field raises no lint — made `unsettled_for()`
/// return `None` forever, made the client's `plane_is_young` cap vacuously true, and
/// restored the ceiling-on-every-command regression the module docs call the reason
/// the gate exists, with the entire suite green. This is that arm.
///
/// Three assertions, each catching a different failure:
/// * before any query the age is `None` (the clock is anchored on real network
///   effort, not on plane construction — the session is lazy);
/// * after ONE query it is `Some(age)` with `age >= the injected budget` (the plane
///   really spent its grace before answering, and the anchor predates that spend);
/// * after a SECOND query the age has GROWN — which is what a `note_attempt` that is
///   idempotent but never CALLED cannot produce, and what a per-query re-anchor
///   (`set` → `store`) would also fail.
#[test]
fn a_never_settling_plane_reports_a_growing_unsettled_age() {
    let manager = networked_test_manager("netd_query_age");
    // A small but NON-zero budget: the age must be observably larger than something,
    // and a zero budget would leave the assertion resting on scheduler noise.
    const BUDGET: Duration = Duration::from_millis(120);
    let plane = GatewayQueryPlane::with_cold_start_budget(Arc::clone(&manager), BUDGET);

    assert_eq!(
        plane.plane_unsettled_for(),
        None,
        "the first-contact clock is anchored on the plane's first real GATHER, not on \
         construction — the zenoh session has not even opened yet"
    );

    let first = plane.query_catalog(None).expect("catalog query runs");
    assert_eq!(
        first.discovery,
        DiscoveryState::NotConverged,
        "precondition: a peerless plane never settles, so it has an age to report"
    );
    let after_one = first
        .unsettled_for
        .expect("a never-settled plane MUST report its age — `None` makes the client's cap inert");
    assert!(
        after_one >= BUDGET,
        "the age must span the grace the plane just spent ({BUDGET:?}), so the anchor \
         predates it: got {after_one:?}"
    );
    assert_eq!(
        plane.plane_unsettled_for().map(|d| d >= after_one),
        Some(true),
        "the observable and the wire field are the same quantity"
    );

    let second = plane
        .query_catalog(None)
        .expect("second catalog query runs");
    let after_two = second
        .unsettled_for
        .expect("still never settled, so still reporting");
    assert!(
        after_two > after_one,
        "the age must GROW across queries — a `note_attempt` that is never called \
         reports None forever, and one that RE-anchors per query would reset it here: \
         {after_one:?} -> {after_two:?}"
    );

    // The SCHEMA verb carries the same quantity (the gate must not be catalog-only).
    let sch = plane
        .query_schema(None, "tf2_msgs/TFMessage")
        .expect("schema query runs");
    assert!(
        sch.unsettled_for.is_some_and(|d| d >= after_two),
        "the schema verb reports the same growing plane age, or `schema info` is \
         uncapped: {:?}",
        sch.unsettled_for
    );
}

// --------------------------------------------------------------------------
// B3: the `runs` verb on the PRODUCTION plane.
// --------------------------------------------------------------------------

/// The runs verb is REAL on the production plane: it opens the ONE lazy session and
/// answers a peerless LAN with an empty list rather than an error — the same
/// no-inert-shipping proof its siblings carry.
///
/// The empty is what an empty MEANS here: no robot answered, marked `NotConverged`
/// because this plane has discovered nothing, so a consumer is forbidden from reading
/// it as "no robot is running anything".
#[test]
fn runs_over_a_networked_manager_opens_the_session_and_returns_authoritatively_empty() {
    let manager = networked_test_manager("netd_runs_ok");
    assert!(
        !manager.network().expect("has network").is_active(),
        "the zenoh session is LAZY — nothing opened before the first query"
    );

    // A ZERO cold-start budget = one harvest per query, no retries: this arm is about
    // session WIRING, and the grace itself is pinned by its own arm below.
    let plane = GatewayQueryPlane::with_cold_start_budget(Arc::clone(&manager), Duration::ZERO);

    // Fan-out: harvest the announce space, find nobody, answer empty.
    let gather = plane.query_runs(None).expect("runs query runs");
    assert!(
        gather.replies.is_empty(),
        "no robot answered → an empty list, not an error"
    );
    assert_eq!(
        gather.discovery,
        DiscoveryState::NotConverged,
        "a peerless plane has discovered nothing — its empty answer must not claim \
         that no robot is running anything"
    );
    assert!(
        manager.network().expect("has network").is_active(),
        "the runs query opened the lazy session — the plane is wired to the manager"
    );

    // Single-robot: one explicit-key GET for an absent robot is also empty, not an
    // error (the desk uses this answer directly; only a QueryError is a fallback).
    let one = plane
        .query_runs(Some("ubuntu"))
        .expect("single-robot runs query runs");
    assert!(one.replies.is_empty());
    assert_eq!(one.discovery, DiscoveryState::NotConverged);
}

/// **THE convergence-state choice, pinned.** `query_runs` reads the SAME
/// `DiscoveryMaturity` its siblings do, so a plane that a CATALOG query settled
/// answers a runs query as SETTLED — without re-paying the cold-start grace.
///
/// What that bit remembers is a fact about this DAEMON's session ("has the network
/// ever answered me?"), which belongs to the LAN and the zenoh plane rather than to a
/// verb. A private maturity would make a desk that has been talking to robots for an
/// hour pay the full grace on its first runs query, and would let two verbs on one
/// daemon give opposite answers about whether discovery works here.
///
/// The oracle is `harvests_run()` plus the reported state, not a wall: a wall tight
/// enough to separate one harvest from a re-paid grace is also tight enough for a
/// loaded runner to invert (the class).
#[test]
fn the_runs_verb_shares_the_planes_convergence_state_with_its_siblings() {
    let manager = networked_test_manager("netd_runs_share");
    // A SHORT but non-zero budget, so exhausting it is cheap and the `grace_spent`
    // latch is genuinely reached.
    let plane =
        GatewayQueryPlane::with_cold_start_budget(Arc::clone(&manager), Duration::from_millis(1));

    // A CATALOG query spends the daemon's ONE cold-start grace.
    let _ = plane.query_catalog(None).expect("catalog query runs");
    assert!(
        plane.is_cold_start_grace_spent(),
        "precondition: the catalog query spent the plane's one grace"
    );
    assert!(
        !plane.is_discovery_settled(),
        "precondition: a peerless plane never settles"
    );

    // …and the RUNS query inherits that, answering after exactly ONE fresh harvest
    // instead of re-paying a grace another verb already spent.
    let before = plane.harvests_run();
    let gather = plane.query_runs(None).expect("runs query runs");
    assert_eq!(
        plane.harvests_run() - before,
        1,
        "a runs query on a spent-grace plane runs exactly ONE harvest — more means it \
         re-paid a grace the catalog query already spent (a private maturity), and \
         ZERO would mean it short-circuited without looking, permanently blinding the \
         desk to a robot that appears later"
    );
    assert_eq!(gather.discovery, DiscoveryState::NotConverged);

    // The sharing runs the OTHER way too: the runs query's own attempt advanced the
    // plane's first-contact clock, which the catalog verb reports.
    assert!(
        plane.plane_unsettled_for().is_some(),
        "the runs gather anchors the same first-contact clock its siblings report"
    );
}
